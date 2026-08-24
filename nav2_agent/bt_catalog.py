"""Behavior Tree catalog loading, validation, and presentation utilities."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import yaml

BT_DOCUMENT_TAGS = {'root', 'BehaviorTree'}


def load_bt_catalog(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f'Behavior Tree catalog file not found: {path}')

    with path.open('r', encoding='utf-8') as yaml_file:
        raw_catalog = yaml.safe_load(yaml_file) or {}

    nodes = raw_catalog.get('nodes')
    actions = raw_catalog.get('actions')
    if nodes is None:
        raise ValueError(f'Behavior Tree node catalog must define a top-level nodes list or map: {path}')
    if not isinstance(actions, dict):
        raise ValueError(f'Behavior Tree node catalog must define a top-level actions map: {path}')

    if isinstance(nodes, list):
        node_catalog = {str(item['id']): dict(item) for item in nodes if 'id' in item}
    elif isinstance(nodes, dict):
        node_catalog = {str(node_id): dict(metadata or {}) for node_id, metadata in nodes.items()}
    else:
        raise ValueError(f'Invalid Behavior Tree node catalog format in {path}')

    action_contracts = {str(action): dict(metadata or {}) for action, metadata in actions.items()}
    if not node_catalog:
        raise ValueError(f'Behavior Tree node catalog is empty: {path}')
    if not action_contracts:
        raise ValueError(f'Behavior Tree action contracts are empty: {path}')
    return {'nodes': node_catalog, 'actions': action_contracts}


def format_catalog_reference(bt_catalog: Dict[str, Any]) -> str:
    node_lines = []
    for node_id, metadata in sorted(bt_catalog.get('nodes', {}).items()):
        parts = [f'- {node_id} ({metadata.get("category", "node")}): {metadata.get("purpose", "No purpose provided.")}']
        for label in ('children', 'required_attributes', 'optional_attributes', 'consumes', 'produces'):
            value = metadata.get(label)
            if isinstance(value, list) and value:
                parts.append(f'{label}=' + ', '.join(str(item) for item in value))
            elif value:
                parts.append(f'{label}={value}')
        node_lines.append(' '.join(parts))

    action_lines = []
    for action_name, metadata in sorted(bt_catalog.get('actions', {}).items()):
        parts = [f'- {action_name}:']
        for label in ('initial_blackboard', 'required_nodes', 'forbidden_nodes'):
            values = metadata.get(label) or []
            if values:
                parts.append(f'{label}=' + ', '.join(str(item) for item in values))
        action_lines.append(' '.join(parts))

    nodes = '\n'.join(node_lines) if node_lines else '- No Behavior Tree nodes are configured.'
    actions = '\n'.join(action_lines) if action_lines else '- No action contracts are configured.'
    return f'Available Behavior Tree nodes:\n{nodes}\n\nAction contracts:\n{actions}'


def validate_behavior_tree(
    xml_text: str,
    bt_catalog: Dict[str, Any],
    action: Optional[str] = None,
    command: str = '',
) -> None:
    root = ET.fromstring(xml_text)
    nodes = bt_catalog.get('nodes', {})
    actions = bt_catalog.get('actions', {})
    used_nodes = behavior_tree_elements(root)
    unknown_nodes = sorted({element.tag for element in used_nodes if element.tag not in nodes})
    if unknown_nodes:
        raise ValueError('Behavior Tree uses nodes not present in catalog: ' + ', '.join(unknown_nodes))

    errors: List[str] = []
    used_node_names = {element.tag for element in used_nodes}
    blackboard = set(actions.get(action, {}).get('initial_blackboard') or [])

    for element in used_nodes:
        metadata = nodes.get(element.tag, {})
        _validate_required_attributes(element, metadata, errors)
        _validate_child_count(element, metadata, errors)
        _validate_activation(element, metadata, command, errors)
        _validate_blackboard_ports(element, metadata, blackboard, errors)

    if action is not None:
        _validate_action_contract(action, actions, used_node_names, errors)

    if errors:
        raise ValueError('Behavior Tree catalog validation failed: ' + ' '.join(errors))


def behavior_tree_elements(root: ET.Element) -> List[ET.Element]:
    return [element for element in root.iter() if element.tag not in BT_DOCUMENT_TAGS]


def behavior_tree_node_summary(xml_text: str) -> List[str]:
    return [element.tag for element in behavior_tree_elements(ET.fromstring(xml_text))]


def behavior_tree_mermaid(xml_text: str) -> str:
    root = ET.fromstring(xml_text)
    lines = ['graph TD']
    counter = 0

    def add_node(element: ET.Element, parent_id: Optional[str] = None) -> None:
        nonlocal counter
        counter += 1
        node_id = f'n{counter}'
        label = element.tag
        name = element.attrib.get('name') or element.attrib.get('ID')
        if name:
            label = f'{label}: {name}'
        lines.append(f'  {node_id}["{escape_mermaid_label(label)}"]')
        if parent_id is not None:
            lines.append(f'  {parent_id} --> {node_id}')
        for child in list(element):
            add_node(child, node_id)

    add_node(root)
    return '\n'.join(lines)


def indent_text(text: str, spaces: int) -> str:
    prefix = ' ' * spaces
    return '\n'.join(prefix + line for line in text.splitlines())


def escape_mermaid_label(label: str) -> str:
    return label.replace('"', "'")


def _validate_required_attributes(element: ET.Element, metadata: Dict[str, Any], errors: List[str]) -> None:
    for attribute in metadata.get('required_attributes') or []:
        if attribute not in element.attrib:
            errors.append(f'<{element.tag}> is missing required attribute {attribute!r}.')


def _validate_child_count(element: ET.Element, metadata: Dict[str, Any], errors: List[str]) -> None:
    children_rule = metadata.get('children')
    child_count = len(list(element))
    if children_rule == 'none' and child_count != 0:
        errors.append(f'<{element.tag}> must not have children.')
    elif children_rule == 'exactly_one' and child_count != 1:
        errors.append(f'<{element.tag}> must have exactly one child, got {child_count}.')
    elif children_rule == 'exactly_two' and child_count != 2:
        errors.append(f'<{element.tag}> must have exactly two children, got {child_count}.')
    elif children_rule == 'one_or_more' and child_count < 1:
        errors.append(f'<{element.tag}> must have at least one child.')


def _validate_activation(element: ET.Element, metadata: Dict[str, Any], command: str, errors: List[str]) -> None:
    if not metadata.get('requires_explicit_request'):
        return

    normalized_command = command.casefold()
    activation_terms = [str(term).casefold() for term in metadata.get('activation_terms') or []]
    if not activation_terms or not any(term in normalized_command for term in activation_terms):
        errors.append(f'<{element.tag}> requires an explicit user request.')


def _validate_blackboard_ports(
    element: ET.Element,
    metadata: Dict[str, Any],
    blackboard: Set[str],
    errors: List[str],
) -> None:
    for attribute in metadata.get('consumes') or []:
        key = _blackboard_key(element, str(attribute), errors)
        if key is not None and key not in blackboard:
            errors.append(f'<{element.tag}> consumes {{{key}}} before it is available.')

    for attribute in metadata.get('produces') or []:
        key = _blackboard_key(element, str(attribute), errors)
        if key is not None:
            blackboard.add(key)


def _validate_action_contract(
    action: str,
    actions: Dict[str, Any],
    used_node_names: Set[str],
    errors: List[str],
) -> None:
    action_contract = actions.get(action)
    if action_contract is None:
        errors.append(f'No Behavior Tree action contract configured for action {action!r}.')
        return

    for required_node in action_contract.get('required_nodes') or []:
        if required_node not in used_node_names:
            errors.append(f'Action {action!r} requires node <{required_node}>.')

    forbidden_nodes = sorted(used_node_names & set(action_contract.get('forbidden_nodes') or []))
    if forbidden_nodes:
        errors.append(f'Action {action!r} forbids node(s): ' + ', '.join(f'<{node}>' for node in forbidden_nodes))


def _blackboard_key(element: ET.Element, attribute: str, errors: List[str]) -> Optional[str]:
    value = element.attrib.get(attribute)
    if value is None:
        return None
    if not value.startswith('{') or not value.endswith('}') or len(value) <= 2:
        errors.append(f'<{element.tag}> attribute {attribute!r} must reference a blackboard key like "{{{attribute}}}".')
        return None
    return value[1:-1]
