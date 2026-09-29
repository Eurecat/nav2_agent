# nav2_agent technical guide

Reference for developers integrating or extending `nav2_agent`. For an overview, see the [README](../README.md).

## Package layout

```text
config/
  agent_params.yaml       Runtime parameters
  bt_catalog.yaml         Behavior Tree node catalog and action contracts
launch/
  nav2_agent.launch.py    Main launch file
nav2_agent/
  agent_node.py           ROS 2 node: command handling, execution, outcome report
  bt_catalog.py           Catalog loading, Behavior Tree validation and rendering
  models.py               Pydantic data models (plan, outcome, response)
  nav2_bridge.py          Nav2 action clients, result and feedback capture
  pydantic_agent.py       Planning agent, tools and outcome report agent
docs/diagrams/            Diagrams and their generators
scripts/gemma4_server.sh  Gemma 4 26B on Jetson Thor via llama.cpp
```

## Native build

Requires ROS 2 Jazzy and Nav2.

```bash
cd ~/ros2_ws/src
git clone https://github.com/Eurecat/nav2_agent.git
python3 -m pip install pydantic pydantic-ai openai PyYAML
cd ~/ros2_ws
colcon build --packages-select nav2_agent
source install/setup.bash
```

## Model server

Any OpenAI-compatible server with tool calling works. Example with vLLM:

```bash
vllm serve google/gemma-4-E4B-it \
  --served-model-name gemma-4-e4b \
  --host 0.0.0.0 --port 8080 --api-key EMPTY \
  --generation-config vllm \
  --enable-auto-tool-choice --tool-call-parser gemma4
```

On a Jetson Thor, [scripts/gemma4_server.sh](../scripts/gemma4_server.sh) downloads Gemma 4 26B (GGUF) and serves it with NVIDIA's llama.cpp container on port 8080.

## Configuration

Parameters live in [config/agent_params.yaml](../config/agent_params.yaml). The defaults below are the node's built-in values; the shipped YAML overrides some of them, such as the model endpoint.

| Parameter | Default | Description |
| --- | --- | --- |
| `vllm_model_name` | `gemma-4-e4b` | Model name served by the LLM server |
| `vllm_api_base` | `http://localhost:8000/v1` | OpenAI-compatible endpoint |
| `vllm_api_key` | `EMPTY` | API key for the endpoint |
| `system_prompt` | built-in | Overrides the planning prompt |
| `bt_catalog_path` | package catalog | Behavior Tree catalog YAML |
| `generated_bt_dir` | `/tmp/nav2_agent/behavior_trees` | Where generated XML and Mermaid files are written |
| `default_trigger_command` | empty | Command run by the trigger service when none was received |
| `navigate_to_pose_action` | `/navigate_to_pose` | Nav2 action name |
| `navigate_through_poses_action` | `/navigate_through_poses` | Nav2 action name |
| `action_server_timeout_sec` | `5.0` | Wait for Nav2 action servers |
| `agent_run_timeout_sec` | `90.0` | Maximum time for one agent run |
| `dry_run_nav2` | `false` | Log the Nav2 goal instead of sending it |
| `report_outcome` | `true` | Let the agent interpret the Nav2 result |

Launch arguments: `params_file`, `bt_catalog_path`, `dry_run_nav2`, `use_sim_time`, `agent_run_timeout_sec`, `log_level`.

## ROS interfaces

| Interface | Type | Direction |
| --- | --- | --- |
| `/user_command` | `std_msgs/String` | Subscribed: natural-language command |
| `/nav2_agent/trigger_command` | `std_srvs/Trigger` | Service: re-runs the last command or `default_trigger_command` |
| `/nav2_agent/status` | `std_msgs/String` (JSON) | Published: progress, outcome and report |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Action client |
| `/navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` | Action client |

### Status messages

Each message is a JSON object with a `state` and a `message`. States follow the command lifecycle: `accepted` → `executing` → `reporting` → `succeeded` or `failed`. The final message includes the plan trace, the Nav2 outcome and the agent report:

```json
{
  "state": "succeeded",
  "message": "Goal reached in 24 s. The recovery branch fired once, then FollowPath finished the path.",
  "outcome": {
    "succeeded": true,
    "status": "SUCCEEDED",
    "error_code": 0,
    "error_msg": "",
    "number_of_recoveries": 1,
    "distance_remaining": 0.0,
    "navigation_time_sec": 24.3,
    "last_pose": {"frame_id": "map", "x": 3.0, "y": 0.0, "theta": 0.0}
  },
  "report": "Goal reached in 24 s. The recovery branch fired once, then FollowPath finished the path.",
  "actions_executed": ["extract_navigation_plan", "..."],
  "trace": ["..."]
}
```

## Behavior Tree catalog

[config/bt_catalog.yaml](../config/bt_catalog.yaml) declares the building blocks the model may use and the contract of each Nav2 action. The catalog constrains structure, not strategy: the model decides whether a tree needs recovery, retry or replanning branches, and justifies it in the tree's `reasoning` field.

```yaml
nodes:
  - id: "ComputePathToPose"
    category: "action"
    purpose: "Compute a path to one target pose."
    required_attributes: ["goal", "path"]
    optional_attributes: ["planner_id"]
    consumes: ["goal"]
    produces: ["path"]

  - id: "RecoveryNode"
    category: "control"
    required_attributes: ["number_of_retries"]
    children: "exactly_two"
    child_categories: [null, ["recovery", "control", "decorator", "condition"]]

actions:
  navigate_to_pose:
    initial_blackboard: ["goal"]
    required_nodes: ["ComputePathToPose", "FollowPath"]
    forbidden_nodes: ["ComputePathThroughPoses"]
```

`child_categories` is aligned with child positions: `null` means any node, and the last rule applies to any extra children.

### Validation

Every tree is checked before it reaches Nav2:

- XML shape: `<root main_tree_to_execute>` with a `<BehaviorTree>` child
- every node exists in the catalog
- required attributes are present
- child counts and `child_categories` branch rules
- the action contract: required and forbidden nodes
- blackboard data flow: a key is consumed only after it is available or produced

Validation errors are raised to the model as `ModelRetry`, so it can repair the XML within the agent's retry budget.

## Tests

```bash
python3 -m unittest discover -s test -p 'test_*.py'
```

The tests cover the catalog and validator in pure Python; they need neither ROS, Nav2 nor an LLM server.

## Diagrams

Diagrams in [docs/diagrams](diagrams) are generated from the scripts in [docs/diagrams/src](diagrams/src); see its README to regenerate them.
