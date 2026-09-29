# Technical guide

## Packages

```text
nav2_agent/                 Node, planning agent, validator and command-line client
  config/
    agent_params.yaml       Node parameters
    bt_catalog.yaml         Behavior Tree node catalog and action contracts
  launch/
    nav2_agent.launch.py
  nav2_agent/
    agent_node.py           ROS 2 node and ExecuteCommand action server
    bt_catalog.py           Catalog loading and Behavior Tree validation
    models.py               Plan, outcome and response models
    nav2_bridge.py          Nav2 action clients
    pydantic_agent.py       Planning and report agents
    send_command.py         Command-line client
  test/
nav2_agent_msgs/            ExecuteCommand action
docker/                     Development container
docs/                       Documentation, diagrams and media
scripts/gemma4_server.sh    Gemma 4 26B server for Jetson Thor
```

## Build

Requires ROS 2 Jazzy and Nav2.

```bash
cd ~/ros2_ws/src
git clone https://github.com/Eurecat/nav2_agent.git
python3 -m pip install pydantic pydantic-ai openai PyYAML
cd ~/ros2_ws
colcon build --packages-up-to nav2_agent
source install/setup.bash
```

## Model server

Any OpenAI-compatible server with tool calling. Example with vLLM:

```bash
vllm serve google/gemma-4-E4B-it \
  --served-model-name gemma-4-e4b \
  --host 0.0.0.0 --port 8080 --api-key EMPTY \
  --generation-config vllm \
  --enable-auto-tool-choice --tool-call-parser gemma4
```

[scripts/gemma4_server.sh](../scripts/gemma4_server.sh) serves Gemma 4 26B (GGUF) with llama.cpp on a Jetson Thor, on port 8080.

## ROS interfaces

| Name | Type | Role |
| --- | --- | --- |
| `/nav2_agent/execute_command` | `nav2_agent_msgs/action/ExecuteCommand` | Action server |
| `/user_command` | `std_msgs/msg/String` | Subscription. Sends the command to the action server |
| `/nav2_agent/status` | `std_msgs/msg/String` | Publisher. JSON status of each command |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action client |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Action client |

### ExecuteCommand

```text
string command
---
bool success
string report
string nav2_action
geometry_msgs/PoseStamped[] poses
string behavior_tree_xml
string nav2_status
uint16 error_code
string error_msg
int32 number_of_recoveries
float32 navigation_time
---
uint8 PHASE_PLANNING=1
uint8 PHASE_EXECUTING=2
uint8 PHASE_REPORTING=3
uint8 phase
string detail
float32 distance_remaining
int32 number_of_recoveries
```

One command runs at a time. A new goal preempts the active one and cancels its Nav2 goal. Canceling the action also cancels the Nav2 goal.

### Command-line client

```bash
ros2 run nav2_agent send "Move 3 meters forward, turn left 90 degrees and advance 1 more meter"
```

```text
[planning] tool_select_navigation_action
[planning] tool_make_relative_translation
[planning] tool_make_relative_turn
[planning] tool_make_relative_translation
[planning] tool_create_behavior_tree
[executing] Sending navigate_through_poses goal
[executing] 2.50 m remaining, 0 recoveries
[reporting]
navigate_through_poses: SUCCEEDED, 11.1 s, 0 recoveries
Goal reached in 11 s without recoveries.
```

Ctrl+C cancels the command. The exit code is 0 when the goal succeeds, 1 when it fails or is canceled, and 2 when the action server is unavailable or rejects the goal.

### Status topic

Each message is a JSON object with `state` and `message`. States: `ready`, `accepted`, `executing`, `reporting`, `succeeded`, `failed`, `canceled`, `rejected` and `ignored`. The final message includes the plan trace, the Nav2 outcome and the report.

```json
{
  "state": "succeeded",
  "message": "Goal reached in 11 s without recoveries.",
  "outcome": {
    "succeeded": true,
    "status": "SUCCEEDED",
    "error_code": 0,
    "error_msg": "",
    "number_of_recoveries": 0,
    "distance_remaining": 0.0,
    "navigation_time_sec": 11.1,
    "last_pose": {"frame_id": "odom", "x": 2.81, "y": 1.49, "theta": 1.74}
  },
  "report": "Goal reached in 11 s without recoveries.",
  "actions_executed": ["extract_navigation_plan", "..."],
  "trace": ["..."]
}
```

## Configuration

Parameters are set in [nav2_agent/config/agent_params.yaml](../nav2_agent/config/agent_params.yaml).

| Parameter | Default | Description |
| --- | --- | --- |
| `vllm_model_name` | `gemma-4-e4b` | Model name on the server |
| `vllm_api_base` | `http://localhost:8000/v1` | OpenAI-compatible endpoint |
| `vllm_api_key` | `EMPTY` | API key |
| `system_prompt` | built-in | Planning prompt override |
| `bt_catalog_path` | package catalog | Behavior Tree catalog file |
| `generated_bt_dir` | `/tmp/nav2_agent/behavior_trees` | Output directory for generated XML and Mermaid files |
| `navigate_to_pose_action` | `/navigate_to_pose` | Nav2 action name |
| `navigate_through_poses_action` | `/navigate_through_poses` | Nav2 action name |
| `action_server_timeout_sec` | `5.0` | Timeout waiting for Nav2 action servers |
| `agent_run_timeout_sec` | `90.0` | Timeout for one agent run |
| `dry_run_nav2` | `false` | Log Nav2 goals without sending them |
| `report_outcome` | `true` | Generate an outcome report after execution |

Launch arguments: `params_file`, `bt_catalog_path`, `dry_run_nav2`, `use_sim_time`, `agent_run_timeout_sec`, `log_level`.

## Behavior Tree catalog

[nav2_agent/config/bt_catalog.yaml](../nav2_agent/config/bt_catalog.yaml) defines the nodes the model can use and the contract of each Nav2 action. The catalog defines structural rules; the model chooses the tree structure and records its choice in the `reasoning` field.

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

`child_categories` lists the allowed node categories for each child position. `null` allows any node. The last entry applies to additional children.

### Validation

- `<root main_tree_to_execute>` with a `<BehaviorTree>` child
- Nodes present in the catalog
- Required attributes
- Child counts and `child_categories`
- Required and forbidden nodes of the action contract
- Blackboard keys consumed only after they are available

Validation errors are raised as `ModelRetry` and returned to the model.

## Tests

```bash
colcon test --packages-select nav2_agent
colcon test-result --verbose
```

The catalog and validator tests also run without ROS:

```bash
cd nav2_agent && python3 -m unittest discover -s test -p 'test_*.py'
```

## Diagrams

The diagrams in [docs/diagrams](diagrams) are generated by the scripts in [docs/diagrams/src](diagrams/src).
