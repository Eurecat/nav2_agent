# Technical guide

## Packages

```text
nav2_agent/                 Node, planning agent, validator and command-line client
  config/
    agent_params.yaml       Node parameters
    bt_catalog.yaml         Behavior Tree node catalog and action contracts
    locations.yaml          Named locations
  launch/
    nav2_agent.launch.py
  nav2_agent/
    agent_node.py           ROS 2 node and ExecuteCommand action server
    bt_catalog.py           Catalog loading and Behavior Tree validation
    conversation.py         Context of previous commands
    locations.py            Named locations
    models.py               Plan, outcome and response models
    nav2_bridge.py          Nav2 action clients
    pydantic_agent.py       Planning and report agents
    send_command.py         Command-line client
  test/
nav2_agent_msgs/            ExecuteCommand action
docker/                     Development container
docs/                       Documentation, diagrams and media
scripts/
  run_llm_server.sh         LLM server with llama.cpp
  sim_demo.sh               TurtleBot 4 simulation demo
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

nav2_agent works with any OpenAI-compatible server that supports tool calling, such as llama.cpp or vLLM.

[scripts/run_llm_server.sh](../scripts/run_llm_server.sh) downloads Gemma 4 26B (GGUF) and serves it with llama.cpp in Docker on port 8080, the endpoint of the default configuration. The default image targets NVIDIA Jetson Thor.

| Variable | Default | Description |
| --- | --- | --- |
| `LLAMA_IMAGE` | `ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-thor` | llama.cpp server image |
| `GEMMA_DIR` | `~/.cache/huggingface/hub/ggml-org_gemma-4-26B-A4B-it-GGUF` | Model directory |
| `GEMMA_MODEL`, `GEMMA_MMPROJ` | Gemma 4 26B A4B Q4_K_M | Model and projector files |
| `HOST`, `PORT` | `0.0.0.0`, `8080` | Listen address |
| `CONTEXT_SIZE` | `8192` | Context length |
| `DOCKER_GPU_ARGS` | `--runtime nvidia` | GPU arguments for `docker run` |
| `EXTRA_LLAMA_ARGS` | empty | Additional `llama-server` arguments |
| `DRY_RUN` | `0` | Print the command without running it |

## Simulation demo

[scripts/sim_demo.sh](../scripts/sim_demo.sh) runs nav2_agent with a simulated TurtleBot 4 and Nav2 in the development container. The default simulation is Nav2's loopback simulator, shown in RViz; `--gazebo` runs the Gazebo simulation.

```bash
LLM_BASE_URL=http://<server>:8080/v1 scripts/sim_demo.sh start
scripts/sim_demo.sh send "Move 2 meters forward"
scripts/sim_demo.sh stop
```

`LLM_BASE_URL`, `LLM_MODEL` and `LLM_API_KEY` configure the model server. The demo runs Nav2 with its default parameters and `PoseProgressChecker`, which counts rotation in place as progress. Logs are written to `log/sim_demo/`.

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
bool reset_conversation
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

One command runs at a time. A new goal preempts the active one and cancels its Nav2 goal. Canceling the action also cancels the Nav2 goal. Commands that do not navigate, such as saving a location or answering a question, return `success` and `report` with an empty `nav2_action`. `reset_conversation` discards the context of previous commands.

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
[executing] 2.5 m remaining, 0 recoveries
[reporting]
navigate_through_poses: SUCCEEDED, 11.1 s, 0 recoveries
Goal reached in 11 s without recoveries.
```

Ctrl+C cancels the command. `--reset` sets `reset_conversation`. The exit code is 0 when the goal succeeds, 1 when it fails or is canceled, and 2 when the action server is unavailable or rejects the goal.

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
| `llm_model` | `gemma-4-e4b` | Model name on the server |
| `llm_base_url` | `http://localhost:8080/v1` | OpenAI-compatible endpoint |
| `llm_api_key` | `EMPTY` | API key |
| `system_prompt` | built-in | Planning prompt override |
| `global_frame` | `map` | Frame for explicit coordinates |
| `robot_base_frame` | `base_link` | Frame for relative motion |
| `locations_path` | package `locations.yaml` | Named locations file. `none` disables locations |
| `bt_catalog_path` | package catalog | Behavior Tree catalog file |
| `generated_bt_dir` | `/tmp/nav2_agent/behavior_trees` | Output directory for generated XML and Mermaid files |
| `navigate_to_pose_action` | `/navigate_to_pose` | Nav2 action name |
| `navigate_through_poses_action` | `/navigate_through_poses` | Nav2 action name |
| `action_server_timeout_sec` | `5.0` | Timeout waiting for Nav2 action servers |
| `agent_run_timeout_sec` | `90.0` | Timeout for one agent run |
| `dry_run_nav2` | `false` | Log Nav2 goals without sending them |
| `report_outcome` | `true` | Generate an outcome report after execution |
| `conversation_turns` | `5` | Previous commands kept as context |
| `conversation_timeout_sec` | `300.0` | Idle time after which the context is discarded. 0 keeps it |

Launch arguments: `params_file`, `log_level`, `bt_catalog_path`, `locations_path`, `dry_run_nav2`, `use_sim_time` and `agent_run_timeout_sec`. The last five are empty by default and override `params_file` only when set:

```bash
ros2 launch nav2_agent nav2_agent.launch.py use_sim_time:=true locations_path:=/path/to/locations.yaml
```

## Locations

`locations_path` points to a YAML file of named poses in `global_frame`. The default [locations.yaml](../nav2_agent/config/locations.yaml) contains locations on Nav2's depot map, used by the simulation demo; replace it or pass another file with `locations_path:=<file>`. The model receives the names and descriptions and resolves them with `tool_get_location`.

```yaml
locations:
  charging_station:
    x: 0.0
    y: 0.0
    theta: 0.0
    description: Robot start position.
  loading_area:
    x: 20.3
    y: 1.2
    theta: 0.0
    description: Next to the east wall.
```

Commands such as *"remember this place as the corner"* save the current robot pose, and *"forget the corner"* removes it. Saved locations are stored in `~/.ros/nav2_agent/locations.yaml`, loaded at startup and take precedence over the file above. Names are case-insensitive; spaces and hyphens are read as underscores. `theta` and `description` are optional. Relative motions are supported before the first named location of a command, not after it.

## Conversation

Each command is sent to the model with a context block: the robot pose from TF (`global_frame` to `robot_base_frame`), the known locations and the last `conversation_turns` commands with their result and start and end poses. This supports follow-up commands such as *"a bit more to the left"* or *"go back"*; the latter uses `tool_get_previous_position` with the recorded poses.

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

`success` declares how a control node succeeds: when `all` children succeed, when `any` child succeeds, or only when the `first` child succeeds, as in `RecoveryNode`.

### Validation

- `<root main_tree_to_execute>` with a `<BehaviorTree>` child
- Nodes present in the catalog
- Required attributes
- Child counts and `child_categories`
- Required and forbidden nodes of the action contract
- Required nodes run in every execution that ends in success
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
