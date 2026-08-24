# nav2_agent

`nav2_agent` is a ROS 2 package that turns natural-language navigation commands into validated Nav2 action goals.

The node uses a local OpenAI-compatible LLM endpoint through PydanticAI. The model selects the Nav2 action, creates target poses, and authors a Behavior Tree XML document from a declarative node catalog. The ROS node validates the structured plan, writes the generated Behavior Tree to disk, and sends the corresponding `NavigateToPose` or `NavigateThroughPoses` goal to Nav2.

## Architecture

```text
/user_command or /nav2_agent/trigger_command
                  |
                  v
          nav2_agent_node
                  |
                  v
        PydanticAI planning agent
                  |
                  v
        validated NavigationPlan
                  |
                  v
        generated Behavior Tree XML
                  |
                  v
            Nav2 ActionClient
```

The LLM does not command robot motion directly. It produces a structured `NavigationPlan`; local validation and ROS 2 Action Clients form the execution boundary.

## Package Layout

```text
config/
  agent_params.yaml       Runtime parameters
  bt_catalog.yaml         Behavior Tree node catalog and action contracts
launch/
  nav2_agent.launch.py    Main launch file
nav2_agent/
  agent_node.py           ROS 2 node, command handling, logging, artifact writing
  bt_catalog.py           Behavior Tree catalog loading, rendering, and validation
  models.py               Pydantic data models
  nav2_bridge.py          Nav2 ActionClient bridge
  pydantic_agent.py       PydanticAI agent, prompt, and planning tools
```

## Requirements

- ROS 2 Jazzy
- Nav2 action servers for `/navigate_to_pose` and `/navigate_through_poses`
- Python packages: `pydantic`, `pydantic-ai`, `openai`, `PyYAML`
- A local OpenAI-compatible LLM server, for example vLLM

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Eurecat/nav2_agent.git
python3 -m pip install pydantic pydantic-ai openai PyYAML
cd ~/ros2_ws
colcon build --packages-select nav2_agent
source install/setup.bash
```

## Configuration

Runtime parameters live in [config/agent_params.yaml](config/agent_params.yaml).

Common parameters:

```yaml
nav2_agent_node:
  ros__parameters:
    vllm_model_name: "gemma-4-e4b"
    vllm_api_base: "http://localhost:8000/v1"
    vllm_api_key: "EMPTY"
    bt_catalog_path: ""
    generated_bt_dir: "/tmp/nav2_agent/behavior_trees"
    navigate_to_pose_action: "/navigate_to_pose"
    navigate_through_poses_action: "/navigate_through_poses"
    action_server_timeout_sec: 5.0
    agent_run_timeout_sec: 90.0
    dry_run_nav2: false
```

If `bt_catalog_path` is empty, the package uses [config/bt_catalog.yaml](config/bt_catalog.yaml).

## Behavior Tree Catalog

[config/bt_catalog.yaml](config/bt_catalog.yaml) defines the Behavior Tree nodes available to the model and the contracts for each Nav2 action.

Node entries may define:

```yaml
- id: "ComputePathToPose"
  category: "action"
  purpose: "Compute a path to one target pose."
  required_attributes: ["goal", "path"]
  optional_attributes: ["planner_id"]
  consumes: ["goal"]
  produces: ["path"]
```

Action contracts define the initial blackboard and required or forbidden nodes:

```yaml
actions:
  navigate_to_pose:
    initial_blackboard: ["goal"]
    required_nodes: ["ComputePathToPose", "FollowPath"]
    forbidden_nodes: ["ComputePathThroughPoses"]
```

The validator checks:

- XML shape: `<root>` with `main_tree_to_execute` and a `<BehaviorTree>` child
- every BT node exists in the catalog
- required node attributes are present
- child-count rules declared by the catalog
- action contracts declared by the catalog
- blackboard data flow from `consumes` and `produces`

## Run

Start an OpenAI-compatible local model server. Example with vLLM:

```bash
vllm serve google/gemma-4-E4B-it \
  --served-model-name gemma-4-e4b \
  --host 0.0.0.0 \
  --port 8080 \
  --api-key EMPTY \
  --generation-config vllm \
  --enable-auto-tool-choice \
  --tool-call-parser gemma4
```

Launch the node:

```bash
ros2 launch nav2_agent nav2_agent.launch.py
```

For local validation without Nav2 action servers:

```bash
ros2 launch nav2_agent nav2_agent.launch.py dry_run_nav2:=true log_level:=debug
```

Send a command:

```bash
ros2 topic pub /user_command std_msgs/msg/String "data: 'Move 2 meters forward'" --once
```

Other examples:

```bash
ros2 topic pub /user_command std_msgs/msg/String "data: 'Move 0.5 meters to the right'" --once
ros2 topic pub /user_command std_msgs/msg/String "data: 'Rotate 90 degrees to the left'" --once
ros2 topic pub /user_command std_msgs/msg/String "data: 'Go to x 1.5 y 2.0 in map'" --once
```

Monitor status:

```bash
ros2 topic echo /nav2_agent/status --field data --full-length
```

The trigger service executes the last command received on `/user_command`, or `default_trigger_command` if no command has been received:

```bash
ros2 service call /nav2_agent/trigger_command std_srvs/srv/Trigger {}
```

Generated XML and Mermaid files are written under `generated_bt_dir`.

## Tests

Run the Python unit tests with:

```bash
python3 -m unittest discover -s test -p 'test_*.py'
```

The current tests are pure Python tests for the Behavior Tree catalog and validator. They do not require ROS, Nav2, or a running LLM server.

## Docker

```bash
cd docker
./build.sh
docker compose run --rm nav2_agent
```

Inside the container:

```bash
cb
ros2 launch nav2_agent nav2_agent.launch.py
```

## License

Apache-2.0. See [LICENSE](LICENSE).
