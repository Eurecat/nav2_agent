# nav2_agent

ROS 2 package that executes natural-language navigation commands with Nav2. A language model plans each command and generates its Behavior Tree; the plan is validated before it is sent to Nav2.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-5C2A96)
![Nav2](https://img.shields.io/badge/Nav2-Jazzy-8750C8)
![Python](https://img.shields.io/badge/Python-3.12-3B1464)
![License](https://img.shields.io/badge/License-Apache%202.0-B795E3)

[![nav2_agent demo: a Unitree G1 following a natural-language command in RViz](docs/media/nav2_agent_demo.gif)](docs/media/nav2_agent_demo.mp4)

<sub>Unitree G1 in simulation.</sub>

## Features

- Relative motions, rotations, map coordinates and multi-step routes from natural-language commands.
- Named locations defined in a YAML file or saved by command.
- Follow-up commands that use the robot pose and the previous commands.
- Execution through standard `NavigateToPose` and `NavigateThroughPoses` goals. The model does not command motion directly.
- A Nav2 Behavior Tree generated for each command from a configurable node catalog, including recovery branches.
- Validation of every plan and Behavior Tree before execution. Invalid trees are returned to the model for correction.
- Outcome report after execution: goal status, recoveries and cause of failure.
- ROS 2 action interface with progress feedback, cancellation and preemption.
- Any OpenAI-compatible model server, including on-robot computers such as NVIDIA Jetson.

## TurtleBot quick demo

Runs a simulated TurtleBot 4 with Nav2 and nav2_agent. Requires Docker.

```bash
scripts/run_llm_server.sh          # keeps running, use a separate terminal
scripts/sim_demo.sh start
scripts/sim_demo.sh send "Move 2 meters forward"
```

`run_llm_server.sh` serves Gemma 4 with llama.cpp and needs an NVIDIA GPU. It targets Jetson Thor by default; on other GPUs, set `LLAMA_IMAGE` (see [Model server](docs/technical.md#model-server)). Any OpenAI-compatible server with tool calling also works. With the server on another machine, use `LLM_BASE_URL=http://<server>:8080/v1 scripts/sim_demo.sh start`. `scripts/sim_demo.sh stop` stops the demo.

## How it works

![How nav2_agent works](docs/diagrams/architecture.svg)

1. A command is received in natural language.
2. The planning agent selects the Nav2 action and calls tools that compute the target poses in ROS conventions.
3. The Behavior Tree is validated against the node catalog. Validation errors are sent back to the model.
4. Nav2 executes the tree. Recoveries run inside Nav2.
5. The agent reports the outcome using the Nav2 result and feedback.

### Example

![One command decomposed into poses](docs/diagrams/example.svg)

### Behavior Tree validation

![Self-repair loop](docs/diagrams/self_repair.svg)

## Example commands

| Command | Nav2 goal |
| --- | --- |
| *"Move 2 meters forward"* | One pose 2 m ahead |
| *"Move 0.5 meters to the right"* | One pose 0.5 m to the right |
| *"Rotate 90 degrees to the left"* | One pose rotated 90° to the left |
| *"Go to x 1.5 y 2.0 in map"* | One pose in the `map` frame |
| *"Move 3 meters forward, turn left 90° and advance 1 more meter"* | Three poses |
| *"Go to the shelves and then to the loading area"* | Two poses from named locations |
| *"Remember this place as the corner"* | Saves the current pose as a location |
| *"Go back"* | One pose at the position before the previous command |
| *"Which locations do you know?"* | Answer without moving |

## Usage

Requires ROS 2 Jazzy, Nav2 running on the robot and a model server reachable from it. See [Model server](docs/technical.md#model-server).

### Installation

Use the development container:

```bash
cd docker && ./build.sh && cd ..
docker compose -f docker/docker-compose.yml run --rm nav2_agent
cb
```

For a native installation, see [Build](docs/technical.md#build).

### Configuration

Set the model server and the robot frames in [nav2_agent/config/agent_params.yaml](nav2_agent/config/agent_params.yaml):

| Parameter | Description |
| --- | --- |
| `llm_base_url` | Model server endpoint |
| `llm_model` | Model name on the server |
| `global_frame` | Frame for explicit coordinates, usually `map` |
| `robot_base_frame` | Frame for relative motion, usually `base_link` |
| `locations_path` | Named locations file, also a launch argument. See [Locations](docs/technical.md#locations) |
| `generated_bt_dir` | Directory for generated Behavior Trees. Must be readable by Nav2's `bt_navigator` |

All parameters are listed in [Configuration](docs/technical.md#configuration).

### Running

```bash
ros2 launch nav2_agent nav2_agent.launch.py
```

Parameters come from `agent_params.yaml`; launch arguments override them when set. Use `use_sim_time:=true` in simulation and `dry_run_nav2:=true` to run without Nav2.

### Sending commands

From the command line:

```bash
ros2 run nav2_agent send "Move 2 meters forward"
```

`send --reset` discards the context of previous commands.

Through the `/nav2_agent/execute_command` action:

```bash
ros2 action send_goal --feedback /nav2_agent/execute_command nav2_agent_msgs/action/ExecuteCommand \
  "{command: 'Move 2 meters forward'}"
```

Through the `/user_command` topic:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'Move 2 meters forward'}"
```

## Documentation

The [technical guide](docs/technical.md) covers the packages, ROS interfaces, configuration, the Behavior Tree catalog and tests.

## License

Apache 2.0. See [LICENSE](LICENSE). Developed at [Eurecat](https://eurecat.org).
