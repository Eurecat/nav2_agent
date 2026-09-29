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
- Execution through standard `NavigateToPose` and `NavigateThroughPoses` goals. The model does not command motion directly.
- A Nav2 Behavior Tree generated for each command from a configurable node catalog, including recovery branches.
- Validation of every plan and Behavior Tree before execution. Invalid trees are returned to the model for correction.
- Outcome report after execution: goal status, recoveries and cause of failure.
- ROS 2 action interface with progress feedback, cancellation and preemption.
- Any OpenAI-compatible model server, including on-robot computers such as NVIDIA Jetson.

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

## Quick start

Requirements: Docker and an OpenAI-compatible model server with tool calling, such as [vLLM](https://docs.vllm.ai) or [llama.cpp](https://github.com/ggml-org/llama.cpp). See [Model server](docs/technical.md#model-server).

Build and enter the container:

```bash
cd docker && ./build.sh
docker compose run --rm nav2_agent
cb
```

Set `vllm_api_base` and `vllm_model_name` in [nav2_agent/config/agent_params.yaml](nav2_agent/config/agent_params.yaml), then launch the node:

```bash
ros2 launch nav2_agent nav2_agent.launch.py
```

Use `dry_run_nav2:=true` to run without Nav2. Send a command from another terminal:

```bash
ros2 run nav2_agent send "Move 2 meters forward"
```

`send` prints progress and the outcome report. Ctrl+C cancels the command.

## Documentation

The [technical guide](docs/technical.md) covers the packages, ROS interfaces, configuration, the Behavior Tree catalog and tests.

## License

Apache 2.0. See [LICENSE](LICENSE). Developed at [Eurecat](https://eurecat.org).
