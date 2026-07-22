# nav2_agent

[![ROS 2 Distribution: Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Framework: PydanticAI](https://img.shields.io/badge/Agent-PydanticAI-emerald.svg)](https://ai.pydantic.dev/)

**`nav2_agent`** is a modular and deterministic architecture for connecting locally deployed Vision-Language Models (VLMs) and Large Language Models (LLMs) with the **Nav2** navigation stack in ROS 2.

Through strict **Tool-Use** and structured output validation with `PydanticAI`, the agent acts as a **semantic orchestrator**. It interprets natural-language instructions, resolves spatial references, selects a suitable Behavior Tree (BT), and delegates navigation execution to a Nav2 bridge layer backed by ROS 2 Action Clients.

---

## Motivation and Approach

Unlike end-to-end black-box robotics policies, **`nav2_agent`** follows a layered design:

1. **The AI reasons:** It interprets natural-language instructions, resolves spatial references and frames, and selects the appropriate navigation strategy.
2. **Nav2 executes:** Nav2 remains responsible for local, deterministic robot control, path planning, recovery, and safety policies.
3. **Inference stays local:** The agent is designed for local OpenAI-compatible inference servers, especially **vLLM**, avoiding cloud API latency and external runtime dependencies.

The goal is not to let an LLM drive the robot directly. The goal is to let the LLM choose and parameterize explicit, validated robotic actions.

---

## System Architecture

```text
 ┌────────────────────────────────────────────────────────┐
 │            Local vLLM Server (OpenAI API)              │
 └──────────────────────────┬─────────────────────────────┘
                            │ Structured JSON / Tool Calls
                            ▼
 ┌────────────────────────────────────────────────────────┐
 │             nav2_agent Node (PydanticAI)               │
 │                                                        │
 │  • Spatial Tools: Resolve TargetPose (x, y, theta)     │
 │  • BT Selector: Validate and select Behavior Trees     │
 │  • Navigation Tools: Send single or multi-pose goals   │
 │  • Recovery Tool: Request recovery behaviors           │
 └──────────────────────────┬─────────────────────────────┘
                            │ ROS 2 Action Clients
                            ▼
 ┌────────────────────────────────────────────────────────┐
 │                 NAV2 INTEGRATION LAYER                 │
 │      ActionClient goals for Nav2 Navigate actions      │
 │     (/navigate_to_pose | /navigate_through_poses)      │
 └────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```text
nav2_agent/
├── config/
│   ├── agent_params.yaml       # vLLM model, API base, prompt, and runtime parameters
│   └── bt_catalog.yaml         # Simulated catalog of available Nav2 Behavior Trees
├── docker/
│   ├── Dockerfile              # ROS 2 Jazzy development image with PydanticAI support
│   ├── docker-compose.yml      # Host-networked development container setup
│   ├── build.sh                # Convenience Docker build helper
│   └── entrypoint.sh           # ROS environment entrypoint
├── launch/
│   └── nav2_agent.launch.py    # Main launch file
├── nav2_agent/
│   ├── agent_node.py           # Main ROS 2 node using rclpy and MultiThreadedExecutor
│   ├── pydantic_agent.py       # PydanticAI agent, system prompt, and tool definitions
│   ├── models.py               # Pydantic schemas: TargetPose, BTSelection, AgentResponse
│   └── nav2_bridge.py          # ROS 2 ActionClient bridge to Nav2 navigation actions
├── package.xml
├── setup.cfg
└── setup.py
```

---

## Installation and Requirements

### Requirements

- **ROS 2:** Jazzy Jalisco.
- **Python:** $\ge 3.10$
- **LLM/VLM backend:** A local [vLLM](https://github.com/vllm-project/vllm) server exposing the OpenAI-compatible API.
- **Python packages:** `pydantic`, `pydantic-ai`, `openai`, and `PyYAML`.

### Clone and Build in a ROS 2 Workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/eurecat-robotics/nav2_agent.git

# Install Python dependencies in your active environment.
python3 -m pip install pydantic pydantic-ai openai PyYAML

cd ~/ros2_ws
colcon build --packages-select nav2_agent
source install/setup.bash
```

### Docker Development Environment

The repository includes a ROS 2 Jazzy Docker setup with the required Python agent dependencies installed in an isolated virtual environment.

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

To rebuild without cache:

```bash
./build.sh --no-cache
```

---

## Configuration

Edit [config/agent_params.yaml](config/agent_params.yaml) to point the agent to your local vLLM server:

```yaml
nav2_agent_node:
  ros__parameters:
    vllm_api_base: "http://localhost:8000/v1"
    vllm_model_name: "Qwen/Qwen2.5-7B-Instruct"
    bt_catalog_path: ""
    default_trigger_command: "Navigate to the staging area using the default behavior tree."
    navigate_to_pose_action: "/navigate_to_pose"
    navigate_through_poses_action: "/navigate_through_poses"
    action_server_timeout_sec: 5.0
    system_prompt: |
      You are a robotic navigation command orchestrator for a ROS 2 robot using Nav2.
      Use the available tools to resolve coordinates, select a Behavior Tree,
      and send navigation goals.
```

Edit [config/bt_catalog.yaml](config/bt_catalog.yaml) to define the Behavior Trees the agent is allowed to select. The current catalog is simulated and includes examples such as `default_nav.xml`, `cautious_slow.xml`, `fast_aggressor.xml`, and `inspection_waypoints.xml`.

---

## Usage

### 1. Start a Local vLLM Server

```bash
vllm serve Qwen/Qwen2.5-7B-Instruct --port 8000
```

Check that the OpenAI-compatible endpoint is reachable:

```bash
curl http://localhost:8000/v1/models
```

### 2. Launch the `nav2_agent` Node

```bash
ros2 launch nav2_agent nav2_agent.launch.py
```

### 3. Send a Natural-Language Instruction

Publish a command through `/voice_command`:

```bash
ros2 topic pub /voice_command std_msgs/msg/String "data: 'Navigate to the pallet in loading area B using cautious navigation'" --once
```

Monitor agent status:

```bash
ros2 topic echo /nav2_agent/status
```

You can also trigger execution through the service. The `std_srvs/srv/Trigger` service does not carry arbitrary command text, so the node executes the last command received on `/voice_command`, or `default_trigger_command` if no command has been received yet.

```bash
ros2 service call /nav2_agent/trigger_command std_srvs/srv/Trigger {}
```

---

## Current Implementation Status

This repository currently targets ROS 2 Jazzy and provides the agent architecture plus a ROS 2 ActionClient-based Nav2 navigation boundary:

- The agent uses `pydantic-ai` with an OpenAI-compatible vLLM endpoint.
- Tools are registered for coordinate extraction, Behavior Tree selection, navigation, multi-pose navigation, and recovery.
- `Nav2Bridge` sends real `NavigateToPose` and `NavigateThroughPoses` goals through ROS 2 Action Clients.
- `Nav2Bridge` converts validated `TargetPose` objects into `geometry_msgs/msg/PoseStamped` goals.
- Behavior Tree selections are passed to Nav2 through the `behavior_tree` goal field.
- Spatial lookup and recovery are still mocked while the perception and recovery integration contracts are refined.
- No direct dependency on `nav2_simple_commander` is used yet.
- Future work can replace the mocked spatial lookup and recovery internals while keeping the agent contract stable.

---

## Presented at ROSCon Spain 2026

This package was presented as a technical talk at **ROSCon Spain 2026**:

- **Title:** *Integrating Nav2 with VLM Agents: From Natural Language to NavigateToPose*
- **Author:** Pau Reverte
- **Organization:** Eurecat Technology Centre, Robotics and Automation Unit

---

## License

This project is distributed under the **Apache 2.0** license. See the `LICENSE` file for details.

---

## Contact and Contributions

Developed and maintained by the **Mobile Robotics Unit at Eurecat**.

- **Author:** Pau Reverte
- **Email:** [pau.reverte@eurecat.org](mailto:pau.reverte@eurecat.org)
- **Organization:** [Eurecat Technology Centre](https://eurecat.org)

Contributions, issues, and pull requests are welcome.
