# amr_sweeper_layer_0_fsm

ROS 2 package that implements a **robot-level finite state machine (FSM)** using:

```bash
ros2 launch amr_sweeper_layer_0_fsm amr_sweeper_layer_0_fsm.launch.py
```

- a non-lifecycle **supervisor** node (`supervisor_node`), and
- one **LifecycleNode** per FSM state:
  - `initializing_state_node`
  - `idling_state_node`
  - `running_state_node`
  - `charging_state_node`
  - `fault_state_node`

The supervisor accepts state-change requests (with priority metadata), drives ROS 2 lifecycle transitions for the state nodes, and publishes FSM status topics based on configurable publish rules.

---

## Repository layout (as shipped)

```
amr_sweeper_layer_0_fsm/
├── launch/
│   └── amr_sweeper_layer_0_fsm.launch.py
├── config/
│   ├── state_parameters.yaml
│   └── profiles/
│       ├── initializing_profiles.yaml
│       ├── idling_profiles.yaml
│       ├── running_profiles.yaml
│       ├── charging_profiles.yaml
│       └── fault_profiles.yaml
├── msg/
│   ├── FSMState.msg
│   └── FSMStatus.msg
├── srv/
│   └── RequestState.srv
├── src/
│   ├── _supervisor/
│   │   ├── supervisor_node.cpp
│   │   ├── state_node_base.cpp
│   │   └── process_manager.cpp
│   └── <state implementations>
└── tests/
    ├── fsm_tester_node.cpp
    └── amr_sweeper_layer_0_fsm.tests.py
```

---

## Launch

The primary launch file starts the supervisor and all lifecycle state nodes:

```bash
ros2 launch amr_sweeper_layer_0_fsm amr_sweeper_layer_0_fsm.launch.py
```

### Launch arguments

`launch/amr_sweeper_layer_0_fsm.launch.py` defines the following launch arguments:

- `namespace` (default: `amr_sweeper`)  
  Top-level namespace for all nodes.

- `use_sim_time` (default: `false`)  
  Passed to all nodes. When `true`, the FSM follows ROS time from `/clock`.

- `start_profile` (default: `001`)  
  Passed to the supervisor as `desired_profile` (integer). This selects the startup profile id.

- `tick_period_ms` (default: `100`)  
  Supervisor tick period in milliseconds.

- `state_params_file` (default: `<package_share>/config/state_parameters.yaml`)  
  ROS parameters file for supervisor + state nodes. The default is resolved with
  `ament_index_python.get_package_share_directory("amr_sweeper_layer_0_fsm")`.

Example (5 second tick, start profile 201, custom namespace):

```bash
ros2 launch amr_sweeper_layer_0_fsm amr_sweeper_layer_0_fsm.launch.py \
  namespace:=robot1 \
  start_profile:=201 \
  tick_period_ms:=5000
```

---

## Configuration model

### 1) `config/state_parameters.yaml`

This single ROS parameters file configures:

- **Supervisor publish rules** under `/**/supervisor.ros__parameters.publish.rules`.

  In the provided default config, the supervisor publishes:
  - `fsm_state` (`amr_sweeper_layer_0_fsm/msg/FSMState`)
  - `fsm_status` (`amr_sweeper_layer_0_fsm/msg/FSMStatus`)

  (These are *relative* names; with the default namespace they become:
  `/amr_sweeper/fsm_state` and `/amr_sweeper/fsm_status`.)

- **Per-state fault handling**, under each `/**/<state>_state.ros__parameters.faults`.

- **Per-state profile file path**, under each `/**/<state>_state.ros__parameters.profiles.file`.

  The per-state profile file paths in `state_parameters.yaml` are provided as **paths relative to the package share directory**
  (e.g. `config/profiles/running_profiles.yaml`). The launch file passes only `state_parameters.yaml`; the state nodes load
  their per-state profile files based on these references.

### 2) `config/profiles/*_profiles.yaml`

Each state has its own profile file with a list of profiles:

- `profiles[].profile.id` (uint16)
- optional `transitions` fields (e.g., auto transition / fault transition targets)
- `processes` list describing what to start/monitor in that profile, including:
  - `startup.exec` and `startup.args`
  - readiness checks (`startup.ready[]` supporting at least `topic` and `service` targets)
  - restart and shutdown policy
  - optional `rosout_triggers`

The default profile wiring is now:

- `001`, `101`, `301`, `400`: layer 1 hardware bringup only
- `201`: layer 1 hardware bringup, layer 2 controller bringup, optional joystick, and layer 3 localization/navigation bringup


---

## Interfaces

### Topics (configured via publish rules)

Message definitions live in `msg/`:

- `amr_sweeper_layer_0_fsm/msg/FSMState`
  - `stamp`
  - `current_state` (string like `"RUNNING"`)
  - `current_profile` (uint16)

- `amr_sweeper_layer_0_fsm/msg/FSMStatus`
  - `stamp`
  - `current_state`
  - `current_lifecycle_state`
  - `current_profile`
  - `transitioning_to_profile`
  - `transition_status`
  - `last_requester`, `last_request_priority`, `effective_priority_gate`, `priority_age_sec`
  - `last_message`

The default config publishes them on `fsm_state` and `fsm_status` once per second.

### Service: request a state/profile

Service definition: `srv/RequestState.srv`

The supervisor exposes a service named **`request_state`** (relative name), i.e. by default:

- `/amr_sweeper/request_state` (with `namespace:=amr_sweeper`)

Request fields include:
- `target_state`: one of `"INITIALIZING"`, `"IDLING"`, `"RUNNING"`, `"CHARGING"`, `"FAULT"`
- `target_lifecycle`: `""` / `"Active"` to activate, or `"Inactive"` to configure only
- `target_profile_id`: uint16 profile id
- metadata: `requester`, `priority`, `force`, `reason`

Example:

```bash
ros2 service call /amr_sweeper/request_state amr_sweeper_layer_0_fsm/srv/RequestState "{target_state: 'RUNNING', target_lifecycle: 'Active', target_profile_id: 200,requester: 'cli', priority: 200, force: false, reason: 'manual switch'}"
```

---

## Profile id “bands”

`RequestState.srv` documents the intended convention that each FSM state has a default “*00” profile id:
- `000` for INITIALIZING
- `100` for IDLING
- `200` for RUNNING
- `300` for CHARGING
- `400` for FAULT

This convention is used by the configuration and transition logic (e.g., auto-transition targets in the profile YAML).

---

## Quick introspection

With default namespace (`amr_sweeper`):

```bash
# Supervisor status streams (per config/state_parameters.yaml)
ros2 topic echo /amr_sweeper/fsm_state
ros2 topic echo /amr_sweeper/fsm_status

# See current desired profile parameter (supervisor)
ros2 param get /amr_sweeper/supervisor desired_profile

# Lifecycle state of a specific FSM state node
ros2 lifecycle get /amr_sweeper/initializing_state
```

---

## Build

Typical colcon build:

```bash
colcon build --packages-select amr_sweeper_layer_0_fsm
source install/setup.bash
```

---

## Notes

- The supervisor tick period is configurable via `tick_period_ms` in the launch file (default: 100 ms).
- Publish periods are configured via `publish.rules` in `config/state_parameters.yaml` and are decoupled from the supervisor tick.
