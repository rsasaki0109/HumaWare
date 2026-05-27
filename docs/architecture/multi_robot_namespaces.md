# Multi-robot Namespace Convention

HumaWare must work as well for one robot as for a fleet of them. Topics,
services, parameters, and bag profiles all use a single namespace
convention so the same runtime contract scales from a single robot
through small fleets without renaming code or rewiring launch profiles.

## Convention

Every runtime topic, service, and parameter is published or served under
a per-robot namespace rooted at the robot identifier. The recommended
form is:

```
/<robot_id>/<runtime_topic>
```

The `<robot_id>` segment is the value of the `robot_id` ROS parameter
declared by every HumaWare node. For example:

```
/mock_001/mode/state
/mock_001/safety/state
/mock_001/runtime/command_arbitration_state
/mock_001/cmd_vel/approved
```

Robot identifiers should be short, stable, and contain only the
characters `[a-z0-9_]`. Identifiers should not encode physical location,
fleet role, or vendor — those facts are recorded as metadata, not as
namespace fragments.

A single-robot deployment still uses a namespace. `robot_id=mock_001`
gives `/mock_001/...` even when there is only one robot on the ROS
graph. Code, dashboards, and bag profiles built against the convention
work unchanged when a second robot is added.

## Launch behavior

Launch files set the namespace through `Node(..., namespace=robot_id,
parameters=[{"robot_id": robot_id}])`. Both forms must agree:

- `namespace` controls where the node's topics and services live on the
  ROS graph;
- the `robot_id` parameter is published inside every runtime message so
  consumers can identify the source even after the message leaves the
  graph (for example, when replayed from a bag).

Adapter, perception, and operator nodes follow the same rule. A node
that cannot honor the namespace is not a HumaWare runtime node.

## Multi-robot deployments

To run two mock robots side by side, launch the bringup twice with
distinct robot identifiers:

```bash
ros2 launch humaware_launch mock_bringup.launch.py robot_id:=mock_001
ros2 launch humaware_launch mock_bringup.launch.py robot_id:=mock_002
```

Both runtimes share the same ROS domain but never publish onto each
other's topics. Operator stations, diagnostics dashboards, and fleet
tools can subscribe to either namespace, both, or use wildcards.

For larger fleets, prefer one bringup process per robot. Avoid sharing
runtime nodes across robots. Cross-robot coordination belongs in a
fleet adapter, not in the runtime layer.

## Cross-namespace links

Some runtime topics may need to be observed from outside the per-robot
namespace, for example by a fleet dashboard or by an Open-RMF adapter.
These links must not bypass the runtime contract:

- consumers subscribe to `/<robot_id>/<topic>` and respect the same
  message contract;
- consumers never publish onto another robot's runtime topics;
- consumers never publish onto a robot's actuator or candidate command
  topics;
- consumers should advertise themselves under their own namespace, for
  example `/fleet/<fleet_id>/...` or `/operator/<operator_id>/...`,
  rather than colonizing a robot's namespace.

## Topic and service mapping

The following topics live under each robot's namespace:

- `mode/state`
- `mode/transition_state`
- `safety/state`
- `safety/mrm_state`
- `locomotion/state`
- `runtime/command_arbitration_state`
- `runtime/health`
- `capabilities`
- `skills/state`
- `cmd_vel/approved`
- `cmd_vel/teleop`, `cmd_vel/autonomy`, `cmd_vel/policy` (candidate
  command topics)
- `hardware/heartbeat`, `teleop/heartbeat`

Services follow the same rule:

- `mode/set`
- `mode/takeover`
- `safety/trigger_mrm`
- `safety/clear_mrm`
- `capabilities/list`
- `skills/execute`

Global topics such as `/diagnostics` and `/tf` remain global by ROS 2
convention. Diagnostics messages must include the robot identifier in
`hardware_id` and in the status name so they remain attributable when
multiple robots publish to the shared topic.

## Replay considerations

Bags should record `/<robot_id>/...` topics under their original
namespace. To replay one robot's bag into a runtime running under a
different identifier, use `ros2 bag play --remap` to rewrite the
namespace prefix. Cross-namespace remapping must never reach hardware
adapters: use the `replay_only.launch.py` profile when playing back
runtime state for inspection.

## Anti-patterns

Avoid:

- using `/` (the global namespace) for any runtime topic;
- encoding fleet membership or location in the robot identifier;
- nesting fleet identifiers inside the robot namespace
  (`/fleet/site_a/robot_01/mode/state`);
- publishing one robot's approved commands onto another robot's
  command topics;
- creating shared runtime nodes that own state for multiple robots.

These break the assumption that a single robot deployment and a
multi-robot deployment look the same to runtime code, and they make
fleet-scale debugging materially harder.
