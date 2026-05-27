# humaware_capability_registry

Stability: experimental.

Publishes the robot capability registry and serves capability queries.

Default topic:

```text
/<robot_id>/capabilities
```

Default service:

```text
/<robot_id>/capabilities/list
```

The registry is a runtime contract for higher-level systems. AI policies,
fleet adapters, and operator tools should inspect capabilities instead of
publishing directly to hardware command topics.
