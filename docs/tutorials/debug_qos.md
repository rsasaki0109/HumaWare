# Debug QoS

Humanoid robots are sensitive to DDS configuration, wireless loss, and topic QoS.

## Checklist

- confirm ROS domain ID
- confirm robot namespace
- inspect topic publishers and subscribers
- inspect QoS compatibility
- check packet loss
- check CPU and memory pressure
- compare wired and wireless behavior
- record network conditions in experiment cards

Future tooling should provide repeatable network diagnostics and recommended QoS profiles.
