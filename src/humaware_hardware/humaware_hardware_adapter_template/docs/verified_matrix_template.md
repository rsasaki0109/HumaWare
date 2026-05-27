# Verified robot matrix template

Real-robot support claims must be backed by evidence. Each row in the
verified matrix corresponds to one combination of robot, firmware, SDK,
and ROS distribution. Use the following template when adding a row to a
release announcement or adapter README.

```markdown
| Robot model | Firmware | Vendor SDK | ROS distro | Adapter version | Adapter git SHA | Launch profile | Bag / log | Operator | Date | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| <model>     | <fw>     | <sdk>      | <distro>   | <semver>        | <short sha>     | <launch file>  | <link>    | <name>   | <YYYY-MM-DD> | <known limits> |
```

Each entry must include:

- **Robot model** — vendor name and exact model identifier.
- **Firmware** — vendor firmware version reported by the robot.
- **Vendor SDK** — version of the SDK linked at runtime.
- **ROS distro** — ROS 2 distribution used for the bringup.
- **Adapter version** — semver of the HumaWare adapter package.
- **Adapter git SHA** — short SHA of the HumaWare commit used.
- **Launch profile** — exact launch file and arguments.
- **Bag / log** — link to the bringup log and runtime rosbag.
- **Operator** — name or alias of the engineer who ran the test.
- **Date** — date of the verification, ISO 8601.
- **Notes** — known limitations, MRM behavior, environment quirks.

An entry without bag or log is provisional and must be removed if
evidence is not provided before the next release.

A robot is considered "supported" only when:

- the row is filled in completely;
- the adapter passes the pre-deployment checklist
  ([`adapter_checklist.md`](adapter_checklist.md));
- MRM and E-stop have been exercised and recorded on real hardware.
