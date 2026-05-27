# Security Policy

HumaWare may eventually operate near real robots and operator workstations. Security issues can become safety issues.

## Reporting

Report vulnerabilities through GitHub private vulnerability reporting when available. If that is not available, contact the maintainers privately before publishing details.

## Sensitive Areas

Please report issues related to:

- remote command execution
- unsafe command bypass
- E-stop or watchdog bypass
- credential leakage
- unprotected remote operator channels
- insecure default network configuration
- bag, log, or dataset leaks containing private environment data

## Safety Boundary

Security fixes must not weaken safety behavior.
