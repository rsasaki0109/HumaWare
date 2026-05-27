# ADR 0002: Core and Universe Split

## Status

Accepted.

## Decision

HumaWare separates stable core contracts from experimental integrations.

## Consequences

- Core packages must have smaller APIs and stronger tests.
- Experimental robot adapters, AI bridges, and simulator profiles can change faster.
- Promotion from experimental to stable requires validation, documentation, and migration notes.
