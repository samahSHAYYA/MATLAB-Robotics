# ADR-0000: Use Agent System for Build Management

**Status:** accepted

**Date:** 2026-06-08

## Context

This project has multiple distinct work types (design, implementation, testing, project management). Managing these responsibilities within a single agent leads to context confusion and inconsistent quality. A structured agent system with role-specific prompts and skills was needed.

## Decision

Use OpenCode's agent system with four specialized agents:

- **Orchestrator** — phase management, delegation, task tracking
- **Architect** — design decisions, state models, ADR creation
- **Developer** — MATLAB code implementation
- **QA** — physics verification, test execution, bug reporting

Each agent has a dedicated prompt file in `.agent/agents/`, permitted tool lists, and defined authority boundaries.

## Consequences

- Clear separation of concerns: no agent steps outside its role.
- Slower per-phase execution (handoffs add overhead), but fewer errors.
- ADR creation is enforced by requiring Architect review before Developer writes code.
