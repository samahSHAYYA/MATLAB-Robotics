# Workflow

## Per-phase cycle

```
1. Orchestrator reads tasks.json → picks next pending phase
2. [If design needed] Orchestrator → Architect:
     Architect reads ADRs, creates new ADR, delivers design spec
3. Orchestrator → Developer:
     Developer loads skill, reads ADR, writes code
4. Orchestrator → QA:
     QA loads skill, runs tests, reports results
5. [If bugs] Orchestrator → Developer:
     Developer fixes bugs → goto 4
6. [If pass] Orchestrator:
     Updates tasks.json, writes log, commits, proceeds
```

## Handoff format

Format for delegating from Orchestrator to sub-agent:

```
Agent: {role}
Phase: {NN - name}
Task: {specific work to do}
Design: {reference to ADR or doc}
Files: {list of files to read/create/modify}
Acceptance: {criteria for pass}
```

## Phase completion log

After each phase, Orchestrator writes `.agent/logs/phase-NN.json`:

```json
{
  "phase": "03",
  "name": "DifferentialDrive + DynamicsEngine",
  "completed_at": "2026-06-08T16:00:00Z",
  "files_created": ["+robot/DifferentialDrive.m", "+robot/DynamicsEngine.m"],
  "files_modified": [],
  "decisions": ["RK4 with sub-stepping at 2 ms physics dt"],
  "verified_by": "QA",
  "blockers": [],
  "next_phase": "04"
}
```

## Emergency stops

- If a phase discovers a design flaw, escalate to Architect before proceeding.
- If a phase is blocked by missing prerequisites, mark it blocked in tasks.json and move to next unblocked phase.
