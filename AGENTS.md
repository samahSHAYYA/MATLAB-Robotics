# AGENTS.md

MATLAB robotics demo project — 6-DOF rigid-body dynamics with wireframe 
visualization and keyboard control.

## Quick start

```
startRobot('Quadruped')            % flagship — robot with gait
startRobot('Quadcopter')           % aerial 6-DOF
startRobot('DifferentialDrive')    % planar wheeled
startRobot('Humanoid')             % bipedal walking
```

## Repository structure

| Path | Purpose |
|---|---|
| `+robot/` | MATLAB package — all class files |
| `.agent/agents/` | Agent prompts (orchestrator, architect, developer, QA) |
| `.agent/skills/` | Loadable domain knowledge + supporting MATLAB scripts |
| `.agent/guidelines/` | Best practices, workflow, review checklists |
| `.agent/tasks.json` | Build phase tracking |
| `.agent/logs/` | Session completion snapshots for fast resume |
| `docs/adr/` | Architecture Decision Records — read before any design work |
| `docs/guide/` | Architecture, interaction, API, demo guide, references |
| `resources/` | Images, schemas, 3D models for MATLAB visualization |
| `.githooks/` | Git hooks (pre-commit, commit-msg, pre-push, post-commit, post-checkout) — enable with `git config core.hooksPath .githooks` |
| `WarmStart.md` | Session memory (gitignored, auto-created by `post-checkout` hook or opencode on first interaction) |

## Build phases

See `.agent/tasks.json` for live status. Current phase: 13 (visual overhaul + fleet app).

## Key rules

1. **Agent-driven workflow** — before starting any task, consult the architect, developer, and QA agents. Gather feedback, agree on the approach, break into sprint tasks, then delegate and follow through to completion and verification.
2. **Read decisions first** — check `docs/adr/` and `.agent/logs/` before starting any phase.
2. **ADR required** — any non-trivial design choice needs an ADR in `docs/adr/`. Use `docs/adr/templates/adr-template.md` to scaffold.
3. **Skills load before work** — load the relevant skill for your domain before writing code.
4. **No force push, no amend** — each phase is one clean commit on `master`.
5. **QA signs off** — no phase is complete until QA verifies it.
6. **Auto-commit on verified completion** — once all agents (architect, developer, QA) sign off on a phase, commit and push automatically. Do not force-push or amend.
7. **Fix and re-iterate on failure** — if a task fails any verification, fix the issue and re-run until it passes, unless the user instructs otherwise.
8. **Context compact at 60%** — when opencode's context usage reaches 60%, stop active work and compact the session into `.agent/logs/` snapshots. Read compacted logs to resume. Do not continue unbounded context growth.
9. **Phase tracking** — every commit that changes `.m` source files must also update `.agent/tasks.json` with the current phase status.
9. **Git hooks** — run `git config core.hooksPath .githooks` once after clone to enable pre-commit, commit-msg, pre-push, post-commit, and post-checkout checks. The pre-commit hook verifies tasks.json is up to date when source files change. The post-commit hook appends commit info to `WarmStart.md` and reminds about doc/task updates. The post-checkout hook seeds `WarmStart.md` if missing.
10. **WarmStart.md** — read at session start for context recovery. Auto-created by `post-checkout` hook on first checkout (or by opencode on first interaction in a fresh clone). Auto-updated by opencode after completed tasks and by post-commit after each commit.

## MATLAB quirks

- Must inherit `handle` for mutable state.
- Use `WindowKeyPressFcn` (not `KeyPressFcn`) for keyboard capture.
- `hgtransform` + `Matrix` property for fast wireframe transforms.
- `drawnow` + `pause` throttling for animation loops (not `drawnow limitrate`).
- Quaternion operations use manual formulas in `robot.Utils` (no built-in `quaternion` class dependency for cross-version compatibility).
- Package `+robot/` means explicit `import robot.ClassName` at top of consuming scripts — no wildcard imports.

## Agents

| Agent | Load with | Purpose |
|---|---|---|
| Orchestrator | `opencode orchestrator` | Phase management, delegation, task tracking |
| Architect | `opencode architect` | Design approval, ADR creation, state model decisions |
| Developer | `opencode developer` | Implement MATLAB code per approved design |
| QA | `opencode qa` | Verify physics, run tests, report bugs |

## Skills

| Skill | `opencode` command | When to load |
|---|---|---|
| MATLAB OOP | `opencode skill matlab-oop` | Before writing any class |
| Robot Dynamics | `opencode skill robot-dynamics` | Before implementing dynamics/state model |
| Wireframe | `opencode skill wireframe` | Before working on visualization |
| Keyboard Control | `opencode skill keyboard-control` | Before working on controller/demo |
| Quadruped IK | `opencode skill quadruped-ik` | Before implementing leg IK |
| Humanoid IK | `opencode skill humanoid-ik` | Before implementing bipedal IK |
| Task Management | `opencode skill task-management` | Before updating tasks.json |
