# Skill: Task Management

How to read and update `.agent/tasks.json`.

## tasks.json structure

```json
{
  "phases": [
    {
      "id": "00",
      "name": "Scaffold project structure",
      "status": "completed",
      "files": ["opencode.json", ".gitignore", ".agent/*", "docs/*", "resources/*"],
      "verified_by": null
    }
  ]
}
```

## Status values

| Status | Meaning |
|---|---|
| `pending` | Not started |
| `in_progress` | Active — someone is working on it |
| `completed` | Done and QA-verified |
| `blocked` | Cannot proceed — depends on another phase or external input |

## Updating

- Only Orchestrator updates `tasks.json`.
- When starting a phase: set status to `in_progress`.
- When QA verifies: set status to `completed`, record `verified_by`.
- When blocked: set status to `blocked`, add `blocker_reason`.

## Fast resume

Before starting work, check `.agent/logs/` for the latest phase log. It contains the session state at phase completion so agents can resume without re-scanning all files.
