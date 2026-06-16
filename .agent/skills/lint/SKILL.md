---
name: lint
description: Run MATLAB checkcode static analysis on all +robot/*.m source files. Use when asked to lint, code-review, or clean up warnings.
---

# Lint

Runs MATLAB's built-in `checkcode` on all source files under `+robot/`.

## Usage

```matlab
addpath('.agent/skills/lint/scripts');
lintCode();
```

The script prints each warning with file, line, column, and message.
Exits with an error if any file has warnings (fails CI).

## CI integration

Referenced in `.github/workflows/ci.yml` as a pre-test step:

```yaml
- name: Lint
  uses: matlab-actions/run-command@v2
  with:
    command: |
      addpath('.agent/skills/lint/scripts');
      lintCode();
```

## What it checks

- Unused arguments and variables
- Preallocation opportunities
- Parfor broadcast variables
- Code efficiency hints
