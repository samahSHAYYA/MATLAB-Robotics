# Review Checklist

## Physics correctness

- [ ] State vector is 13 elements (pos, quat, vel, omega)
- [ ] Quaternion remains unit norm after integration
- [ ] `move(FORWARD, 1)` produces forward acceleration in body frame
- [ ] Gravity acts in -z direction
- [ ] Zero input does not produce spontaneous acceleration
- [ ] Spring-damper GRF prevents foot penetration
- [ ] Energy is conserved (or near-conserved) with no dissipation

## Code quality

- [ ] Class inherits from `handle`
- [ ] All abstract methods are implemented
- [ ] No dead code, commented-out blocks, or debug prints
- [ ] `import robot.*` at top of consuming scripts
- [ ] `drawnow limitrate` used (not `drawnow`)
- [ ] `WindowKeyPressFcn` used (not `KeyPressFcn`)

## Edge cases

- [ ] Empty params struct handled gracefully
- [ ] `move(STOP)` zeroes all forces
- [ ] `reset()` restores initial conditions
- [ ] Rapid key presses do not crash
- [ ] Figure close exits cleanly (no zombie processes)

## ADR compliance

- [ ] All design decisions documented in `docs/adr/`
- [ ] ADR has context, decision, and consequences
- [ ] ADR references no out-of-date information
