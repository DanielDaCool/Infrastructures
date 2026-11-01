# Subagent learnings (Infrastructures)

## Observations
date | task type | model used | signal | better model
2026-10-31 | fact-only code inventory with file:line across 5 repos | haiku | accurate first try; reviewer confirmed 60+ cites | haiku (right)
2026-10-31 | design chapter: path geometry math | opus (high) | first try ok; one test value from a draft script contradicted the final rule (Major) | opus (right)
2026-10-31 | design chapter: speed profile / jerk law physics | opus (high) | 3 rounds of user-driven changes, each verified by script; no Critical | opus (right)
2026-10-31 | design chapter: follower control | opus (high) | tuning formula had a sign error (Critical at review) | opus (no higher tier; add sign checks to prompt)
2026-10-31 | API chapter + user guide from finished math | sonnet | found 26 cross-chapter conflicts; quick-start snippet broke on red alliance (Major) | sonnet (right; reviewer catches snippet bugs)
2026-10-31 | integrating 5 chapters + applying review fixes | opus (high) | one review fix introduced a Critical (braking rule) | opus (right; always re-review fixes)
2026-11-01 | doc simplification pass with math re-verification | opus (high) | correctly halted one item on script evidence; constant-speed-arc change had a Major found at review | opus (right)
2026-11-01 | design review / re-review (math, sim) | opus (high) | found Critical/Major issues each round with independent scripts | opus (right)

## Rules
- design-math editing and fix application (geometry/physics/control docs): opus; always re-review fixes, they introduced issues twice.

## Notes
- Parallel design chapters drift apart; define notation and decisions in one shared brief first, and plan an integration pass.
- Ask math workers to verify every formula with an independent script and to check signs end to end.
