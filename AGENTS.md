# Test case guidance

Use a **less is more** philosophy: keep the smallest set of tests that provides meaningful confidence.

- Test observable behavior, important contracts, safety boundaries, and known regressions.
- Add a regression test when fixing a meaningful bug; make it fail for the original failure.
- Prefer a few representative cases and relevant boundaries over exhaustive combinations.
- Reuse existing fixtures and parameterize closely related cases instead of duplicating tests.
- Avoid tests that mirror implementation steps, assert incidental details, or merely repeat constants from production code.
- Skip new tests for documentation, formatting, and reversible low-impact changes unless they affect behavior.
- Use simulation or integration checks when command ownership, cancellation, units, or IO interactions are the actual concern; avoid elaborate mocks that bypass that concern.
- Keep assertions deterministic and tied to justified tolerances. Distinguish numerical simulation checks from hardware evidence.
- Run the checks relevant to the change. Broaden testing for shared behavior or unresolved risk; do not rerun unchanged checks without a reason.

Every test should protect a distinct behavior or failure mode. If its removal would not reduce confidence, consolidate or remove it. Test count and coverage percentage are not goals by themselves.
