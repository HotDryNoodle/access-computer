# access-computer Guidelines

## Structure

- Public API and implementations: `src/` (`planner/`, `geometry/`, `gmat/`)
- Plugin entry: `src/access-computer/main.cpp`
- Manifest: `configs/plugins/`

## Build

From `satellite-workspace`: `./scripts/build-all.sh` or Meson in this directory after SDK wrap is configured.

Contract tests: `scripts/contract-test-access-computer.sh`

## Coding Style

C++17, `warning_level=2`. Format with repo-root `.clang-format`.

When working inside `satellite-workspace`, see `skills/coding-style-rules/SKILL.md` at the workspace root. Standalone clone: follow the same rules locally.

- **Naming**: PascalCase types; `snake_case` functions and files; binary name `access-computer`.
- **API docs**: Doxygen `/** ... */` on planner/geometry public APIs.
- **No silent fallback**; GMAT vs dry-run paths and coordinate/time assumptions must be explicit.

Details: workspace `skills/coding-style-rules/references/cpp-style-details.md`.
