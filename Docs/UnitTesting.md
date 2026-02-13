# Unit Testing (Host / WSL)

This project includes a host-side Unity test harness that does **not** change STM32 debug/release firmware build behavior.

## Requirements

- WSL installed with `make` and `gcc`
- Repo opened at workspace root

## Test Layout

- Test file naming: `tests/test_<module>.c`
- Firmware module under test: `Core/Src/<module>.c`
- Host build file: `tests/HostTests.make`
- Host stubs/shims:
  - `tests/include/`
  - `tests/support/`

## Commands

Run all discovered tests:

```bash
wsl make -f tests/HostTests.make test
```

Run one module:

```bash
wsl make -f tests/HostTests.make test-file FILE=balance_manager
```

List discovered test modules:

```bash
wsl make -f tests/HostTests.make list
```

Clean host test build artifacts:

```bash
wsl make -f tests/HostTests.make clean
```

## Adding a New Module Test

1. Create `tests/test_<module>.c`
2. Add Unity tests for public APIs in `Core/Src/<module>.c`
3. Extend `tests/support/*_stub.c` with any additional dependency stubs needed by that module
4. Run `test-file FILE=<module>` first, then `test`

## Notes

- Embedded targets remain unchanged (`Build STM`, `Build Clean STM`, `Flash STM`).
- Current host stubs are intentionally minimal and deterministic for unit testing logic paths.
- If WSL reports clock-skew warnings on Windows filesystems, rerun the command; test outcomes remain valid.
