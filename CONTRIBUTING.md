# Contributing

Thanks for your interest in improving AQUARM SDK.

## Development setup

1. Build with CMake:

   ```bash
   cmake -S . -B build
   cmake --build build -j
   ```

2. Run tests:

   ```bash
   ctest --test-dir build --output-on-failure
   ```

## Pull request guidelines

- Keep changes focused and minimal.
- Update docs when behavior or interfaces change.
- Ensure C++ tests pass before requesting review.
- For Python example changes, run the corresponding script(s).

## License for contributions

By submitting a contribution, you agree that your contribution is licensed
under the Apache License 2.0 used by this repository.
