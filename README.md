# AQUARM SDK 1.1.0

## Requirements

- OS: Linux x86_64 (Ubuntu 22.04+ recommended)
- C++ build dependencies:

```bash
sudo apt install cmake g++ libeigen3-dev
```

- Python example runtime: CPython 3.10 with bundled modules in `python/`

## Build

Run from the SDK root:

```bash
cmake -S . -B build
cmake --build build -j
```

Generated targets:

- C++ examples: `ik_kcan`, `joint_kcan`
- Tests: `test_ik`, `test_trajectory`

## Run

### C++ examples

```bash
./build/joint_kcan
# or IK -> quintic trajectory -> KCAN command streaming
./build/ik_kcan
```

### C++ tests

```bash
ctest --test-dir build --output-on-failure
```

### Python

Basic IK smoke test:

```bash
python3 python/test_ik.py
```

KCAN examples:

```bash
python3 example/python/joint_kcan.py
python3 example/python/ik_kcan.py
```

Depending on KCAN device/network state, you may see `send_operational failed` (this is separate from build/link issues).

### MuJoCo Simulator

```bash
pip install mujoco
python3 simulator/MuJoCo/simulate_aquarm.py
```

Use the Viewer Control panel sliders to set each joint target angle (rad) and the gripper.

## Integrate into Your Project

Place the SDK folder in your project, then add this to `CMakeLists.txt`:

```cmake
add_subdirectory(aquarm_sdk-1.1.0 EXCLUDE_FROM_ALL)
target_link_libraries(my_app PRIVATE aquarm_sdk)
```

```cpp
#include "kalman_ik.hpp"
```

## Notes

- The 6D order for twist/Jacobian is `[linear; angular]`.
- URDF support is limited to `revolute`, `continuous`, and `fixed` joints in a single root-to-tip chain.
- Avoid `-mavx` and `-march=native`. They can change Eigen alignment assumptions relative to the prebuilt `libkalman_ik.so.<version>`.


