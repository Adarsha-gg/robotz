# Livox-SDK2 Build Guide

## How to Build

```bash
cd /home/pass/robot_cpp/Livox-SDK2
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## Problem / Troubleshooting

If build fails with errors, here's what we fixed:

### Error 1: C++20 Compatibility Error
```
error: identifier 'char8_t' is a keyword in C++20 [-Werror=c++20-compat]
```

**Fix:**
Edit `/home/pass/robot_cpp/Livox-SDK2/CMakeLists.txt` and add:
```cmake
add_compile_options(-Wno-error=c++20-compat -Wno-c++20-compat)
```

### Error 2: Missing uint8_t/uint16_t types
```
error: 'uint8_t' in namespace 'std' does not name a type
error: 'uint16_t' in namespace 'std' does not name a type
```

**Fix:**
Edit `/home/pass/robot_cpp/Livox-SDK2/sdk_core/comm/define.h` and add at the top:
```cpp
#include <cstdint>
```

### Error 3: Missing uint64_t type
```
error: 'uint64_t' does not name a type
```

**Fix:**
Edit `/home/pass/robot_cpp/Livox-SDK2/sdk_core/logger_handler/file_manager.h` and add:
```cpp
#include <cstdint>
```

### After Fixes, Rebuild:
```bash
cd /home/pass/robot_cpp/Livox-SDK2/build
sudo rm -rf *
cmake ..
make
```

Done.