#!/bin/bash
set -e

SDK_DIR=/home/pass/robot_cpp/lidar/ws_livox/src/Livox-SDK2

echo "🔧 Starting Livox-SDK2 Fix & Build Script..."

# --- Ensure directory exists ---
if [ ! -d "$SDK_DIR" ]; then
  echo "❌ SDK directory not found at $SDK_DIR"
  echo "➡️  Clone it first: git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/robot_cpp/Livox-SDK2"
  exit 1
fi

cd "$SDK_DIR"

# --- Patch CMakeLists.txt ---
echo "🛠  Patching CMakeLists.txt for C++20 compatibility..."
if ! grep -q "add_compile_options(-Wno-error=c++20-compat" CMakeLists.txt; then
  sed -i '/cmake_minimum_required/a add_compile_options(-Wno-error=c++20-compat -Wno-c++20-compat)' CMakeLists.txt
fi

# --- Patch header files ---
echo "🧩 Ensuring <cstdint> is included in necessary headers..."
DEFINE_H="$SDK_DIR/sdk_core/comm/define.h"
FILE_MANAGER_H="$SDK_DIR/sdk_core/logger_handler/file_manager.h"

for FILE in "$DEFINE_H" "$FILE_MANAGER_H"; do
  if ! grep -q "#include <cstdint>" "$FILE"; then
    sed -i '1i #include <cstdint>' "$FILE"
  fi
done

# --- Clean build directory ---
echo "🧹 Cleaning old build..."
mkdir -p build && cd build
sudo rm -rf *

# --- Build and install ---
echo "⚙️  Building SDK..."
cmake ..
make -j$(nproc)
sudo make install

# --- Verify installation ---
echo "📦 Installed libraries:"
ls /usr/local/lib | grep livox || echo "⚠️  Livox library not found — something went wrong."

echo "✅ Livox-SDK2 build complete!"
