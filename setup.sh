#!/bin/bash

# Unitree SDK2 Setup Script
# This script will install dependencies and build the Unitree SDK2

set -e  # Exit on error

echo "================================================"
echo "Unitree SDK2 Setup Script"
echo "================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if running as root for dependency installation
if [ "$EUID" -ne 0 ]; then 
    print_warning "This script needs sudo privileges to install dependencies"
    print_warning "You may be prompted for your password"
fi

# Step 1: Update package list
print_status "Updating package list..."
sudo apt-get update

# Step 2: Install dependencies
print_status "Installing required dependencies..."
sudo apt-get install -y \
    cmake \
    g++ \
    build-essential \
    libyaml-cpp-dev \
    libeigen3-dev \
    libboost-all-dev \
    libspdlog-dev \
    libfmt-dev

if [ $? -eq 0 ]; then
    print_status "Dependencies installed successfully!"
else
    print_error "Failed to install dependencies"
    exit 1
fi

# Step 3: Navigate to unitree_sdk2 directory
print_status "Navigating to unitree_sdk2 directory..."
cd unitree_sdk2

# Step 4: Create build directory
print_status "Creating build directory..."
if [ -d "build" ]; then
    print_warning "Build directory already exists. Cleaning..."
    rm -rf build
fi
mkdir build
cd build

# Step 5: Run CMake
print_status "Running CMake configuration..."
cmake ..

if [ $? -eq 0 ]; then
    print_status "CMake configuration successful!"
else
    print_error "CMake configuration failed"
    exit 1
fi

# Step 6: Build the project
print_status "Building the project (this may take a few minutes)..."
make -j$(nproc)

if [ $? -eq 0 ]; then
    print_status "Build completed successfully!"
else
    print_error "Build failed"
    exit 1
fi

# Step 7: Optional - Install to system
echo ""
print_status "Build completed successfully!"
echo ""
echo "================================================"
echo "Next Steps:"
echo "================================================"
echo ""
echo "1. To install SDK to system directory:"
echo "   sudo make install"
echo ""
echo "2. To install to custom directory (e.g., /opt/unitree_robotics):"
echo "   sudo cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics"
echo "   sudo make install"
echo ""
echo "3. To run examples:"
echo "   Navigate to build/example/ directory"
echo ""
echo "4. Build directory location:"
echo "   $(pwd)"
echo ""
echo "================================================"