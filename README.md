# VL53L1X Library for Raspberry Pi Pico

A translation of Pololu's [VL53L1X Arduino Library](https://github.com/pololu/vl53l1x-arduino) for Raspberry Pi Pico using the Pico SDK. Provides a basic I2C interface for ST's VL53L1X time-of-flight distance sensor.

## Installation

### Method 1: Copy to Project

1. Download the library directory.
2. Place it in your project's `lib` folder.
3. Add to your `CMakeLists.txt`:

```cmake
add_subdirectory(lib/vl53l1x-pico)
target_link_libraries(your_target VL53L1X)
```

### Method 2: Git Submodule

To add this library as a Git submodule, run:

```bash
git submodule add https://github.com/x33025/vl53l1x-pico.git lib/vl53l1x-pico
git submodule update --init --recursive
```

In your `CMakeLists.txt`:

```cmake
add_subdirectory(lib/vl53l1x-pico)
target_link_libraries(your_target VL53L1X)
```

#### Cloning with Submodules
If you are cloning a project that includes this as a submodule, use:

```bash
git clone --recurse-submodules https://github.com/yourproject/repository.git
```

To update submodules:

```bash
git submodule update --remote --merge
```

## CMake Configuration

The library requires these Pico SDK components:

```cmake
# In your project's CMakeLists.txt:
target_link_libraries(your_target
    pico_stdlib
    hardware_i2c
    VL53L1X
)
```

## Key Features

- Basic ranging measurements
- Timing budget configuration
- Distance mode selection (Short/Medium/Long)
- Timeout handling
- Continuous & single-shot modes

## Documentation

Most functionality matches the original Arduino library. For a detailed API reference, see [Pololu's Documentation](https://github.com/pololu/vl53l1x-arduino).

## Dependencies

- Raspberry Pi Pico SDK
- `hardware_i2c` (from Pico SDK)
