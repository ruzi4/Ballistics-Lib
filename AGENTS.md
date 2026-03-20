# Ballistics-Lib Agent Guidelines

This file provides instructions for agentic coding agents working on the Ballistics-Lib repository.

## Table of Contents
1. [Build Commands](#build-commands)
2. [Testing](#testing)
3. [Linting and Formatting](#linting-and-formatting)
4. [Code Style Guidelines](#code-style-guidelines)
5. [Dependencies](#dependencies)
6. [Repository Structure](#repository-structure)

## Build Commands

### Standard Build (Library + Tests + Examples)
```bash
cmake -B build
cmake --build build
```

### Library Only
```bash
cmake -B build -DBALLISTICS_BUILD_TESTS=OFF -DBALLISTICS_BUILD_EXAMPLES=OFF
cmake --build build
```

### Windows (MSVC) Specific
```bat
cmake -B build
cmake --build build --config Release
ctest --test-dir build --build-config Release --output-on-failure
```

### Clean Build
```bash
rm -rf build
cmake -B build
cmake --build build
```

## Testing

### Run All Tests
```bash
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

### Run a Single Test
```bash
ctest --test-dir build -R <test_name> --output-on-failure
```
Example: `ctest --test-dir build -R trajectory --output-on-failure`

### Run Tests in Debug Configuration (Windows)
```bat
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --build-config Debug --output-on-failure
```

### Enable Sanitizers (Linux/macOS)
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DSANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Linting and Formatting

### Clang-Format
The project uses clang-format for code formatting. Configuration is in `.clang-format`.

#### Format All Files
```bash
# Using the CMake target (preferred)
cmake --build build --target clang-format

# Or directly with clang-format
find . -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" | xargs clang-format -i
```

#### Check Formatting (CI-friendly)
```bash
# Using CMake target
cmake --build build --target clang-format-check

# Or directly
find . -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" | xargs clang-format -n
```

### Clang-Tidy
The project includes a `.clang-tidy` configuration.

#### Run Clang-Tidy
```bash
# Using CMake target (if available)
cmake --build build --target clang-tidy

# Or directly with run-clang-tidy.py (requires LLVM tools)
run-clang-tidy.py -p build
```

## Code Style Guidelines

### Language Standard
- C++17 is required
- Do not use C++20 features unless guarded by feature detection

### Formatting (from `.clang-format`)
- **Indentation**: 4 spaces, no tabs
- **Column Limit**: 100 characters
- **Brace Style**: Attach (K&R - opening brace on same line)
- **Pointer/Reference Alignment**: Left (`const Vec3& v`, not `Vec3 &v`)
- **Include Order**:
  1. Ballistics library headers (`^"ballistics/`)
  2. Project-relative headers (`^"`)
  3. C++ standard library (`^<[a-z_]+>`)
  4. Everything else (system headers, third-party)
- **Include Sorting**: Case sensitive
- **Spaces**:
  - Before parentheses: Only for control statements (`if`, `for`, `while`, `switch`)
  - After C-style cast: None
  - In parentheses: None
- **Braced List Style**: C++11 style (`{1, 2, 3}`)
- **Namespace Comments**: Fixed (closing namespace comments)
- **Namespace Indentation**: None
- **Access Modifier Offset**: -4
- **Reflow Comments**: true
- **Maximum Empty Lines**: 2

### Naming Conventions
Observed from the codebase:
- **Classes and Structs**: PascalCase (e.g., `TrajectorySimulator`, `FireSolution`)
- **Functions and Methods**: snake_case (e.g., `solve_elevation`, `step()`)
- **Variables**: snake_case (e.g., `muzzle_speed_ms`, `flight_time`)
- **Member Variables**: snake_case with trailing underscore (e.g., `munition_`, `atmosphere_`)
- **Constants**: kConstantName or ALL_CAPS_WITH_UNDERSCORES (context-dependent)
- **Template Parameters**: PascalCase (e.g., `typename T`)
- **Enumerations**: PascalCase for enum name, snake_case for values (or PascalCase for enum class values)
- **Macros**: ALL_CAPS_WITH_UNDERSCORES (rarely used)

### Types
- Use `using` declarations for type aliases when improving readability
- Prefer `std::uint32_t`, `std::int64_t`, etc. from `<cstdint>` for fixed-width integers
- Use `double` for floating-point unless precision/size constraints apply
- Use `bool` for boolean values
- Use `std::string` for text; `std::vector` for dynamic arrays
- Use `std::array` for fixed-size arrays when size is known at compile time

### Error Handling
- The library uses return values and output parameters for error indication (no exceptions)
- Functions returning `FireSolution` or `SolveResult` have a `.valid` boolean member
- Always check `.valid` before using results from solving functions
- Assertions are used for internal consistency checks (`assert()` from `<cassert>`)
- Invalid inputs should be handled gracefully where possible (e.g., clamping, returning invalid results)

### Comments
- Use Doxygen-style comments (`///`) for public API documentation
- Use `//` for implementation comments
- Comment the *why*, not the *what*
- Keep comments updated when changing code
- Use TODO comments for future work: `// TODO: explain`

### Includes
- Prefer `#include <ballistics/...>` for public headers when possible
- Use `#include "..."` for internal headers
- Sort includes according to the .clang-format rules
- Avoid unnecessary includes in headers (use forward declarations when possible)

### Other Guidelines
- Mark functions `const` when they don't modify object state
- Use `const&` for parameter passing unless small/trivial to copy
- Return small structs by value (NRVO/move semantics)
- Use `std::move` only when transferring ownership
- Prefer `emplace_back` over `push_back` when constructing elements in-place
- Use range-based for loops when appropriate
- Mark single-argument constructors `explicit` unless implicit conversion is intended
- Use `override` and `final` specifiers for virtual functions
- Prefer `nullptr` over `NULL` or `0`
- Use `auto` for complex types when it improves readability
- Avoid `using namespace` in headers

## Dependencies
- **nlohmann_json**: For JSON parsing (munitions data)
- **Threads**: For asynchronous solver functionality
- **raylib** and **raygui**: Only required for examples (optional)
- **Catch2**: Used for testing (fetched via FetchContent in tests/CMakeLists.txt)

## Repository Structure
```
Ballistics-Lib/
├── .clang-format        # clang-format configuration
├── .clang-tidy          # clang-tidy configuration
├── CMakeLists.txt       # Main build configuration
├── include/             # Public headers
│   └── ballistics/      # Library API
├── src/                 # Source implementation
├── tests/               # Unit and performance tests
├── data/                # JSON data files (munition specs)
├── examples/            # Example applications
├── rag/                 # Retrieval-Augmented Generation system
├── bitbucket-pipelines.yml  # CI configuration
└── Jenkinsfile          # Jenkins CI configuration
```

## Additional Notes
- The library is designed for real-time performance (~200ns per trajectory step)
- Avoid `-ffast-math` as it can change trajectory results; safe optimizations like `-fno-math-errno` and `-freciprocal-math` are used instead
- MSVC builds automatically use `/utf-8` to handle Unicode characters (like µ) correctly
- When adding new public API, update the corresponding header in `include/ballistics/` and document with Doxygen comments
- Tests should cover both normal operation and edge cases
- Performance tests are in the `tests/` directory and can be run alongside unit tests
