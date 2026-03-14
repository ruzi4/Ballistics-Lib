# Ballistics-Lib

A C++17 library for real-time ballistic trajectory simulation and fire-control
computation. Designed for integration into game engines, simulation tools, and
interactive applications.

## Features

- **Trajectory simulation** — RK4 (default) or symplectic-Euler integration with
  drag, gravity, and wind. A single step costs ~200 ns; thousands of concurrent
  projectiles are feasible at 60 Hz.
- **Fire-control solver** — `solve_elevation` finds the required launch angle for a
  given range using ternary-search + bisection over full trajectory simulations.
- **Pre-computed range tables** — `FireControlTable` sweeps elevation angles once
  (~50 – 300 ms, suitable for a background thread) and serves per-frame lookups in
  under 1 µs via binary search with linear interpolation.
- **ISA atmosphere model** — International Standard Atmosphere from sea level to
  20 km; moist-air density correction via the Magnus formula.
- **Munition library** — load projectile specifications from JSON; includes five
  real-world reference rounds out of the box.
- **Interactive renderer** — a standalone 3D visualisation application (raylib +
  raygui) that draws the launcher, target, and trajectory arc in real time.

## Coordinate convention

All APIs use a consistent **right-handed, z-up** world frame:

| Axis | Direction |
|---|---|
| +x | East |
| +y | North |
| +z | Up |

Angles follow standard conventions: azimuth 0° = North, 90° = East; elevation 0° =
horizontal, 90° = straight up.

## Requirements

- CMake 3.20+ (including CMake 4.x)
- A C++17-capable compiler (GCC 9+, Clang 10+, or MSVC 2019+)
- Internet access at configure time (nlohmann/json is fetched via `FetchContent`;
  raylib and raygui are fetched only when examples are enabled)

## Building

Both tests and examples are built by default. Pass `-DBALLISTICS_BUILD_TESTS=OFF`
or `-DBALLISTICS_BUILD_EXAMPLES=OFF` to skip either.

```bash
# Library + tests + examples (default)
cmake -B build
cmake --build build

# Library only
cmake -B build -DBALLISTICS_BUILD_TESTS=OFF -DBALLISTICS_BUILD_EXAMPLES=OFF
cmake --build build

# Run tests
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

### Windows (MSVC)

The build is fully supported on Windows with Visual Studio 2019 or later.
Use the `--config` flag to select the build configuration:

```bat
cmake -B build
cmake --build build --config Release
ctest --test-dir build --build-config Release --output-on-failure
```

Source files containing `µ` (U+00B5) are compiled with `/utf-8` automatically
so console output is correct regardless of the system ANSI code page.

## Quick start

```cpp
#include <ballistics/ballistics.hpp>
using namespace ballistics;

// Load a munition
MunitionLibrary lib;
lib.load("data/munitions.json");
const MunitionSpec& m855 = lib.get("5.56x45_m855_62gr");

// Set up simulator (sea-level ISA, no wind)
AtmosphericConditions atmo = isa_conditions(0.0);
TrajectorySimulator sim(m855, atmo);

// Solve for a 600 m shot due North
LauncherOrientation orient{ .azimuth_deg = 0.0 };
FireSolution sol = solve_elevation(sim, orient, 600.0, /*muzzle_speed=*/930.0);

if (sol.valid)
    printf("Elevation %.2f deg,  ToF %.0f ms\n",
           sol.elevation_deg, sol.flight_time_ms);
```

---

## Repository layout

| Path | Contents |
|---|---|
| [`include/`](include/README.md) | Public C++17 headers — the full API surface |
| [`src/`](src/README.md) | Library implementation files |
| [`tests/`](tests/README.md) | 24 unit and performance tests |
| [`data/`](data/README.md) | `munitions.json` — five reference projectile specs |
| [`examples/`](examples/README.md) | Runnable demos and the 3D renderer |

## Performance reference

| Operation | Typical cost |
|---|---|
| `TrajectorySimulator::step()` (RK4) | ~200 ns |
| `solve_elevation()` | 5 – 100 ms (120 – 180 simulations) |
| `FireControlTable::build()` | 50 – 300 ms (async-friendly) |
| `FireControlTable::lookup()` | < 1 µs |

---

## RAG — ask questions about this codebase

The `rag/` directory contains a Retrieval-Augmented Generation system that lets any
Claude interface answer questions about this library by searching the actual source
code. It uses [ChromaDB](https://www.trychroma.com/) for local vector storage
(all-MiniLM-L6-v2 embeddings) and `claude-opus-4-6` for answer generation.

### One-time setup

```bash
cd rag
pip install -r requirements.txt   # anthropic, chromadb, mcp
python main.py index              # builds rag/rag_db/ — run again after big changes
```

You also need an Anthropic API key for the answer-generation step:

```bash
export ANTHROPIC_API_KEY=sk-ant-...
```

---

### Terminal CLI

Ask questions directly from the shell. The answer streams token-by-token, followed
by a list of the source files that were retrieved.

```bash
cd rag
python main.py query "How does the RK4 integrator work?"
python main.py query "What munitions are included out of the box?"
python main.py query "How do I use FireControlTable for real-time lookup?"
python main.py query --results 12 "Explain the moving-target intercept algorithm"
```

Full options:

```
python main.py index [--repo ..] [--db ./rag_db]
python main.py query [--results N] [--db ./rag_db] "<question>"
```

---

### Claude Code CLI

`rag/mcp_server.py` is an MCP server that exposes the RAG as a `search_codebase`
tool. Register it once per project and Claude Code will call it automatically
whenever it needs to look something up.

**Register the server**

```bash
# Run from the repo root. Adjust the path if your repo lives elsewhere.
claude mcp add \
  --scope project \
  --env ANTHROPIC_API_KEY="$ANTHROPIC_API_KEY" \
  --env BALLISTICS_RAG_DB="$PWD/rag/rag_db" \
  ballistics-rag \
  -- python "$PWD/rag/mcp_server.py"
```

This writes an `.mcp.json` at the repo root (safe to commit so teammates pick it
up automatically). Restart Claude Code after adding the server.

**Verify it loaded**

```
/mcp
```

You should see `ballistics-rag` listed with the `search_codebase` tool.

**Use it in a session**

Just ask naturally — Claude Code decides when to call the tool:

```
How does the atmosphere model handle humidity?
What's the difference between solve_elevation and solve_moving_target?
Show me where FireControlTable::lookup does its binary search.
```

> **Windows note:** replace `python` with the full path to your Python executable
> (e.g. `C:\Python311\python.exe`).

---

### Cline (VS Code extension)

Cline reads MCP servers from a JSON settings file. Open it via
**Cline sidebar → MCP Servers → Configure MCP Servers**, or edit the file directly:

| OS | Path |
|---|---|
| macOS | `~/Library/Application Support/Code/User/globalStorage/saoudrizwan.claude-dev/settings/cline_mcp_settings.json` |
| Linux | `~/.config/Code/User/globalStorage/saoudrizwan.claude-dev/settings/cline_mcp_settings.json` |
| Windows | `%APPDATA%\Code\User\globalStorage\saoudrizwan.claude-dev\settings\cline_mcp_settings.json` |

Add this block (adjust paths to match your system):

```json
{
  "mcpServers": {
    "ballistics-rag": {
      "command": "python",
      "args": ["/absolute/path/to/Ballistics-Lib/rag/mcp_server.py"],
      "env": {
        "ANTHROPIC_API_KEY": "sk-ant-...",
        "BALLISTICS_RAG_DB": "/absolute/path/to/Ballistics-Lib/rag/rag_db"
      },
      "alwaysAllow": ["search_codebase"],
      "disabled": false
    }
  }
}
```

Save the file — Cline picks up the change immediately, no restart needed. The
`search_codebase` tool then appears in Cline's tool list and is called automatically
during conversations about the library.

---

### Claude web app (claude.ai)

The web app supports remote MCP servers on Pro, Max, Team, and Enterprise plans.
Because your RAG runs locally, you have two options:

**Option A — expose the MCP server over HTTP (full RAG)**

Deploy `mcp_server.py` to any public HTTPS endpoint
(Railway, Fly.io, a VPS, etc.) with the `rag_db/` folder included, then add it at
**Settings → Connectors → Add custom connector**:

```
Transport: HTTP (SSE)
URL:        https://your-domain.example.com/mcp
```

**Option B — upload the source as context (no server needed)**

For ad-hoc questions without a hosted server, paste or upload relevant source files
directly into a conversation. The web app's 200 K-token context window is large
enough to hold the entire library.

1. Concatenate the headers and source you care about:
   ```bash
   cat include/ballistics/*.hpp src/*.cpp > ballistics_full_source.txt
   ```
2. Upload `ballistics_full_source.txt` to a conversation (Pro/Max) or to a
   **Project** (Team/Enterprise) so it persists across sessions.

---

### Claude mobile app (iOS / Android)

The mobile app shares your claude.ai account settings, so any remote MCP server
you configured on the web (Option A above) is automatically available on mobile —
no extra setup required.

Local/stdio servers are not reachable from the mobile app. If you only need
occasional questions on mobile, the file-upload approach (Option B above) works
there too.
