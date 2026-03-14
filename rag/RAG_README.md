# RAG — ask questions about this codebase

This directory contains a Retrieval-Augmented Generation system that lets any
Claude interface answer questions about Ballistics-Lib by searching the actual
source code. It uses [ChromaDB](https://www.trychroma.com/) for local vector
storage (all-MiniLM-L6-v2 embeddings) and `claude-opus-4-6` for answer
generation.

## Files

| File | Purpose |
|---|---|
| `main.py` | CLI entry point — `index` and `query` sub-commands |
| `indexer.py` | Chunks the repo into 60-line overlapping segments and loads them into ChromaDB |
| `query.py` | Retrieves the top-N relevant chunks and streams an answer via Claude |
| `mcp_server.py` | MCP server exposing a `search_codebase` tool for Claude Code / Cline |
| `requirements.txt` | Python dependencies: `anthropic`, `chromadb`, `mcp` |
| `rag_db/` | Generated vector database (git-ignored; created by `python main.py index`) |

---

## One-time setup

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

## Terminal CLI

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
python main.py query [--results N] [--db ./rag_db] [--retrieve-only] "<question>"
```

Use `--retrieve-only` to print the retrieved source chunks without calling Claude
(no `ANTHROPIC_API_KEY` required — useful for verifying the index).

---

## Claude Code CLI

`mcp_server.py` is an MCP server that exposes the RAG as a `search_codebase`
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

## Cline (VS Code extension)

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

## Claude web app (claude.ai)

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

## Claude mobile app (iOS / Android)

The mobile app shares your claude.ai account settings, so any remote MCP server
you configured on the web (Option A above) is automatically available on mobile —
no extra setup required.

Local/stdio servers are not reachable from the mobile app. If you only need
occasional questions on mobile, the file-upload approach (Option B above) works
there too.
