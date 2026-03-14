#!/usr/bin/env python3
"""MCP server that exposes the Ballistics-Lib RAG as a tool.

Claude Code / Cline use this via stdio transport:
  claude mcp add --scope project ballistics-rag -- python /path/to/rag/mcp_server.py

The server exposes one tool:
  search_codebase(query, n_results=8) -> retrieved chunks + Claude answer
"""
from __future__ import annotations

import asyncio
import os
import sys

# Resolve paths relative to this file so the server can be invoked from anywhere.
_RAG_DIR = os.path.dirname(os.path.abspath(__file__))
if _RAG_DIR not in sys.path:
    sys.path.insert(0, _RAG_DIR)

import mcp.types as types
from mcp.server import Server
from mcp.server.stdio import stdio_server

DB_PATH = os.environ.get("BALLISTICS_RAG_DB", os.path.join(_RAG_DIR, "rag_db"))

app = Server("ballistics-rag")


@app.list_tools()
async def list_tools() -> list[types.Tool]:
    return [
        types.Tool(
            name="search_codebase",
            description=(
                "Search the Ballistics-Lib C++ source code and return a grounded answer. "
                "Use this to ask questions about the API, physics model, fire-control "
                "solver, munitions, atmosphere model, or any implementation detail."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "Natural-language question about Ballistics-Lib",
                    },
                    "n_results": {
                        "type": "integer",
                        "description": "Number of source chunks to retrieve (default 8)",
                        "default": 8,
                    },
                },
                "required": ["query"],
            },
        )
    ]


@app.call_tool()
async def call_tool(name: str, arguments: dict) -> list[types.TextContent]:
    if name != "search_codebase":
        raise ValueError(f"Unknown tool: {name}")

    query: str = arguments["query"]
    n_results: int = int(arguments.get("n_results", 8))

    # Run the potentially-blocking RAG query in a thread to avoid blocking the loop.
    def _run() -> str:
        from query import query_rag

        return query_rag(query, db_path=DB_PATH, n_results=n_results, stream=False)

    answer = await asyncio.get_event_loop().run_in_executor(None, _run)
    return [types.TextContent(type="text", text=answer)]


async def main() -> None:
    async with stdio_server() as (read_stream, write_stream):
        await app.run(
            read_stream,
            write_stream,
            app.create_initialization_options(),
        )


if __name__ == "__main__":
    asyncio.run(main())
