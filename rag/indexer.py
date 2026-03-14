"""Index the Ballistics-Lib repository into a ChromaDB vector store."""
from __future__ import annotations

import os
from pathlib import Path

import chromadb

EXTENSIONS = {".cpp", ".hpp", ".h", ".md", ".json", ".cmake", ".txt", ".yml", ".yaml"}
SKIP_DIRS = {"build", "rag_db", "__pycache__", ".git", "node_modules"}
CHUNK_LINES = 60
OVERLAP_LINES = 10
COLLECTION = "ballistics"


def _chunk(path: Path, content: str) -> list[dict]:
    lines = content.splitlines()
    chunks: list[dict] = []
    i = 0
    while i < len(lines):
        end = min(i + CHUNK_LINES, len(lines))
        text = "\n".join(lines[i:end]).strip()
        if text:
            chunks.append(
                {
                    "text": text,
                    "file": str(path),
                    "start_line": i + 1,
                    "end_line": end,
                }
            )
        i += CHUNK_LINES - OVERLAP_LINES
    return chunks


def index_repo(repo_root: str = "..", db_path: str = "./rag_db") -> None:
    repo = Path(repo_root).resolve()
    client = chromadb.PersistentClient(path=db_path)

    try:
        client.delete_collection(COLLECTION)
        print("Cleared existing index.")
    except Exception:
        pass

    collection = client.create_collection(COLLECTION)

    all_chunks: list[dict] = []
    for fpath in sorted(repo.rglob("*")):
        if not fpath.is_file():
            continue
        if fpath.suffix not in EXTENSIONS:
            continue
        rel = fpath.relative_to(repo)
        if any(part in SKIP_DIRS for part in rel.parts):
            continue
        try:
            content = fpath.read_text(encoding="utf-8", errors="replace")
            all_chunks.extend(_chunk(rel, content))
        except Exception as e:
            print(f"  Skipping {rel}: {e}")

    print(f"Indexing {len(all_chunks)} chunks from {repo} ...")

    batch_size = 100
    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i : i + batch_size]
        collection.add(
            documents=[c["text"] for c in batch],
            metadatas=[
                {"file": c["file"], "start_line": c["start_line"], "end_line": c["end_line"]}
                for c in batch
            ],
            ids=[f"chunk_{i + j}" for j in range(len(batch))],
        )

    print(f"Done. {len(all_chunks)} chunks indexed into {db_path}.")
