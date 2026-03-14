#!/usr/bin/env python3
"""Ballistics-Lib RAG CLI.

Usage:
  python main.py index                    # build the index (run once)
  python main.py query "How does RK4 work?"
  python main.py query --results 12 "What munitions are supported?"
"""
from __future__ import annotations

import argparse
import sys


def cmd_index(args: argparse.Namespace) -> None:
    from indexer import index_repo

    index_repo(repo_root=args.repo, db_path=args.db)


def cmd_query(args: argparse.Namespace) -> None:
    from query import query_rag

    question = " ".join(args.question)
    if not question.strip():
        print("Error: provide a non-empty question.", file=sys.stderr)
        sys.exit(1)

    print(f"\nQuery: {question}\n{'─' * 60}\n")
    answer = query_rag(question, db_path=args.db, n_results=args.results, retrieve_only=args.retrieve_only)

    if not args.retrieve_only:
        # Sources footer
        # (query_rag already printed the streamed answer; just list sources)
        from chromadb import PersistentClient

        db = PersistentClient(path=args.db)
        collection = db.get_collection("ballistics")
        results = collection.query(query_texts=[question], n_results=args.results)
        metas = results["metadatas"][0]

        print(f"\n{'─' * 60}\nSources:")
        seen: set[str] = set()
        for m in metas:
            label = f"  {m['file']}:{m['start_line']}–{m['end_line']}"
            if label not in seen:
                print(label)
                seen.add(label)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Ballistics-Lib RAG — index and query the codebase with Claude"
    )
    parser.add_argument("--db", default="./rag_db", help="ChromaDB directory (default: ./rag_db)")

    sub = parser.add_subparsers(dest="command", required=True)

    p_index = sub.add_parser("index", help="Build or rebuild the vector index")
    p_index.add_argument(
        "--repo",
        default="..",
        help="Path to the repository root (default: ..)",
    )

    p_query = sub.add_parser("query", help="Ask a question about the codebase")
    p_query.add_argument("question", nargs="+", help="Your question")
    p_query.add_argument(
        "--results",
        type=int,
        default=8,
        help="Number of chunks to retrieve (default: 8)",
    )
    p_query.add_argument(
        "--retrieve-only",
        action="store_true",
        default=False,
        help="Print retrieved source chunks without calling Claude (no API key required)",
    )

    args = parser.parse_args()

    if args.command == "index":
        cmd_index(args)
    elif args.command == "query":
        cmd_query(args)


if __name__ == "__main__":
    main()
