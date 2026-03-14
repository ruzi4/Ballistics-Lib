"""Query the Ballistics-Lib RAG: retrieve relevant chunks and answer with Claude."""
from __future__ import annotations

import anthropic
import chromadb

COLLECTION = "ballistics"
SYSTEM_PROMPT = """\
You are an expert on Ballistics-Lib, a high-performance C++17 library for real-time \
ballistic trajectory simulation and fire-control computation. \
Answer questions using the provided source code context. \
When referencing code, cite the file and line range. \
If the answer is not in the context, say so clearly."""


def query_rag(
    question: str,
    db_path: str = "./rag_db",
    n_results: int = 8,
    stream: bool = True,
    retrieve_only: bool = False,
) -> str:
    db = chromadb.PersistentClient(path=db_path)
    collection = db.get_collection(COLLECTION)

    results = collection.query(query_texts=[question], n_results=n_results)
    docs: list[str] = results["documents"][0]
    metas: list[dict] = results["metadatas"][0]

    context_parts: list[str] = []
    for doc, meta in zip(docs, metas):
        context_parts.append(
            f"### {meta['file']} (lines {meta['start_line']}–{meta['end_line']})\n"
            f"```\n{doc}\n```"
        )
    context = "\n\n".join(context_parts)

    if retrieve_only:
        print(context)
        return context

    user_message = f"<context>\n{context}\n</context>\n\nQuestion: {question}"

    if not __import__("os").environ.get("ANTHROPIC_API_KEY"):
        raise RuntimeError(
            "ANTHROPIC_API_KEY environment variable is not set. "
            "Export it before running: export ANTHROPIC_API_KEY=sk-ant-..."
        )

    client = anthropic.Anthropic()

    if stream:
        answer_parts: list[str] = []
        with client.messages.stream(
            model="claude-opus-4-6",
            max_tokens=2048,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": user_message}],
        ) as s:
            for text in s.text_stream:
                print(text, end="", flush=True)
                answer_parts.append(text)
        print()
        return "".join(answer_parts)
    else:
        response = client.messages.create(
            model="claude-opus-4-6",
            max_tokens=2048,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": user_message}],
        )
        answer = next(b.text for b in response.content if b.type == "text")
        return answer
