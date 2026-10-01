#!/usr/bin/env python3
"""Interactive DeepSeek terminal chat client."""

from __future__ import annotations

import os
import sys
from typing import Any

from openai import APIConnectionError, APIStatusError, OpenAI


SYSTEM_PROMPT = (
    "你是部署在无人船 Jetson 上的 ROS 2 助手。"
    "回答应准确、简洁；涉及控制设备时，必须给出可验证的结构化建议，"
    "不得假设命令已经执行。"
)


def require_api_key() -> str:
    api_key = os.environ.get("DEEPSEEK_API_KEY", "").strip()
    if not api_key:
        print("错误：未设置 DEEPSEEK_API_KEY。", file=sys.stderr)
        print(
            '请先执行：read -s -p "DeepSeek API Key: " '
            "DEEPSEEK_API_KEY; echo; export DEEPSEEK_API_KEY",
            file=sys.stderr,
        )
        raise SystemExit(2)
    return api_key


def main() -> int:
    client = OpenAI(
        api_key=require_api_key(),
        base_url=os.environ.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com"),
        timeout=60.0,
        max_retries=2,
    )
    model = os.environ.get("DEEPSEEK_MODEL", "deepseek-flash")
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": SYSTEM_PROMPT}
    ]

    print(f"DeepSeek 对话已启动（模型：{model}）")
    print("命令：/clear 清空上下文，/exit 退出\n")

    while True:
        try:
            question = input("你> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n对话已结束。")
            return 0

        if not question:
            continue
        if question.lower() in {"/exit", "exit", "quit", "退出"}:
            print("对话已结束。")
            return 0
        if question.lower() == "/clear":
            messages = [{"role": "system", "content": SYSTEM_PROMPT}]
            print("上下文已清空。\n")
            continue

        messages.append({"role": "user", "content": question})
        answer_parts: list[str] = []
        print("DeepSeek> ", end="", flush=True)

        try:
            stream = client.chat.completions.create(
                model=model,
                messages=messages,
                stream=True,
            )
            for chunk in stream:
                content = chunk.choices[0].delta.content
                if content:
                    print(content, end="", flush=True)
                    answer_parts.append(content)
            print("\n")
        except APIStatusError as exc:
            messages.pop()
            print(f"\nAPI 请求失败：HTTP {exc.status_code}\n", file=sys.stderr)
            continue
        except APIConnectionError as exc:
            messages.pop()
            print(f"\n网络连接失败：{exc}\n", file=sys.stderr)
            continue
        except Exception as exc:  # Keep the interactive shell alive.
            messages.pop()
            print(f"\n调用失败：{type(exc).__name__}: {exc}\n", file=sys.stderr)
            continue

        answer = "".join(answer_parts).strip()
        if answer:
            messages.append({"role": "assistant", "content": answer})


if __name__ == "__main__":
    raise SystemExit(main())
