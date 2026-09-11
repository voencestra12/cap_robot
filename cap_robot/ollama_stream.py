"""Ollama NDJSON 스트림을 누적하면서 추론과 최종 응답을 표시합니다."""

from __future__ import annotations

import json
import sys
from typing import Callable, Tuple


def _stdout_writer(text: str) -> None:
    sys.stdout.write(text)
    sys.stdout.flush()


def consume_ollama_stream(
    response,
    *,
    endpoint: str,
    label: str,
    show_thinking: bool = True,
    show_content: bool = True,
    writer: Callable[[str], None] | None = None,
) -> Tuple[str, str]:
    """Ollama 스트림을 소비하고 ``(최종 응답, thinking)``을 반환합니다.

    ``/api/chat``은 토큰을 ``message.content``/``message.thinking``에,
    ``/api/generate``는 ``response``/``thinking``에 담습니다.
    """
    if endpoint not in ('chat', 'generate'):
        raise ValueError(f'지원하지 않는 Ollama endpoint입니다: {endpoint}')

    write = writer or _stdout_writer
    content_parts = []
    thinking_parts = []
    visible_section = None

    for raw_line in response.iter_lines(decode_unicode=True):
        if not raw_line:
            continue
        if isinstance(raw_line, bytes):
            raw_line = raw_line.decode('utf-8')
        try:
            chunk = json.loads(raw_line)
        except (TypeError, UnicodeDecodeError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f'Ollama 스트림 JSON 해석 실패: {raw_line!r}'
            ) from error

        if chunk.get('error'):
            raise RuntimeError(f"Ollama 스트림 오류: {chunk['error']}")

        if endpoint == 'chat':
            message = chunk.get('message') or {}
            thinking_piece = str(message.get('thinking') or '')
            content_piece = str(message.get('content') or '')
        else:
            thinking_piece = str(chunk.get('thinking') or '')
            content_piece = str(chunk.get('response') or '')

        if thinking_piece:
            thinking_parts.append(thinking_piece)
            if show_thinking:
                if visible_section != 'thinking':
                    write(f'\n🧠 [{label}] 추론>\n')
                    visible_section = 'thinking'
                write(thinking_piece)

        if content_piece:
            content_parts.append(content_piece)
            if show_content:
                if visible_section != 'content':
                    write(f'\n\n🤖 [{label}] 응답>\n')
                    visible_section = 'content'
                write(content_piece)

    if visible_section is not None:
        write('\n')

    return ''.join(content_parts).strip(), ''.join(thinking_parts).strip()
