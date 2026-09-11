from cap_robot.ollama_stream import consume_ollama_stream


class FakeResponse:
    def __init__(self, lines):
        self.lines = lines

    def iter_lines(self, decode_unicode=False):
        assert decode_unicode is True
        return iter(self.lines)


def test_chat_stream_separates_thinking_and_content():
    output = []
    response = FakeResponse([
        '{"message":{"thinking":"계산 "},"done":false}',
        '{"message":{"thinking":"중"},"done":false}',
        '{"message":{"content":"{\\"status\\":"},"done":false}',
        '{"message":{"content":"\\"ready\\"}"},"done":true}',
    ])

    content, thinking = consume_ollama_stream(
        response,
        endpoint='chat',
        label='Workstation',
        writer=output.append,
    )

    assert thinking == '계산 중'
    assert content == '{"status":"ready"}'
    rendered = ''.join(output)
    assert '🧠 [Workstation] 추론>' in rendered
    assert '🤖 [Workstation] 응답>' in rendered


def test_generate_stream_can_hide_thinking_but_still_collects_it():
    output = []
    response = FakeResponse([
        b'{"thinking":"secret","done":false}',
        b'{"response":"answer","done":false}',
        b'{"done":true}',
    ])

    content, thinking = consume_ollama_stream(
        response,
        endpoint='generate',
        label='agent1',
        show_thinking=False,
        writer=output.append,
    )

    assert content == 'answer'
    assert thinking == 'secret'
    assert 'secret' not in ''.join(output)
    assert 'answer' in ''.join(output)


def test_stream_error_is_reported():
    response = FakeResponse(['{"error":"model runner stopped"}'])

    try:
        consume_ollama_stream(
            response,
            endpoint='generate',
            label='agent1',
            writer=lambda _text: None,
        )
    except RuntimeError as error:
        assert 'model runner stopped' in str(error)
    else:
        raise AssertionError('RuntimeError가 발생해야 합니다.')
