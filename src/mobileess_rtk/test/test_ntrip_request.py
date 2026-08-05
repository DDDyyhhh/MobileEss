import base64
import unittest

from mobileess_rtk.ntrip import NtripClient, build_request


class FakeSocket:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []
        self.closed = False
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, size):
        if not self.responses:
            return b''
        return self.responses.pop(0)

    def shutdown(self, how):
        pass

    def close(self):
        self.closed = True


class TestNtripRequest(unittest.TestCase):
    def test_request_contains_mountpoint_and_auth(self):
        request = build_request('caster.example', 2101, '/MOUNT', 'user', 'secret')
        text = request.decode('ascii')
        self.assertIn('GET /MOUNT HTTP/1.1', text)
        self.assertIn('Host: caster.example:2101', text)
        self.assertIn('Ntrip-Version: Ntrip/2.0', text)
        encoded = base64.b64encode(b'user:secret').decode('ascii')
        self.assertIn(f'Authorization: Basic {encoded}', text)

    def test_chunked_body_is_decoded_across_fragments(self):
        client = NtripClient('caster', 8002, 'MOUNT', 'user', 'secret')
        response = (
            b'HTTP/1.1 200 OK\r\n'
            b'TrAnSfEr-EnCoDiNg: chunked\r\n'
            b'Content-Type: gnss/data\r\n\r\n'
            b'4\r\nAB'
        )
        fake = FakeSocket([response, b'CD\r\n3;foo=bar\r\nEF', b'G\r\n0\r\n', b'Trailer: x\r\n\r\n'])
        client._socket = fake
        # Exercise connect parsing with the fake socket without real DNS.
        original = __import__('mobileess_rtk.ntrip', fromlist=['socket']).socket.create_connection
        module = __import__('mobileess_rtk.ntrip', fromlist=['socket'])
        module.socket.create_connection = lambda address, timeout: fake
        try:
            client.connect()
            payload = bytearray()
            while True:
                data = client.read()
                if not data:
                    break
                payload.extend(data)
            self.assertEqual(bytes(payload), b'ABCDEFG')
        finally:
            module.socket.create_connection = original
            client.close()

    def test_body_received_with_headers_is_not_lost(self):
        client = NtripClient('caster', 8002, 'MOUNT', 'user', 'secret')
        fake = FakeSocket([
            b'HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\nRTCM',
        ])
        module = __import__('mobileess_rtk.ntrip', fromlist=['socket'])
        original = module.socket.create_connection
        module.socket.create_connection = lambda address, timeout: fake
        try:
            client.connect()
            self.assertEqual(client.read(), b'RTCM')
        finally:
            module.socket.create_connection = original
            client.close()

    def test_non_200_status_is_rejected_and_socket_closed(self):
        client = NtripClient('caster', 8002, 'MOUNT', 'user', 'secret')
        fake = FakeSocket([b'HTTP/1.1 401 Unauthorized\r\n\r\n'])
        module = __import__('mobileess_rtk.ntrip', fromlist=['socket'])
        original = module.socket.create_connection
        module.socket.create_connection = lambda address, timeout: fake
        try:
            with self.assertRaises(ConnectionError):
                client.connect()
            self.assertTrue(fake.closed)
        finally:
            module.socket.create_connection = original


if __name__ == '__main__':
    unittest.main()
