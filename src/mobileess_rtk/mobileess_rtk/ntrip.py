"""Minimal NTRIP client for forwarding decoded RTCM corrections."""
import base64
import socket
from typing import Dict, Optional



def build_request(host: str, port: int, mountpoint: str, username: str,
                  password: str) -> bytes:
    token = base64.b64encode(f'{username}:{password}'.encode('utf-8')).decode('ascii')
    mountpoint = mountpoint.lstrip('/')
    return (
        f'GET /{mountpoint} HTTP/1.1\r\n'
        f'Host: {host}:{port}\r\n'
        'Ntrip-Version: Ntrip/2.0\r\n'
        'User-Agent: NTRIP MobileEss/0.1\r\n'
        'Accept: */*\r\n'
        f'Authorization: Basic {token}\r\n\r\n'
    ).encode('ascii')


class _ChunkDecoder:
    """Incrementally remove HTTP chunk framing from a response body."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._remaining: Optional[int] = None
        self._done = False
        self._expect_data_crlf = False
        self._reading_trailers = False

    @property
    def done(self) -> bool:
        return self._done

    def feed(self, data: bytes) -> bytes:
        self._buffer.extend(data)
        output = bytearray()
        while not self._done:
            if self._reading_trailers:
                end = self._buffer.find(b'\r\n\r\n')
                if end >= 0:
                    del self._buffer[:end + 4]
                    self._done = True
                elif self._buffer == b'\r\n':
                    self._buffer.clear()
                    self._done = True
                break

            if self._expect_data_crlf:
                if len(self._buffer) < 2:
                    break
                if self._buffer[:2] != b'\r\n':
                    raise ConnectionError('invalid chunk data terminator')
                del self._buffer[:2]
                self._expect_data_crlf = False
                continue

            if self._remaining is None:
                end = self._buffer.find(b'\r\n')
                if end < 0:
                    break
                line = bytes(self._buffer[:end])
                del self._buffer[:end + 2]
                size_text = line.split(b';', 1)[0].strip()
                try:
                    self._remaining = int(size_text, 16)
                except ValueError as exc:
                    raise ConnectionError('invalid chunk size') from exc
                if self._remaining == 0:
                    self._reading_trailers = True
                    self._remaining = None
                    continue

            if self._remaining is not None:
                if not self._buffer:
                    break
                count = min(self._remaining, len(self._buffer))
                output.extend(self._buffer[:count])
                del self._buffer[:count]
                self._remaining -= count
                if self._remaining == 0:
                    self._remaining = None
                    self._expect_data_crlf = True
        return bytes(output)


class NtripClient:
    def __init__(self, host: str, port: int, mountpoint: str, username: str,
                 password: str, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.timeout = timeout
        self._socket: Optional[socket.socket] = None
        self._body_buffer = bytearray()
        self._decoded_buffer = bytearray()
        self._chunked = False
        self._decoder: Optional[_ChunkDecoder] = None

    def connect(self, gga: Optional[str] = None) -> None:
        sock = socket.create_connection((self.host, self.port), self.timeout)
        try:
            sock.settimeout(self.timeout)
            sock.sendall(build_request(self.host, self.port, self.mountpoint,
                                       self.username, self.password))
            response = bytearray()
            while b'\r\n\r\n' not in response and len(response) < 8192:
                chunk = sock.recv(1024)
                if not chunk:
                    raise ConnectionError('NTRIP caster closed during response')
                response.extend(chunk)
            separator = response.find(b'\r\n\r\n')
            if separator < 0:
                raise ConnectionError('NTRIP response headers are incomplete')
            header_bytes = bytes(response[:separator])
            body = bytes(response[separator + 4:])
            lines = header_bytes.split(b'\r\n')
            status_line = lines[0].decode('latin1', errors='replace')
            status_parts = status_line.split()
            if status_line.upper().startswith('ICY '):
                status_code = status_parts[1] if len(status_parts) > 1 else ''
            else:
                status_code = status_parts[1] if len(status_parts) > 1 else ''
            if status_code != '200':
                raise ConnectionError(header_bytes.decode('latin1', errors='replace'))
            headers: Dict[str, str] = {}
            for line in lines[1:]:
                if b':' in line:
                    key, value = line.split(b':', 1)
                    headers[key.decode('latin1').strip().lower()] = (
                        value.decode('latin1').strip().lower())
            self._chunked = 'chunked' in headers.get('transfer-encoding', '')
            self._decoder = _ChunkDecoder() if self._chunked else None
            self._body_buffer = bytearray(body)
            self._decoded_buffer.clear()
            if gga:
                sock.sendall((gga.strip() + '\r\n').encode('ascii'))
            self._socket = sock
        except Exception:
            try:
                sock.close()
            except OSError:
                pass
            raise

    def read(self, size: int = 4096) -> Optional[bytes]:
        if self._socket is None:
            raise ConnectionError('NTRIP client is not connected')
        while True:
            if self._chunked and self._decoder is not None:
                if self._decoded_buffer:
                    data = bytes(self._decoded_buffer[:size])
                    del self._decoded_buffer[:size]
                    return data
                decoded = self._decoder.feed(bytes(self._body_buffer))
                self._body_buffer.clear()
                if decoded:
                    self._decoded_buffer.extend(decoded)
                    continue
                if self._decoder.done:
                    return b''
            elif self._body_buffer:
                data = bytes(self._body_buffer[:size])
                del self._body_buffer[:size]
                return data

            try:
                data = self._socket.recv(size)
            except socket.timeout:
                return None
            if not data:
                return b''
            if self._chunked and self._decoder is not None:
                self._body_buffer.extend(data)
            else:
                return data

    def send_gga(self, gga: str) -> None:
        if self._socket is not None:
            self._socket.sendall((gga.strip() + '\r\n').encode('ascii'))

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        self._body_buffer.clear()
        self._decoder = None
        self._decoded_buffer.clear()
        self._chunked = False
