from websockets.sync.client import connect
import numpy as np
import numpy.typing as npt
import gzip
import struct
import typing

formats = {
    0: 1
}

def int_encode(value: int) -> bytes:
    return int(value).to_bytes(byteorder="little", length=8, signed=True)

def int_decode(value: bytes) -> int:
    return int().from_bytes(value, byteorder="little", signed=True)

def double_encode(value: float) -> bytes:
    return struct.pack("d", value)

def double_decode(value: bytes) -> float:
    return float(struct.unpack("d", value)[0])

class VideoCapture:
    def __init__(self, vp_id: int = 0, url: str = "ws://localhost:9080"):
        self.ws = connect(url, max_size=2**64)
        self.vp_id = vp_id
        self.rows, self.columns, self.format = self.ping()
        self.channels = formats[self.format]

    def ping(self):
        self.ws.send(b"\x00" + int_encode(self.vp_id))
        data = self.ws.recv()
        assert b"\x00" == data[0:1]
        assert self.vp_id == int_decode(data[1:9])
        return int_decode(data[9:17]), int_decode(data[17:25]), int_decode(data[25:33])


    def read(self, debug: bool = False) -> typing.Tuple[bool, npt.ArrayLike]:
        try:
            self.ws.send(b"\x04" + int_encode(self.vp_id))
            data = self.ws.recv()
            assert b"\x04" == data[0:1]
            assert self.vp_id == int_decode(data[1:9])
            compressed_data = data[9:]
            bytes_data = gzip.decompress(compressed_data)
            ravel_data = np.frombuffer(bytes_data, dtype=np.uint8)
            img = np.reshape(ravel_data, (self.rows, self.columns, self.channels))
            return True, img
        except Exception as err:
            if debug:
                raise err
            return False, None

    def double_echo(self):
        test_num = float(np.pi)
        self.ws.send(b"\x05" + double_encode(test_num))
        data = self.ws.recv()
        assert b"\x05" == data[0:1]
        val = double_decode(data[1:9])
        #print(f"{test_num} -> {val}")

    def release(self):
        self.ws.close()


class Control:
    def __init__(self, vp_id: int = 0, url: str = "ws://localhost:9080"):
        self.ws = connect(url, max_size=2**64)

    def move_to(self, x: float, y: float, z: float):
        self.ws.send(b"\x01" + double_encode(x) + double_encode(y) + double_encode(z))
        data = self.ws.recv()
        assert b"\x01" == data[0:1]