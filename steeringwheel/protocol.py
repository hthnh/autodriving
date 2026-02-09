# protocol.py

PKT_HEADER = 0xAA

CMD_DRIVE = 0x01
CMD_PING  = 0xF0


def calc_crc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def build_packet(cmd: int, payload: bytes = b'') -> bytes:
    length = 1 + len(payload)  # CMD + PAYLOAD
    frame = bytes([PKT_HEADER, length, cmd]) + payload
    crc = calc_crc(frame)
    return frame + bytes([crc])
