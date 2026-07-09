import socket
import struct
import time
import zlib

HOST = "127.0.0.1"
PORT = 5000


def build_packet(packet_type: int, payload: bytes, packet_number: int = 0x01) -> bytes:
    start_marker = b"\xDE\xAD\xBE\xEF"
    packet_length = len(payload) + 8
    payload_length = len(payload)

    header = bytes([packet_length, packet_type, packet_number, payload_length])
    crc = struct.pack(">I", zlib.crc32(header + payload) & 0xFFFFFFFF)
    return start_marker + header + payload + crc


def build_invalid_packet(kind: str) -> bytes:
    valid_packet = build_packet(0x01, make_sample_ekf_payload())

    if kind == "bad_crc":
        bad_crc = bytearray(valid_packet)
        bad_crc[-1] ^= 0x01
        return bytes(bad_crc)

    if kind == "bad_marker":
        return b"\x00" + valid_packet[1:]

    if kind == "truncated":
        return valid_packet[:-2]

    if kind == "bad_length":
        bad_length = bytearray(valid_packet)
        bad_length[4] = 0x00
        return bytes(bad_length)

    if kind == "short_header":
        return b"\xDE\xAD\xBE\xEF\x01\x01"

    return valid_packet


def make_sample_ekf_payload() -> bytes:
    telem_type = 0x00
    valid = 0x01
    utc_time = 1700000000
    values = (
        1.0,
        2.0,
        3.0,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
        0.7,
        0.8,
        0.9,
        9.81,
        1.23,
    )
    return struct.pack(">BBI" + "f" * len(values), telem_type, valid, utc_time, *values)


def make_sample_ekf_payload_variant() -> bytes:
    telem_type = 0x00
    valid = 0x01
    utc_time = 1700010000
    values = (
        4.0,
        5.0,
        6.0,
        0.11,
        0.22,
        0.33,
        0.44,
        0.55,
        0.66,
        0.77,
        0.88,
        0.99,
        10.0,
        1.5,
    )
    return struct.pack(">BBI" + "f" * len(values), telem_type, valid, utc_time, *values)


def main() -> None:
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((HOST, PORT))
        print(f"Mock CM5 sending UDP packets to {HOST}:{PORT}")

        packet_sequence = [
            ("valid_ekf_1", 0x01, make_sample_ekf_payload()),
            ("valid_ekf_2", 0x01, make_sample_ekf_payload_variant()),
            ("invalid_bad_crc", None, None),
            ("valid_ekf_3", 0x01, make_sample_ekf_payload()),
            ("invalid_bad_marker", None, None),
            ("valid_ekf_4", 0x01, make_sample_ekf_payload_variant()),
            ("invalid_truncated", None, None),
            ("valid_ekf_5", 0x01, make_sample_ekf_payload()),
            ("invalid_bad_length", None, None),
            ("valid_ekf_6", 0x01, make_sample_ekf_payload_variant()),
            ("invalid_short_header", None, None),
        ]
        next_index = 0

        while True:
            name, packet_type, payload = packet_sequence[next_index]
            if name.startswith("invalid"):
                packet = build_invalid_packet(name.split("_", 1)[1])
                print(f"Sent malformed packet ({name}) ({len(packet)} bytes) over UDP")
            else:
                packet = build_packet(packet_type, payload)
                print(f"Sent {name} packet ({len(packet)} bytes) over UDP")

            sock.send(packet)
            next_index = (next_index + 1) % len(packet_sequence)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting Mock CM5")
    finally:
        if sock is not None:
            sock.close()


if __name__ == "__main__":
    main()
