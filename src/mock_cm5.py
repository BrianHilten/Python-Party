import socket
import struct
import time
import zlib

HOST = "127.0.0.1"
PORT = 5000


def build_packet(packet_type: int, payload: bytes) -> bytes:
    start_marker = b"\xDE\xAD\xBE\xEF"
    packet_length = len(payload) + 8
    packet_number = 0x01
    payload_length = len(payload)

    header = bytes([packet_length, packet_type, packet_number, payload_length])
    crc = struct.pack(">I", zlib.crc32(header + payload) & 0xFFFFFFFF)
    return start_marker + header + payload + crc


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


def main() -> None:
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((HOST, PORT))
        print(f"Mock CM5 sending UDP packets to {HOST}:{PORT}")

        while True:
            user_input = input("Enter 'send' to transmit a sample EKF packet, or 'exit' to quit: ").strip().lower()
            if user_input == "exit":
                print("Exiting Mock CM5")
                break

            if user_input != "send":
                print("Type 'send' to transmit a sample packet")
                continue

            payload = make_sample_ekf_payload()
            packet = build_packet(0x01, payload)
            sock.send(packet)
            print(f"Sent sample telemetry packet ({len(packet)} bytes) over UDP")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting Mock CM5")
    finally:
        if sock is not None:
            sock.close()


if __name__ == "__main__":
    main()
