# Mock Data Source so we can test the pipeline.
# Author: Claude
import socket
import struct
import time
import zlib

HOST = "127.0.0.1"
PORT = 5000

# Packet types (goes in the frame header)
PACKET_TYPE_EKF = 0x01
PACKET_TYPE_SENSOR = 0x01

# Telemetry types (first byte inside the payload itself)
TELEM_TYPE_EKF = 0x00
TELEM_TYPE_SENSOR = 0x01


def build_packet(packet_type: int, payload: bytes, packet_number: int = 0x01) -> bytes:
    start_marker = b"\xDE\xAD\xBE\xEF"
    packet_length = len(payload) + 8
    payload_length = len(payload)

    header = bytes([packet_length, packet_type, packet_number, payload_length])
    crc = struct.pack(">I", zlib.crc32(header + payload) & 0xFFFFFFFF)
    return start_marker + header + payload + crc


def build_invalid_packet(kind: str) -> bytes:
    valid_packet = build_packet(PACKET_TYPE_EKF, make_sample_ekf_payload())

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
    telem_type = TELEM_TYPE_EKF
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
    telem_type = TELEM_TYPE_EKF
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


def encode_int_frac(value: float) -> tuple[int, int]:
    """Split a float into (int_part, frac_part) such that
    value == int_part + frac_part / 1_000_000, matching the receiver's
    decode logic. frac_part carries the same sign as the value so negative
    numbers round-trip correctly.
    """
    int_part = int(value)  # truncates toward zero, same as the receiver assumes
    frac_part = round((value - int_part) * 1_000_000)
    return int_part, frac_part


def make_sensor_payload(utc_time: int, values: tuple) -> bytes:
    """Builds a sensor payload matching the receiver's parsing layout:
    telem_type(B) valid(B) utc_time(I) then 14x [int_part(i) frac_part(i)]
    in the order: high_g_accel(x,y,z), imu_accel(x,y,z), imu_gyro(x,y,z),
    pressure, temperature, longitude, latitude, altitude.
    """
    telem_type = TELEM_TYPE_SENSOR
    valid = 0x01

    if len(values) != 14:
        raise ValueError(f"Expected 14 sensor values, got {len(values)}")

    parts = []
    for v in values:
        int_part, frac_part = encode_int_frac(v)
        parts.append(int_part)
        parts.append(frac_part)

    return struct.pack(">BBI" + "ii" * 14, telem_type, valid, utc_time, *parts)


def make_sample_sensor_payload() -> bytes:
    utc_time = 1700000000
    values = (
        0.12,    # high_g_accel_x
        -0.05,   # high_g_accel_y
        9.79,    # high_g_accel_z
        0.10,    # imu_accel_x
        -0.02,   # imu_accel_y
        9.81,    # imu_accel_z
        0.001,   # imu_gyro_x
        -0.003,  # imu_gyro_y
        0.0005,  # imu_gyro_z
        101325.0,   # pressure (Pa)
        22.5,       # temperature (C)
        -104.9903,  # longitude
        39.7392,    # latitude
        1609.3,     # altitude (m)
    )
    return make_sensor_payload(utc_time, values)


def make_sample_sensor_payload_variant() -> bytes:
    utc_time = 1700010000
    values = (
        0.34,
        -0.11,
        9.75,
        0.22,
        -0.04,
        9.77,
        0.0015,
        -0.0042,
        0.0009,
        101280.5,
        23.1,
        -104.9911,
        39.7401,
        1622.7,
    )
    return make_sensor_payload(utc_time, values)


def main() -> None:
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((HOST, PORT))
        print(f"Mock CM5 sending UDP packets to {HOST}:{PORT}")

        packet_sequence = [
            ("valid_ekf_1", PACKET_TYPE_EKF, make_sample_ekf_payload()),
            ("valid_sensor_1", PACKET_TYPE_SENSOR, make_sample_sensor_payload()),
            ("valid_ekf_2", PACKET_TYPE_EKF, make_sample_ekf_payload_variant()),
            ("valid_sensor_2", PACKET_TYPE_SENSOR, make_sample_sensor_payload_variant()),
            ("invalid_bad_crc", None, None),
            ("valid_ekf_3", PACKET_TYPE_EKF, make_sample_ekf_payload()),
            ("valid_sensor_3", PACKET_TYPE_SENSOR, make_sample_sensor_payload()),
            ("invalid_bad_marker", None, None),
            ("valid_ekf_4", PACKET_TYPE_EKF, make_sample_ekf_payload_variant()),
            ("valid_sensor_4", PACKET_TYPE_SENSOR, make_sample_sensor_payload_variant()),
            ("invalid_truncated", None, None),
            ("valid_ekf_5", PACKET_TYPE_EKF, make_sample_ekf_payload()),
            ("valid_sensor_5", PACKET_TYPE_SENSOR, make_sample_sensor_payload()),
            ("invalid_bad_length", None, None),
            ("valid_ekf_6", PACKET_TYPE_EKF, make_sample_ekf_payload_variant()),
            ("valid_sensor_6", PACKET_TYPE_SENSOR, make_sample_sensor_payload_variant()),
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
