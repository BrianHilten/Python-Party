# Packet parser for SRL's mission control application
import json
from operator import sub

import zmq
import struct
import json

from collections import deque

ZMQ_SUB_PORT = 5001 # Port to subscribe to via ZMQ
ZMQ_PUB_PORT = 5002 # Port to publish data to (data will be multiparted so subscriber can filter by topic)
START_MARKER = b"\xDE\xAD\xBE\xEF"

zmq_context = zmq.Context()
publisher = zmq_context.socket(zmq.PUB)
publisher.bind(f"tcp://127.0.0.1:{ZMQ_PUB_PORT}") # tcp loopback since no ipc on windows

class PacketParser:
    def __init__(self, size):
        self.size = size
        self.buffer = deque(maxlen=size)

    def print_buffer(self):
        print(self.buffer)

    def append(self, byte):
        self.buffer.append(byte)

    def get(self,index):
        return self.buffer[index]
    
    def calculate_crc32(self, data):
        # Calculate CRC32 checksum for the given data
        import zlib
        #data = int.from_bytes(data, byteorder='big').to_bytes(len(data), byteorder='big')  # Ensure data is in bytes
        return zlib.crc32(data) & 0xffffffff
    
    # Parses data buffer looking for full packets.
    # Returns a full packet as a list of bytes when a complete frame is present, otherwise None.
    # For PHX packets:
    # 0xDEADBEEF is the start of packet (4 bytes)
    # packet_length (1 byte) -> total frame length after the start marker

    def parse_buffer(self):
        if not self.buffer:
            return None

        buffer_bytes = bytes(self.buffer)
        start_index = buffer_bytes.find(START_MARKER)

        if start_index == -1:
            print("Packet start not found; clearing stale bytes")
            self.buffer.clear()
            return None

        if start_index > 0:
            discarded = [self.buffer.popleft() for _ in range(start_index)]
            print(f"Discarded bytes before packet start: {discarded}")
            buffer_bytes = bytes(self.buffer)
            start_index = 0

        if len(buffer_bytes) < 8:
            print("Not enough bytes in buffer to determine packet length. Waiting for more data.")
            return None

        packet_length = buffer_bytes[4]
        total_frame_length = 4 + packet_length
        print(f"Start index of packet: {start_index}")
        print(f"Packet length: {packet_length}")

        if len(buffer_bytes) < total_frame_length:
            print(
                f"Packet length {packet_length} is greater than available bytes {len(buffer_bytes)}. "
                "Waiting for more data."
            )
            return None

        frame = list(buffer_bytes[:total_frame_length])
        for _ in range(total_frame_length):
            self.buffer.popleft()
        return frame
        
    def parse_telemetry(self, payload):
        if len(payload) < 62:
            print(f"Telemetry payload too short for EKF packet: {len(payload)} bytes")
            return None

        print('Parsing telem data:')
        telem_type = payload[0]
        match telem_type:
            case 0x00:
                print("EKF Telemetry Packet Received")
                #parse the ekf packet
                valid = payload[1]
                utc_time = int.from_bytes(payload[2:6], byteorder='big') # NEED TO VERIFY ENDIANNESS
                pos_x = struct.unpack('>f', payload[6:10])[0] # NEED TO VERIFY ENDIANNESS
                pos_y = struct.unpack('>f', payload[10:14])[0] # NEED TO VERIFY ENDIANNESS
                pos_z = struct.unpack('>f', payload[14:18])[0] # NEED TO VERIFY ENDIANNESS
                vel_x = struct.unpack('>f', payload[18:22])[0] # NEED TO VERIFY ENDIANNESS
                vel_y = struct.unpack('>f', payload[22:26])[0] # NEED TO VERIFY ENDIANNESS
                vel_z = struct.unpack('>f', payload[26:30])[0] # NEED TO VERIFY ENDIANNESS
                q0 = struct.unpack('>f', payload[30:34])[0] # NEED TO VERIFY ENDIANNESS
                q1 = struct.unpack('>f', payload[34:38])[0] # NEED TO VERIFY ENDIANNESS
                q2 = struct.unpack('>f', payload[38:42])[0] # NEED TO VERIFY ENDIANNESS
                q3 = struct.unpack('>f', payload[42:46])[0] # NEED TO VERIFY ENDIANNESS
                ang_vel_x = struct.unpack('>f', payload[46:50])[0] # NEED TO VERIFY ENDIANNESS
                ang_vel_y = struct.unpack('>f', payload[50:54])[0] # NEED TO VERIFY ENDIANNESS
                ang_vel_z = struct.unpack('>f', payload[54:58])[0] # NEED TO VERIFY ENDIANNESS  
                mass = struct.unpack('>f', payload[58:62])[0] # NEED TO VERIFY ENDIANNESS
                parsed_packet = [valid, utc_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, q0, q1, q2, q3, ang_vel_x, ang_vel_y, ang_vel_z, mass]
                print(f"Parsed EKF packet: {parsed_packet}")
                parsed_packet = json.dumps(parsed_packet).encode('utf-8') # convert to json and encode to bytes
                #publish the ekf packet w/ ekf topic
                publisher.send_multipart([b"ekf", parsed_packet])
            case 0x01:
                print("Sensor Telemetry Packet Received")
                #parse the sensor packet
                utc_time = int.from_bytes(payload[2:6], byteorder='big') # NEED TO VERIFY ENDIANNESS FOR ALL OF THESE
                high_g_accel_x_int = struct.unpack('>f', payload[6:10])[0]
                high_g_accel_x_frac = struct.unpack('>f', payload[10:14])[0]
                high_g_accel_x = high_g_accel_x_int + (high_g_accel_x_frac / 1000000)
                high_g_accel_y_int = struct.unpack('>f', payload[14:18])[0]
                high_g_accel_y_frac = struct.unpack('>f', payload[18:22])[0]
                high_g_accel_y = high_g_accel_y_int + (high_g_accel_y_frac / 1000000)
                high_g_accel_z_int = struct.unpack('>f', payload[22:26])[0]
                high_g_accel_z_frac = struct.unpack('>f', payload[26:30])[0]
                high_g_accel_z = high_g_accel_z_int + (high_g_accel_z_frac / 1000000)
                imu_accel_x_int = struct.unpack('>f', payload[30:34])[0]
                imu_accel_x_frac = struct.unpack('>f', payload[34:38])[0]
                imu_accel_x = imu_accel_x_int + (imu_accel_x_frac / 1000000)
                imu_accel_y_int = struct.unpack('>f', payload[38:42])[0]
                imu_accel_y_frac = struct.unpack('>f', payload[42:46])[0]
                imu_accel_y = imu_accel_y_int + (imu_accel_y_frac / 1000000)
                imu_accel_z_int = struct.unpack('>f', payload[46:50])[0]
                imu_accel_z_frac = struct.unpack('>f', payload[50:54])[0]
                imu_accel_z = imu_accel_y_int + (imu_accel_y_frac / 1000000)
                imu_gyro_x_int = struct.unpack('>f', payload[54:58])[0]
                imu_gyro_x_frac = struct.unpack('>f', payload[58:62])[0]
                imu_gyro_x = imu_gyro_x_int + (imu_gyro_x_frac / 1000000)
                imu_gyro_y_int = struct.unpack('>f', payload[62:66])[0]
                imu_gyro_y_frac = struct.unpack('>f', payload[66:70])[0]
                imu_gyro_y = imu_gyro_y_int + (imu_gyro_y_frac / 1000000)
                imu_gyro_z_int = struct.unpack('>f', payload[70:74])[0]
                imu_gyro_z_frac = struct.unpack('>f', payload[74:78])[0]
                imu_gyro_z = imu_gyro_z_int + (imu_gyro_z_frac / 1000000)
                pressure_int = struct.unpack('>f', payload[78:82])[0]
                pressure_frac = struct.unpack('>f', payload[82:86])[0]
                pressure = pressure_int + (pressure_frac / 1000000)
                temperature_int = struct.unpack('>f', payload[86:90])[0]
                temperature_frac = struct.unpack('>f', payload[90:94])[0]
                temperature = temperature_int + (temperature_frac / 1000000)
                longitude_int = struct.unpack('>f', payload[94:98])[0]
                longitude_frac = struct.unpack('>f', payload[98:102])[0]
                longitude = longitude_int + (longitude_frac / 1000000)
                latitude_int = struct.unpack('>f', payload[102:106])[0]
                latitude_frac = struct.unpack('>f', payload[106:110])[0]
                latitude = latitude_int + (latitude_frac / 1000000)
                altitude_int = struct.unpack('>f', payload[110:114])[0]
                altitude_frac = struct.unpack('>f', payload[114:118])[0]
                altitude = altitude_int + (altitude_frac / 1000000)
                #publish the sensor packet w/ sensor topic
                parsed_packet = [utc_time, high_g_accel_x, high_g_accel_y, high_g_accel_z, imu_accel_x, imu_accel_y, imu_accel_z, imu_gyro_x, 
                                 imu_gyro_y, imu_gyro_z, pressure, temperature, longitude, latitude, altitude]
                parsed_packet = json.dumps(parsed_packet).encode('utf-8') # convert to json and encode to bytes
                publisher.send_multipart([b"sensor", parsed_packet])
            case 0x02:
                print("State Telemetry Packet Received")
                #parse the state packet
                #publish the state packet w/ state topic
                publisher.send_multipart([b"state", parsed_packet])

    def parse_battery(self, payload):
        print(f"Battery packet received: {payload}")

    def parse_ascii(self, payload):
        print(f"ASCII packet received: {payload}")

    def parse_ack(self, payload):
        print(f"ACK packet received: {payload}")

    # 0xDEADBEEF is the start of packet (4 bytes)
    # packet_length (1 byte) -> total frame length after the start marker
    # packet_type (1 byte) -> 0x01(telem), 0x02(battery), 0x03(ASCII), 0x04(ACK)
    # packet_number (1 byte)
    # payload_length (1 byte) -> length of payload
    # payload (variable length)
    # CRC32 (4 bytes, big endian)
    def parse_packet(self, packet):
        if not packet or len(packet) < 8:
            print("Packet too short to parse")
            return None

        if packet[:4] != START_MARKER:
            print("Packet marker mismatch")
            return None

        print(f"Parsing packet: {packet}")
        frame_without_marker = packet[4:]
        packet_length = frame_without_marker[0]
        if len(frame_without_marker) != packet_length:
            print(f"Packet length mismatch: expected {packet_length} bytes after marker, got {len(frame_without_marker)}")
            return None

        packet_type = frame_without_marker[1]
        print(f'Packet Type: {packet_type}')
        packet_number = frame_without_marker[2]
        payload_length = frame_without_marker[3]
        if len(frame_without_marker) < 4 + payload_length + 4:
            print("Packet payload is truncated")
            return None

        payload = frame_without_marker[4:4+payload_length]
        crc32 = int.from_bytes(frame_without_marker[-4:], byteorder='big')
        packet_without_crc = frame_without_marker[:-4]
        if crc32 != self.calculate_crc32(packet_without_crc):
            print(f"CRC32 mismatch! Calculated: {self.calculate_crc32(packet_without_crc)}, Received: {crc32}")
            return None

        match packet_type:
            case 0x01:
                print(f"Telemetry packet received: {payload}")
                self.parse_telemetry(payload)
            case 0x02:
                print(f"Battery packet received: {payload}")
                self.parse_battery(payload)
            case 0x03:
                print(f"ASCII packet received: {payload}")
                self.parse_ascii(payload)
            case 0x04:
                print(f"ACK packet received: {payload}")
                self.parse_ack(payload)
            case _:
                print(f"Unknown packet type received: {packet_type}")

try:
    with zmq_context.socket(zmq.SUB) as subscriber:
        subscriber.connect(f"tcp://127.0.0.1:{ZMQ_SUB_PORT}")
        subscriber.setsockopt(zmq.SUBSCRIBE, b'')
        print(f'Subscribed to {ZMQ_SUB_PORT}!')

        #Set up parser
        parser = PacketParser(1024)

        while True:
            data = subscriber.recv()
            print(f"Received data over ZMQ!: {data}")
            for byte in data:
                parser.append(byte)
            #print(f"Current buffer state: {parser.print_buffer()}")
            packet = parser.parse_buffer()
            if packet is not None:
                packet = bytes(packet)
                print(f"Full packet found: {packet}")
                parser.parse_packet(packet)

except KeyboardInterrupt:
    print("Exiting Parser")
finally:
    zmq_context.term()