# Packet parser for SRL's mission control application
import json
from operator import sub

import zmq
import struct
import json

from collections import deque

ZMQ_SUB_PORT = 5001 # Port to subscribe to via ZMQ
ZMQ_PUB_PORT = 5002 # Port to publish data to (data will be multiparted so subscriber can filter by topic)

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
    
    # Parses data buffer looking for full packets
    # Return: index of packet start and full packet (as a list) or NONE if no full packet found
    # For PHX packets:
    # 0xDEADBEEF is the start of packet (4 bytes)
    # packet_length (1 byte) -> payload length + 8)

    def parse_buffer(self):
        # Super simple, just looks for the start of packet and then checks if the buffer has enough bytes for a full packet
        # If it does, it returns the full packet as a list. If not, it returns None
        # If buffer has a partial packet, it will wait for more data to be appended to the buffer before returning a full packet
        try:
            buffer_bytes = bytes(self.buffer)
            start_index = buffer_bytes.find(b"\xDE\xAD\xBE\xEF")
            print(f"Start index of packet: {start_index}")
            # start_index = self.buffer.index(b"\xDE\xAD\xBE\xEF") # find the start of the packet')
            discard = [self.buffer.popleft() for byte in range(start_index)] # discard any bytes before the start of the packet
            print(f"Discarded bytes before packet start: {discard}")
            if start_index + 4 < len(self.buffer):
                packet_length = self.buffer[start_index + 4] # get the packet length from the buffer
                print(f"Packet length: {packet_length}")
                if packet_length > len(self.buffer) - start_index:
                    print(f"Packet length {packet_length} is greater than buffer length {len(self.buffer) - start_index}. Waiting for more data.")
                    return None
                
                packet = [self.buffer.popleft() for byte in range(min(start_index + packet_length, len(self.buffer)))]
                return packet
            else:
                print("Not enough bytes in buffer to determine packet length. Waiting for more data.")
                return None
            
        except ValueError:
            print('Packet start not found')
            return None
        
    def parse_telemetry(self, payload):
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
                #publish the sensor packet w/ sensor topic
                publisher.send_multipart([b"sensor", payload])
            case 0x02:
                print("State Telemetry Packet Received")
                #parse the state packet
                #publish the state packet w/ state topic
                publisher.send_multipart([b"state", payload])

    # 0xDEADBEEF is the start of packet (4 bytes)
    # packet_length (1 byte) -> payload length + 8
    # packet_type (1 byte) -> 0x01(telem), 0x02(battery), 0x03(ASCII), 0x04 (ACK)
    # packet_number (1 byte)
    # payload_length (1 byte) -> length of payload
    # payload (variable length)
    # CRC32 (4 bytes, big endian)
    def parse_packet(self, packet):
        print(f"Parsing packet: {packet}")
        packet = packet[4:] # remove the start of packet
        packet_length = packet[0]
        packet_type = packet[1]
        print(f'Packet Type: {packet_type}')
        packet_number = packet[2]
        payload_length = packet[3]
        payload = packet[4:4+payload_length]
        crc32 = int.from_bytes(packet[-4:], byteorder='big') # last 4 bytes are the CRC32
        packet = packet[:-4] # remove the CRC32
        # if(crc32 != self.calculate_crc32(packet)):
        #     print(f"CRC32 mismatch! Calculated: {self.calculate_crc32(packet)}, Received: {crc32}")
        #     return None
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