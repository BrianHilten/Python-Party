# This is the backend serial handler for SRL's mission control application
# 1. Open a UDP socket for receiving continues bytes from the CM5 over ethernet
# 2. Take those bytes and push them via ZMQ so the dedicated parser can pick them up
# 3. That's literally it...
import socket
import zmq

HOST = '0.0.0.0'
PORT = 5000 # Port to listen on
ZMQ_PORT = 5001 # Port to publish to via ZMQ

try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_port:
        udp_port.bind((HOST, PORT)) # sets udp_port to that address, port

        #set up ZMQ
        zmq_context = zmq.Context()
        publisher = zmq_context.socket(zmq.PUB)
        publisher.bind(f"tcp://127.0.0.1:{ZMQ_PORT}") # tcp loopback since no ipc on windows

        # Core loop
        while True:
            cm5_data, addr = udp_port.recvfrom(1024) # blocks and waits for new connection 
            print(f"Connected to {addr}")
            print(f"Received: {len(cm5_data)} bytes from {addr}: {cm5_data}")
            if not cm5_data:
                print("Nothing recv")
                break
            try: 
                publisher.send(cm5_data)
                print(f"Published {len(cm5_data)} bytes via ZMQ to {ZMQ_PORT}: {cm5_data}")
            except:
                print("Failed to publish over ZMQ")
            
            # reply = f"Echo: {cm5_data.decode()}".encode()
            # udp_port.sendto(reply, addr)

except KeyboardInterrupt:
    print("Exiting serial handler")    
# cleanup
udp_port.close()
publisher.close()
zmq_context.term()      

