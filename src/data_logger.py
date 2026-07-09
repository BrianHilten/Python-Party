# Data logger process:
# Allows user to subscribe to multiple data streams and log them to a file
import zmq
import json
import csv
import os

ZMQ_SUB_PORT = 5002 # Port to subscribe to via ZMQ

zmq_context = zmq.Context()

# 0 = not logging, 1 = logging
log_channels = {
    "ekf": 1,
    "sensor": 0,
    "state": 0,
}
try:
    with zmq_context.socket(zmq.SUB) as subscriber:
        subscriber.connect(f"tcp://127.0.0.1:{ZMQ_SUB_PORT}")
        subscriber.setsockopt(zmq.SUBSCRIBE, b'')
        print(f'Subscribed to {ZMQ_SUB_PORT}!')

        while True:
            topic, payload = subscriber.recv_multipart()
            print(f"Received data over ZMQ!: {topic}: {payload}")
            if log_channels.get(topic.decode('utf-8'), 0) == 1:
                # Log the data to a file
                with open(f"{topic.decode('utf-8')}_log.csv", "a", newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    data = json.loads(payload.decode("utf-8"))
                    writer.writerow(data)
                pass

except KeyboardInterrupt:
    print("Exiting Parser")
finally:
    zmq_context.term()