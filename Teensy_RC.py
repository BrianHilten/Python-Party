class Teensy_RC:
    def __init__(self, serial_device):
        try:
            self.ser = serial_device 
            self.steering = 0.0
            self.throttle = 0.0
            self.running = True
            self.buffer = ""
            self.packet_found = False
            self.recording = False
            self.mode = 'user'
            self.lock = threading.Lock()
            print("Teensy RC thread created successfully!")
        except:
            print("Failed to create Teensy RC")
            exit()
    
    def verifyCheckSum(self, parsed_packet, received_check_sum):
        # print(f"Received CS: {received_check_sum}")
        # print(f"Received Packet: {parsed_packet}")
        check_sum = 0
        for c in parsed_packet:
            check_sum ^= ord(c)
        # print(f"Check Sum: {check_sum}")
        return int(received_check_sum) == check_sum
    
    def parsePacket(self, buffer):
        buffer = buffer[1:-1] # Strip our frame delimiters
        parsed_packet = buffer.split('|') # Split into command data and checksum
        if(self.verifyCheckSum(parsed_packet[0], parsed_packet[1])):
            commands = parsed_packet[0].split(',') # Split into Steering and Throttle values
            steering = float(commands[0]) # Steering will be sent first, already turned into a +/- 1 value by the teensy for PWM
            throttle = float(commands[1]) # Throttle sent next, already turned into a +/- 1 value by the teensy for PWM
            recording = bool(commands[2]) # Recording Mode (1 == Recording, 0 == Not Recording)
            return steering, throttle, recording
        else:
            print("Full Teensy RC packet not detected. Returning.")
            return None
    
    def update(self):
        while self.running:
            try:
                char = self.ser.read().decode('utf-8')
                if char == '<':
                    self.buffer += "<"
                    self.packet_found = True
                elif (self.packet_found):
                    self.buffer += char
                    if char == '>':
                        print(f"Full Packet: {self.buffer}")
                        commands = self.parsePacket(self.buffer) # Read in our packet
                        self.buffer = ""
                        self.packet_found = False
                        if commands: # Only updates if commands are available
                            print(f"Steering: {commands[0]} Throttle: {commands[1]} Recording: {commands[2]}")
                            with self.lock: # For thread safety
                                self.steering = commands[0] # Update steering
                                self.throttle = commands[1] # Update throttle
                                self.recording = commands[2] # Update recording mode
                
            except Exception as e:
                print("No incoming control data detected: {e}")
    
    def run_threaded(self): # Required for threading by vehicle.py
        with self.lock:
            return self.steering, self.throttle, self.recording, self.mode
    
    def run(self): # Required by vehicle.py
        return self.run_threaded()
        # self.update()
        # return self.steering, self.throttle
        
    def shutdown(self): # Required for threading by vehicle.py
        self.running = False
        print("Teensy_RC Shutting Down")