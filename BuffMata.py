# Class to read in values either from a controller (Teensy_RC) or the pilot model to send PWM commands to 
# the control Teensy
import serial
import time

class BuffMata:
    def __init__(self, STEERING_PIN, THROTTLE_PIN, serial_device):
        try:
            self.ser = serial_device
            self.steeringPin = STEERING_PIN
            self.throttlePin = THROTTLE_PIN
            self.running = True
            print(r"""
                                _.-````'-,_
                      _,.,_ ,-'`           `'-.,_
                    /)     (\                   '``-.
                    ((      ) )                      `\
                    \)    (_/                        )\
                    |       /)           '    ,'    / \
                    `\    ^'            '     (    /  ))
                      |      _/\ ,     /    ,,`\   (  "`
                       \Y,   |  \  \  | ````| / \_ \
                        `)_/    \  \  )    ( >  ( >
                                 \( \(     |/   |/
                                /_(/_(    /_(  /_(
                             ____  _   _   _    ____  
                            / ___|| | | | / \  |  _ \ 
                            | |   | | | |/ _ \ | |_) |
                            | |___| |_| / ___ \|  _ < 
                            \____| \___/_/   \_\_| \_\
            
                  """)
            print("BuffMata created")
        except:
            print("Failed to create BuffMata")
            print("Exiting")
            exit()
    
    def mapSteerPWM(self, steering):
        return 90 + steering*45 # Full Left = 45 (-1); Full Right = 135 (1); Center = 90 (0)
    
    def mapThrottlePWM(self, throttle):
        return 90 + throttle*30 # Full Throttle = 120 (1); Full Reverse = 70 (-1); Deadzone = 90-98 (0-ish)
    
    def checkSum(self, packet): # Simple XOR Checksum, might not be sufficient 
        cs = 0
        for byte in packet.encode():
            cs ^= byte
        return cs
    
    def run(self, steering, throttle):
        steering = self.mapSteerPWM(max(-1.0, min(1.0, steering))) # Clamping
        throttle = self.mapThrottlePWM(max(-1.0, min(1.0, throttle))) # Clamping
        cmd = f"{self.steeringPin},{int(steering)},{self.throttlePin},{int(throttle)}"
        cs = self.checkSum(cmd) # Checksum for packet loss validation
        packet = f"<{cmd}|{cs}>\n" 
        # '<' packet start; {steerpin,steerPWM,throttlepin,throttlePWM}; '|' Cmd/cs Delimiter; '>' packet end; '\n' for marking end of line
        self.ser.write(packet.encode())

    def shutdown(self):
        # set steering straight
        cmd = f"{self.steeringPin} {90}\n" # Sends steering command as PWM between 0-255
        self.ser.write(cmd.encode())
        cmd = f"{self.throttlePin} {90}\n" # Sends throttle command as PWM between 0-255
        self.ser.write(cmd.encode())
        time.sleep(0.3)
        self.running = False
        self.ser.close()
        print("BuffMata Shutting Down")
# Use this in add_drivetrain method in manage.py
# if cfg.DRIVE_TRAIN_TYPE == "BUFFMATA":
#         drivetrain = BuffMata(cfg.STEERING_ARDUINO_PIN, cfg.THROTTLE_ARDUINO_PIN, serial_device)
#         V.add(drivetrain, inputs=['steering','throttle'])
