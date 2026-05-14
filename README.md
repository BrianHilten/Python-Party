# Python-Party
Personal repo for python related projects, particularly for CU Autonomous Racing Team  
BuffMata.py: An actuator class for use with Donkeycar. It is primarily responsible for transmitting target throttle and steering positions to a microcontroller.  
Teensy_RC.py: A controller class for use with Donkeycar. It is primarily responsible for interpreting radio controller commands sent from a microcontroller over serial communication. It then records the received packet information in channel values for use by the AI and BuffMata.  
Teensy_Buffmata_Interface.ino: This code runs on a Teensy 4.1. It is responsible for sending and receiving packets to/from the Jetson Nano. It interprets SBUS data sent via a radio controller, and then uses various feedback PID/PD loops to output PWM signals to control the braking (linear actuator), steering (DC motor), and throttle (DC motor) subsystems. It also has error logging functionality.  
