/*
THIS SKETCH IS INTENDED TO INTERFACE WITH BUFFMATA TO RECEIVE PWM COMMANDS
SENT BY DONKEYCAR VIA PYSERIAL. IT EXPECTS PACKETS IN THE FOLLOWING FORMAT:
<cmd|cs>\n
Where cmd = steering_pin,steering_pwm,throttle_brake_switch,throttle_pwm
Where cs = Checksum (basic XOR, considering upgrading to CRC-8 if needed)
Example Packet:
<6,120,1,100|23>\n
---------------------------------------------------------------------------------------------------------------------------------------
IN FULL FUNCTIONAL KART MODE, THE RADIO TRANSMITS SBUS DATA WHICH CONTAINS STEERING AND THROTTLE COMMANDS (PLUS SOME ADDITIONAL INFO).
THIS INFORMATION IS ASSEMBLED INTO A PACKET WHICH IS SENT TO THE JETSON NANO OVER USB.
THE JETSON NANO WILL REPLY WITH THE ACTUATOR TARGET VALUES.
THE FEEDBACK CONTROL SYSTEM WILL MOVE THE ACTUATORS UNTIL THEY MATCH THE COMMANDED TARGET POSITION.

USING CONTROLLER == TRUE (TRAINING MODE):
RADIO-->TEENSY-->JETSON-->TEENSY-->ACTUATORS

AI MODE:
JETSON-->TEENSY-->ACTUATORS

---------------------------------------------------------------------------------------------------------------------------------------
TO DO:
-Clean this up. The random globals give me anxiety.

Authors: Brian Hilten and Alexander Gholmieh for CU Autonomous Racing Fall 2025-Spring 2026
*/
#include "sbus.h"
#include <SD.h>
#include <SPI.h>

// You MUST choose one of these. Comment out the one you don't want to use
//#define PROTOTYPE
#define KART
//#define LOGGER // For logging to in built SD card slot on Teensy
//#define JETSON_WATCHDOG
#define MAX_JETSON_TIME_LAPSE 15000
//-----------------------
#define STEERING_PIN 6 // for our prototype
#define THROTTLE_PIN 5 //Throttle control
#define USING_CONTROLLER true //If true --> SBUS scanning will occur every loop, only use during training
#define SBUS_CHANNEL_1 0 //Our expected channel from the R8EF STEERING
#define SBUS_CHANNEL_3 2 //Our expected channel from the R8EF THROTTLE
#define SBUS_CHANNEL_5 4 //Our expected channel monitoring the kill switch command
#define SBUS_CHANNEL_6 5 //Channel for delete last n records command on Jetson
#define SBUS_CHANNEL_7 6 //For toggling recording (200(L) == Don't record, 1800(H) == Record)

#define POTENTIOMETER_PIN_STEERING A2
#define STEERING_PIN_RIGHT 15 //A1 //Right is capital A on H-Bridge
#define STEERING_PIN_LEFT 25 //Left is capital B-Bridge
#define STEERINGPWMPIN 2
// Right or left should be both 0 to turn off and only one on to turn right

#define THROTTLE_KART_PIN A4 //18
#define MAX_THROTTLE 120 //Update on nano

#define POT_VOLTAGE_WHEELS_STRAIGHT 1.37f
#define POT_VOLTAGE_WHEELS_MIN  0.75f   // .57 is actual min
#define POT_VOLTAGE_WHEELS_MAX  2.0f   // 2.2 is actual max 
#define WHEELS_DEGREES_OF_FREEDOM 14.0f
#define MAX_VOLTAGE 3.3f

#define KP_VAL  9.0f
#define KI_VAL  0.5f
#define KD_VAL  0.5f
#define KP_VAL_BRAKE 10.0f
#define KD_VAL_BRAKE 0.5f

#define brakingMaxPotentiometer 1.0f //Actual is 1 V
#define brakingMinPotentiometer 0.04f
#define brakingMidPotentiometer 0.25f

#define BRAKING_RETRACT_PIN 14 //A0   Retracts  lowercase a on h bridge 
#define BRAKING_EXTEND_PIN 24 //A10   EXTENDS   lowercase b on h bridge
#define BRAKING_PWM_PIN 4
#define POTENTIOMETER_PIN_BRAKING A3 // 17

//Set up our SBUS objects:
bfs::SbusRx sbus_rx(&Serial2); //For reading SBUS from Serial 2 RX pin
bfs::SbusTx sbus_tx(&Serial2); //For writing SBUS to pins, if needed (shouldn't be)
bfs::SbusData data; //For storing data received from SBUS
/* SBUSDATA struct data members:
bool lost_frame Whether a frame has been lost.
bool failsafe Whether the receiver has entered failsafe mode or to command servos to enter failsafe mode.
bool ch17, ch18 State of channel 17 and channel 18.
static constexpr int8_t NUM_CH = 16 The number of SBUS channels.
int16_t ch[NUM_CH] An array of SBUS channel data.
*/
//------------------------------Global Variables----------------------------------//
bool running = true; // Comment this out to ignore SBUS and packet reading functionality
unsigned long last_packet_sent = 0; // For SBUS timing

#ifdef KART
  //For logging:
  File dataFile;

  //For steering and braking feedback control systems:
  float total_P = 0;
  float last_P = 0;
  float total_P_brake = 0;
  float last_P_brake = 0;

  // Steering H-Bridge pins. These two pins will be set for right or left (one must be off at a time)
  int drivingPin = STEERING_PIN_RIGHT;
  int offPin = STEERING_PIN_LEFT;

  //Startup positions for actuators. 90 for wheels straight on kart. 0 for throttle
  int target_angle = 90;
  int target_throttle = 0; // -100 FULL BRAKE | 0 NO BRAKE || 255 FULL THROTTLE
  int previous_throttle = 0;
  int throttle_brake_switch = 0; // 0 = BRAKE | 1 = THROTTLE

  //Misc. globals 
  float voltage_actual = 0.0f; // For feedback control systems
  unsigned long last_time_steering = 0; //For timing in core loop (STEERING CONTROL FUNCTION)
  unsigned long last_time_throttle_braking = 0; //For timing in core loop (THROTTLE/BRAKE FUNCTION)
  unsigned long last_test_iteration = 0; //For timing in core loop (TESTING)
  unsigned long last_cycle_time = 0; //For timing in core loop (WATCHDOG)
  bool test_state = 1; // For testing purposes
  int brake_mode = 0; // Initial brake/throttle mode (0 is brake)
  int WATCHDOG = 1; // For Jetson-Teensy WATCHDOG functionality. Start petted (Pat?).
#endif

//-----------------------------Function Prototypes--------------------------------//
void parsePacket(char* cmd, int* cmd_values);
bool verifyCheckSum(String cmd, int check_sum);
int createCheckSum(String packet);
float fmap(float, float, float, float, float);
int SteeringPID(int, int, float*, float*);
int BrakingPD(int,int,float*);
void steering_control(int, float*);
void throttle_brake_control(int, int);
void brake_test(int);
void steer_test(int);
void log_error(const char*, File*);
void pet_watchdog();
void kill_kart_dead();
void throttle_control(int);
//-----------------------------------Setup---------------------------------------//
void setup() {
  Serial1.begin(57600);
  Serial.begin(57600);
  Serial.setTimeout(50);
  
  #ifdef KART
    pinMode(STEERING_PIN_LEFT, OUTPUT);
    pinMode(STEERING_PIN_RIGHT, OUTPUT);

    pinMode(BRAKING_RETRACT_PIN, OUTPUT);
    pinMode(BRAKING_EXTEND_PIN, OUTPUT);

    analogReadResolution(12);      // Increase resolution to 12-bit for smoother PID
    analogReadAveraging(16);       // Hardware-average 32 samples per read to kill noise

  #endif

  #ifdef LOGGER
    if (!SD.begin(BUILTIN_SDCARD)) {
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);
      
      return;
    }
    dataFile = SD.open("errorlog.txt", FILE_WRITE);
    log_error("File opened.", &dataFile);
  #endif

  #ifdef PROTOTYPE
    //Send neutral (1500 microseconds) on startup for safety
    analogWriteFrequency(STEERING_PIN, 50); //Set steering pin PWM frequency to 50 Hz 
    analogWriteFrequency(THROTTLE_PIN, 50); //Set throttle pin PWM frequency to 50 Hz
    analogWriteResolution(16);
    pinMode(STEERING_PIN, OUTPUT);
    pinMode(THROTTLE_PIN, OUTPUT);
    int neutral = (1500 * 65536L) / 20000;
    analogWrite(STEERING_PIN, neutral);
    analogWrite(THROTTLE_PIN, neutral);
  #endif

  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  delay(1000);
}

//----------------------------------Core Loop------------------------------------//
void loop() {

  if(running){
    #ifdef JETSON_WATCHDOG
      unsigned long current_cycle_time = millis();
      if(current_cycle_time - last_cycle_time > MAX_JETSON_TIME_LAPSE && last_cycle_time != 0){
        WATCHDOG = 0; // Reset watchdog
      }
      if(!WATCHDOG){
        //Serial.println("WATCHDOG expired! Doggy MAD!!");
        log_error("WATCHDOG expired! Doggy MAD!!", &dataFile);
        kill_kart_dead();
      }
    #endif
    //-------------------------SBUS READ, PARSE & WRITE----------------------------//
    //Check for SBUS inputs and send to Jetson Nano over UART (Serial 1)
    if(USING_CONTROLLER && millis() - last_packet_sent >= 5){
      if(sbus_rx.Read()){
        data = sbus_rx.data(); //Store our received SBUS data
        if(!data.failsafe && !data.lost_frame){
          //Data access: data.ch[i], where i is the channel we are expecting data from
          //data.ch[index], index is expected to be int8_t 
          int mode = 0; // "1" == RECORDING, "0" == NOT RECORDING
          float user_steering = float(data.ch[SBUS_CHANNEL_1]); //contains an 11 bit integer value
          float user_throttle = float(data.ch[SBUS_CHANNEL_3]); //contains an 11 bit integer value  
          int kill_switch = data.ch[SBUS_CHANNEL_5]; 
          if(kill_switch == 200){
            log_error("Kill switch triggered. Exiting.", &dataFile);
            analogWrite(STEERING_PIN, (1500 * 65536L) / 20000);
            analogWrite(THROTTLE_PIN, (1500 * 65536L) / 20000);
            running = false;
          }

          //Convert 11 bit channel values into +/- 1 for sending to Jetson
          user_steering = fmap(user_steering, 200, 1800, -1.0f, 1.0f);
          user_throttle = fmap(user_throttle, 200, 1800, -1.0f, 1.0f);

          //Assemble packet for transmission to Jetson:
          //For toggling record mode
          if(data.ch[SBUS_CHANNEL_7] == 1800){
            mode = 1;
          }
          //For deleting the last 10 seconds of recorded data
          if(data.ch[SBUS_CHANNEL_6] == 1800){
            mode = 2;
          }

          String out_packet = String(user_steering) + "," + String(user_throttle) + "," + String(mode);
          int check_sum_out = createCheckSum(out_packet);
          out_packet = "<" + String(user_steering) + "," + String(user_throttle) + "," + String(mode) + "|" + String(check_sum_out) + ">\n";

          //Send command packet to Jetson:
          /* <steering,throttle,record_mode|checksum>\n */
          Serial.write(out_packet.c_str(), out_packet.length());
          last_packet_sent = millis();
        }
      }
    }

    //-----------------------USB JETSON COMMANDS READ AND PARSE-------------------//
    //Read USB input from jetson for instructions from BuffMata
    if(Serial.available() > 0){

      //Grab our packet
      String packet = Serial.readStringUntil('\n');
      
      //Do your duty. Pet the watchdog!
      pet_watchdog();

      //Remove delimiters:
      packet.trim();
      if(packet.startsWith("<") && packet.endsWith(">")){
          packet = packet.substring(1, packet.length() - 1); // Grab packet contents minus the start/end frame characters
      }
      else{
        log_error("ERROR: Invalid packet format received", &dataFile);
        return;
      }
        
      //Split into command and checksum
      int cmd_end = packet.indexOf("|");
      if(cmd_end == -1){
        log_error("ERROR: No checksum operator detected", &dataFile);
        return;
      }
      String cmd = packet.substring(0, cmd_end);
      int check_sum = packet.substring(cmd_end + 1).toInt();

      //Verify checksum
      if(verifyCheckSum(cmd, check_sum)){
        //Split command 
        int command_values[4];
        char cmd_buffer[cmd.length() + 1]; //For C-style string processing
        cmd.toCharArray(cmd_buffer, cmd.length() + 1); //For C-style string processing
        parsePacket(cmd_buffer, command_values);
        uint8_t steering_pin = static_cast<uint8_t>(command_values[0]);
        int steering_pwm = command_values[1];
        uint8_t throttle_pin = static_cast<uint8_t>(command_values[2]); // This is gonna be kart braking(0) and throttle (1)
        int throttle_pwm = command_values[3];
        #ifdef PROTOTYPE
          // Convert servo angles (0-180) to microseconds (1000-2000µs)
          int steering_us = map(steering_pwm, 0, 180, 1000, 2000);
          int throttle_us = map(throttle_pwm, 0, 180, 1000, 2000);
          
          // Convert microseconds to 16-bit PWM value for 50Hz
          // At 50Hz: period = 20,000µs, scale factor = 65536/20000 = 3.2768
          steering_pwm = (steering_us * 65536L) / 20000;
          throttle_pwm = (throttle_us * 65536L) / 20000;
          analogWrite(steering_pin, steering_pwm); //uint8_t, int
          analogWrite(throttle_pin, throttle_pwm); //uint8_t, int
        #endif

        //Update kart steering and throttle target values
        #ifdef KART
          target_angle = steering_pwm;
          target_throttle = throttle_pwm;
          throttle_brake_switch = throttle_pin;
        #endif
      }
    }
    //------------------------Update Steering/Throttle/Brake Values---------------------------//
    #ifdef KART
      if(millis() - last_time_steering > 5){
        steering_control(target_angle, &voltage_actual);
        last_time_steering = millis();
      }
      
      if(millis() - last_time_throttle_braking > 50){
        throttle_brake_control(throttle_brake_switch, target_throttle);
        last_time_throttle_braking = millis();
      }
    #endif
  
  }
}
//-----------------------------Actuator Control Functions--------------------------------//
void steering_control(int steering_pwm, float* voltage_actual){
    int target_angle = fmap(steering_pwm,45,135,90-WHEELS_DEGREES_OF_FREEDOM,90+WHEELS_DEGREES_OF_FREEDOM);

    int pot_value_steering = analogRead(POTENTIOMETER_PIN_STEERING); //Don't tell the DEA about our pot
    *voltage_actual = 3.3 * (pot_value_steering / 4095.0f);
    int actual_angle = 90;

    static float filtered_voltage = -1.0f;
    if (filtered_voltage < 0) {
      filtered_voltage = *voltage_actual;
    }   // This is for startup
    else {
      filtered_voltage = 0.4f * filtered_voltage + 0.6f * (*voltage_actual);
      *voltage_actual = filtered_voltage;
    }


    if (*voltage_actual < POT_VOLTAGE_WHEELS_STRAIGHT) {
        actual_angle = 90 - ((POT_VOLTAGE_WHEELS_STRAIGHT - *voltage_actual) / 
                        (POT_VOLTAGE_WHEELS_STRAIGHT - POT_VOLTAGE_WHEELS_MIN)) * WHEELS_DEGREES_OF_FREEDOM;
    } 
    else if (*voltage_actual <= MAX_VOLTAGE) {
        actual_angle = 90 + 
        ((*voltage_actual - POT_VOLTAGE_WHEELS_STRAIGHT) / (POT_VOLTAGE_WHEELS_MAX - POT_VOLTAGE_WHEELS_STRAIGHT)) * WHEELS_DEGREES_OF_FREEDOM;
    } 
    else {
      log_error("WARNING: OVERVOLTAGE DETECTED", &dataFile);
      digitalWrite(drivingPin, LOW);
      digitalWrite(offPin, LOW);
      Serial1.println("!!!!!!!!!!");
      kill_kart_dead();
    }

    // Run PID for steering
    int pid_output = SteeringPID(target_angle, actual_angle, &total_P, &last_P);
    
    int direction = pid_output >= 0 ? 1 : 0;
    int magnitude = abs(pid_output);
    
    if (direction == 1){
      drivingPin = STEERING_PIN_RIGHT;
      offPin = STEERING_PIN_LEFT;
    }
    else{
      drivingPin = STEERING_PIN_LEFT;
      offPin = STEERING_PIN_RIGHT;
    }
  
  if (magnitude > 0 && magnitude < 20) magnitude = 20;
  if(pid_output == -1000){
      analogWrite(STEERINGPWMPIN, 0);
      digitalWrite(drivingPin, LOW);
      digitalWrite(offPin, LOW);
  }
  else{
      digitalWrite(drivingPin, HIGH);
      digitalWrite(offPin, LOW);
      analogWrite(STEERINGPWMPIN, magnitude);
  }       
}

void throttle_brake_control(int throttle_brake_switch, int throttle_pwm){
  float pot_value_braking = analogRead(POTENTIOMETER_PIN_BRAKING); //Don't tell the DEA about our pot
  float voltage_actual_braking = 3.3 * (pot_value_braking / 4095.0f);

  static float filtered_voltage_braking = -1.0f;
  if (filtered_voltage_braking < 0){
    filtered_voltage_braking = voltage_actual_braking;   // This is for startup
  }
  else {
    filtered_voltage_braking = 0.4f * filtered_voltage_braking + 0.6f * voltage_actual_braking;
    voltage_actual_braking = filtered_voltage_braking;
  }

  int braking = (voltage_actual_braking>brakingMidPotentiometer)? 1 : 0;
  
  if(throttle_brake_switch == 1 && !braking){
    analogWrite(BRAKING_PWM_PIN, 0);
    throttle_control(throttle_pwm);
  }
  else { // throttle_pin == 0 and we brake
    int current_position = (voltage_actual_braking / brakingMaxPotentiometer) * -100;
    int braking_pwm = BrakingPD(current_position, throttle_pwm, &last_P_brake);
    if(braking_pwm < 25){
      braking_pwm = 25;
    }

    float targetLActuator = fmap(throttle_pwm, 0, -100, brakingMinPotentiometer, brakingMaxPotentiometer);
    if ((targetLActuator>voltage_actual_braking-.03) && targetLActuator<voltage_actual_braking+.03){
      digitalWrite(BRAKING_RETRACT_PIN, LOW);
      digitalWrite(BRAKING_EXTEND_PIN, LOW);
      analogWrite(BRAKING_PWM_PIN, 0);
    }
    else if (targetLActuator > voltage_actual_braking){
      digitalWrite(BRAKING_RETRACT_PIN, LOW);
      digitalWrite(BRAKING_EXTEND_PIN, HIGH);
      analogWrite(BRAKING_PWM_PIN, braking_pwm);
    }
    else if (targetLActuator < voltage_actual_braking){
      digitalWrite(BRAKING_RETRACT_PIN, HIGH);
      digitalWrite(BRAKING_EXTEND_PIN, LOW);
      analogWrite(BRAKING_PWM_PIN, braking_pwm);
    }
  }
}

void throttle_control(int target_throttle){
  if ((target_throttle - previous_throttle) > -3 && (target_throttle - previous_throttle) < 3){
    analogWrite(THROTTLE_KART_PIN, target_throttle);
    previous_throttle = target_throttle;
  } 
  else if (target_throttle>previous_throttle && target_throttle < MAX_THROTTLE){
    analogWrite(THROTTLE_KART_PIN, previous_throttle+3);
    previous_throttle += 3;
  }
  else if (target_throttle<previous_throttle){
    analogWrite(THROTTLE_KART_PIN, previous_throttle+3);
    previous_throttle -= 3;
  }
  else{
    // We should never get here (add kill switch function){
      //killswitch_triggered();
    }
}

void kill_kart_dead(){
  // 1. Put throttle to 0
  // 2. Stop steering
  // 3. Slowly but firmly apply brakes to bring kart to a controlled stop
  log_error("I killed myself.", &dataFile);
  
  // 1.
  analogWrite(THROTTLE_KART_PIN, 0); //uint8_t, int
  // 2.
  digitalWrite(STEERING_PIN_RIGHT, LOW);
  digitalWrite(STEERING_PIN_LEFT, LOW);
  // 3. 
  digitalWrite(BRAKING_RETRACT_PIN, LOW);
  digitalWrite(BRAKING_EXTEND_PIN, HIGH);
  int brake_pwm = 50;
  float brake_pot_voltage = 0.0f;
  while( brake_pot_voltage <= 2 && brake_pwm <= 255){
    brake_pot_voltage = (analogRead(POTENTIOMETER_PIN_BRAKING) * 3.3 / 4095.0f);
    analogWrite(BRAKING_PWM_PIN, brake_pwm);
    brake_pwm += 50;
    delay(500);
  }
  running = false;
  dataFile.close();
}
//-----------------------------Feedback Control Functions--------------------------------//

int SteeringPID(int target_angle, int actual_angle, float* total_P, float* last_P){
  //Update time
  static unsigned long last_time = 0;
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  if(dt > 0.1f) dt = 0.1f;  // cap at 100ms
  last_time = now;
  //Misc Variables
  int steering_output = 0;
  int unclamped_steering_output = 0;
  static float kP = KP_VAL; //Proportional gain
  static float kI = KI_VAL; //Integral gain
  static float kD = KD_VAL; //Derivative gains
  static float P, I, D;

  P = target_angle - actual_angle;

  if(P <= 3 && P >= -3){
    return -1000;
  }
  
  if(*total_P == 0 && *last_P == 0){
    dt = 0.1;
    I = 0;
    D = 0;
  } 
  else{
    I = *total_P + (*last_P + P)/2 * dt;
    D = (P - *last_P)/dt;
  }

  *last_P = P; 
  unclamped_steering_output = P*kP + I*kI + D*kD;
  //Combat integral windup
  if(unclamped_steering_output >= -255 && unclamped_steering_output <= 255){
    *total_P = I;
  }
  //Clamping outputs
  steering_output = max(-255, min(unclamped_steering_output, 255));
  return steering_output;
}

int BrakingPD(int target_position, int actual_position, float* last_P_brake){
  //Update time
  static unsigned long last_time = 0;
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  if(dt > 0.1f) dt = 0.1f;  // cap at 100ms
  last_time = now;
  //Misc Variables
  int braking_output = 0;
  int unclamped_braking_output = 0;
  float kP = KP_VAL_BRAKE; //Proportional gain
  float kD = KD_VAL_BRAKE; //Derivative gains
  float P, D;

  P = abs(target_position - actual_position);

  if(P <= 3){
    return -1000;
  }
  
  if(*last_P_brake == 0){
    dt = 0.1;
    D = 0;
  } 
  else{
    D = (P - *last_P_brake)/dt;
  }

  *last_P_brake = P; 
  unclamped_braking_output = P*kP + D*kD;
  braking_output = max(0, min(unclamped_braking_output, 255));
  return braking_output;
}

//-----------------------------Packet Assembly and Parsing Functions--------------------------------//
void parsePacket(char *cmd, int *command_values){
  int index = 0;
  char *delim = strtok(cmd, ",");
  while(delim != NULL && index < 4){
    command_values[index] = atoi(delim);
    delim = strtok(NULL, ","); // Start where last "," found
    index++;
  }
}

bool verifyCheckSum(String cmd, int received_check_sum){
  int check_sum = 0;
  for(int i = 0; i < static_cast<int>(cmd.length()); i++){
    check_sum ^= cmd[i];
  } 
  return check_sum == received_check_sum;
}

int createCheckSum(String packet){
  int check_sum = 0;
  for(int i = 0; i < static_cast<int>(packet.length()); i++){
    check_sum ^= packet[i];
  } 
  return check_sum;
}

float fmap(float input, float in_min, float in_max, float out_min, float out_max) {
  float mapped = (input - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
  return max(out_min, min(mapped, out_max));
}

//-----------------------------Test and Misc Functions--------------------------------//
void brake_test(int brake_mode){
  if(brake_mode == 0){
    digitalWrite(BRAKING_RETRACT_PIN, LOW);
    digitalWrite(BRAKING_EXTEND_PIN, HIGH);
    analogWrite(BRAKING_PWM_PIN, 255);
  }

  if(brake_mode == 1){
    digitalWrite(BRAKING_RETRACT_PIN, HIGH);
    digitalWrite(BRAKING_EXTEND_PIN, LOW);
    analogWrite(BRAKING_PWM_PIN, 255);
  }
  
}
void steer_test(int target_angle){
  steering_control(target_angle, &voltage_actual);
}

void log_error(const char* message, File* dataFile) {
  #ifdef LOGGER
    if (*dataFile) {
      dataFile->println(message);
      dataFile->flush();
    }
  #endif
}

void pet_watchdog(){
  last_cycle_time = millis();
  WATCHDOG = 1;
}

