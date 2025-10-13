/*
  CAN Send Example

  This will setup the CAN controller(MCP2515) to send CAN frames.
  Transmitted frames will be printed to the Serial port.
  Transmits a CAN standard frame every 2 seconds.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/

// radio stuffs
#include <SPI.h>
#include "RF24.h"
#define CE_PIN 7
#define CSN_PIN 8

uint16_t payload[1] = {0};
const byte address[] = "2Node";
uint16_t inner_angle_ticks;

#define LED_PIN 9 // Use this for your status indicator

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

int PWMpin = 5;  // connect AS5600 OUT pin here

#define MAX_VELO_RPM 15
#define GEAR_RATIO 6
#define ACCEL_RAD_S_S 0.2
#define DT_MS 300
#define ANGLE_OF_ATTACK 15
#define SEED 2132138

#define EN_CAN 0

#include "mcp2515.h"
struct can_frame canMsg;
struct can_frame canMsg1;
#define CAN_CS_PIN 53
MCP2515 mcp2515(CAN_CS_PIN);


#define TIMING_CYCLE 100
#define TIMING_TOLERANCE 10

#define SERIAL_TIMING 0
#define COMMAND_TIMING 20
#define QUERY_TIMING 40
#define PRINT_TIMING 80

#define DT_CYCLES DT_MS/TIMING_CYCLE

struct Motor {
  int16_t position;
  int16_t speed;
  int16_t current;
  uint8_t temp;
  uint8_t error_code;
  /**
    error codes:
    0: no fault
    1: motor over temperature
    2: over-current
    3: over-voltage
    4: under-voltage
    5: encoder fault
    6: MOSFET over-temp
    7: motor lock-up
  */
};



enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD
} CAN_PACKET_ID;

void printMotor(Motor m, char c = '?');
void setVelocity(uint8_t controller_id, float velocity_rpm);
void setAngle(uint8_t controller_id, float velocity_rpm, float angle_deg, float RPA);

Motor motorA;
Motor motorB;

Motor motors[2] = {motorA, motorB};

double heading = 0;

// String inst;

int commandLoop = 0;
int printLoopCount = 0;

const char endCharA = 'A';
const char endCharB = 'B';
const char purge = '_';

float valueA = 0;
float valueB = 0;

int pointCount = 0;

char mode = 'm';

bool serialRead = 0;
bool commandSent = 0;
bool querySent = 0;
bool imuRecv = 0;
bool printSent = 0;

// for CAN recieve
uint32_t id;
uint8_t data[8];
uint8_t len;

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
    canMsg1.can_id = CAN_EFF_FLAG + id;
    canMsg1.can_dlc = len;
    memcpy(canMsg1.data, data, len);
    mcp2515.sendMessage(&canMsg1);
}

// populate CAN frame with incoming message
bool comm_can_recieve_eid(uint32_t* inc_id, uint8_t* inc_data, uint8_t* inc_len) {
  struct can_frame canMsg;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    if (canMsg.can_id & CAN_EFF_FLAG) {
      *inc_id = canMsg.can_id & CAN_EFF_MASK; // strip extended flag
    } else {
      *inc_id = canMsg.can_id;
    }

    *inc_len = canMsg.can_dlc;
    memcpy(inc_data, canMsg.data, *inc_len);
    // for (int i = 0; i < 8; i++) {
    //   Serial.print("data[");
    //   Serial.print(i);
    //   Serial.print("] = 0x");
    //   if (data[i] < 0x10) Serial.print("0"); // Leading zero for single digit
    //   Serial.println(data[i], HEX);
    // }
    return true;
  }
  return false;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

//int16数据位整理
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

const uint8_t can_test[8] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
void setup() {
  Serial.begin(115200);

    // --- ADD THIS BLOCK ---
  // Manually manage CS pins to prevent conflict
  pinMode(CSN_PIN, OUTPUT);
  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);    // De-select the radio
  digitalWrite(CAN_CS_PIN, HIGH); // De-select the CAN controller
  // --- END BLOCK ---

  if (mcp2515.reset() == MCP2515::ERROR_OK) {
    Serial.print("CAN init ok!\r\n");
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    comm_can_transmit_eid(0x00, can_test, 8);
  } else Serial.print("CAN init fault!\r\n");

  digitalWrite(CAN_CS_PIN, HIGH); // De-select the CAN controller
  //radio 
  if (!radio.begin()) {
     Serial.println(F("radio hardware is not responding!!"));
     while (1) {}  // hold in infinite loop
  } else {
    Serial.println("radio good");
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, address);
  radio.startListening();

  pinMode(LED_PIN, OUTPUT);

  randomSeed(SEED);
}




void loop() {
  // grab inner angle
  if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    Serial.print("Received: ");
    inner_angle_ticks = payload[0];
    Serial.print("inner: ");
    Serial.println(inner_angle_ticks);
  }

  // grab outer angle
  uint16_t outer_angle_ticks = measureTicks();
  if (outer_angle_ticks != 0xFFFF) {
    // Print confirmation to Serial Monitor for debugging
      Serial.print("outer: ");
      Serial.println(outer_angle_ticks);
  }

  
  long time = millis();
  int cyclePosition = time%TIMING_CYCLE;

  if(cyclePosition > SERIAL_TIMING && cyclePosition <= SERIAL_TIMING + TIMING_TOLERANCE){
    if(Serial.available()){
      char inChar = Serial.read();
      // Serial.println("--");
      // Serial.println(inChar);
      // Serial.println((int)inChar);
      // Serial.println("--");
      if(inChar >= 48 && inChar <= 57){
        // inst += inChar;
      }else if(inChar == 'p'){
        mode = 'p';
        // setCurrent(0x141,0);
        // setCurrent(0x142,0);
      }else if(inChar == 'a'){
        mode = 'a';
        // setCurrent(0x141,0);
        // setCurrent(0x142,0);
      }else if(inChar == 'v'){
        mode = 'v';
        // setCurrent(0x141,0);
        // setCurrent(0x142,0);
      }else if(inChar == 'r'){
        mode = ' ';
        // setCurrent(0x141,0);
        // setCurrent(0x142,0);
      }else if(mode == 'p' && inChar == endCharA){
        // valueA = atof(inst.c_str());
        // setAngleSingle(0x141,MAX_VELO_RPM,(int16_t)(valueA*100));
        // inst = "";
      }else if(mode == 'p' && inChar == endCharB){
        // valueB = atof(inst.c_str());
        // setAngleSingle(0x142,MAX_VELO_RPM,(int16_t)(valueB*100));
        // inst = "";
      }else if(mode == 'v' && inChar == endCharA){
        // valueA = atof(inst.c_str());
        // setVelocity(0x141,(int32_t)(valueA*100));
        // inst = "";
      }else if(mode == 'v' && inChar == endCharB){
        // valueB = atof(inst.c_str());
        // setVelocity(0x142,(int32_t)(valueB*100));
        // inst = "";
      }else if(inChar == purge){
        // inst = "";
      }else if(inChar == 'd'){
        Serial.print("#");
        Serial.print(motors[0].position);
        Serial.print("=");
        Serial.print(motors[1].position);
        Serial.print("=");
        // Serial.print(motors[0].speed);
        // Serial.print("=");
        // Serial.print(motors[1].speed);
        // Serial.print("=");
        Serial.print(motors[0].temp);
        Serial.println("=");
        // Serial.print(motors[1].temp);
        // Serial.print("=");
      }
    }
  }else if(cyclePosition > COMMAND_TIMING && cyclePosition <= COMMAND_TIMING + TIMING_TOLERANCE){
    if(!commandSent){
      if(mode == 'a'){
        if(commandLoop > DT_CYCLES){
          heading += (random(65536)/65536.0) * ANGLE_OF_ATTACK - (ANGLE_OF_ATTACK/2);
          double heading_rad = atan2(3.14159265358979,exp(1));
          setVelocity(0x64,sin(heading_rad)*MAX_VELO_RPM*GEAR_RATIO);
          setVelocity(0x0A,cos(heading_rad)*MAX_VELO_RPM*GEAR_RATIO);
          // printMotor(motorA,'a');
          // printMotor(motorB,'b');
          commandLoop = 0;
          pointCount ++;
          digitalWrite(LED_PIN, HIGH);
        }else{
          digitalWrite(LED_PIN, LOW);
        }
        commandLoop ++;

      }
      
      commandSent = 1;
    }
  }else if(cyclePosition > QUERY_TIMING && cyclePosition <= QUERY_TIMING + TIMING_TOLERANCE){
    if(!querySent){
      querySent = 1;
    }
  }else if(cyclePosition > PRINT_TIMING && cyclePosition <= PRINT_TIMING + TIMING_TOLERANCE){
    if(!printSent){
      if(printLoopCount > 1){

        double heading_rad = heading * 3.14159 / 180;
        // Serial.print("Heading:");
        // Serial.print(heading_rad);
        // Serial.print(" A Des:[");
        // Serial.print(sin(heading_rad)*MAX_VELO_RPM);
        // Serial.print("] Act:[");
        // Serial.print(motors[0].velocity/10.0);
        // Serial.print("] B Des:");
        // Serial.print(cos(heading_rad)*MAX_VELO_RPM);
        // Serial.print("] Act:[");
        // Serial.print(motors[1].velocity/10.0);

        // Serial.print("[Immediate] [");
        // Serial.print(accel.acceleration.x + xOffset);
        // Serial.print("] [");
        // Serial.print(accel.acceleration.y + yOffset);
        // Serial.print("] [");
        // Serial.print(accel.acceleration.z + zOffset);

        // Serial.print("] [Integral G] X:");
        // Serial.print(accelX);
        // Serial.print(" Y:");
        // Serial.print(accelY);
        // Serial.print(" Z:");
        // Serial.print(accelZ);

        // Serial.print(" | PC:");
        // Serial.println(pointCount);

        printLoopCount = 0;
      }
      printLoopCount ++;
      printSent = 1;
    }
  }else{
    serialRead = 0;
    commandSent = 0;
    querySent = 0;
    printSent = 0;
  }
  // if CAN message was successfully recieved, pass data into the motor struct
  
  digitalWrite(CSN_PIN, HIGH);     // make sure radio is idle
  if (comm_can_recieve_eid(&id, data, &len)) {
    // Serial.print("ID received: ");
    // Serial.print(id, HEX);
    // Serial.print("    len received: ");
    // Serial.println(len);
    getFeedback();
    float outer_angle = (float)((motors[1].position)*0.1f);
    float inner_angle = (float)((motors[0].position)*0.1f);
    // Serial.print("#");
    // Serial.print(inner_angle);
    // Serial.print("=");
    // Serial.print(outer_angle);
    // Serial.println("=");
  }

  delay(1);
}


// void printMotor(Motor m, char c = '?'){
//   Serial.print("Motor ");
//   Serial.print(c);
//   Serial.print("| Position: ");
//   Serial.print(m.angle);
//   Serial.print("| Velocity");
//   Serial.print(m.velocity);
//   Serial.print("| Temp");
//   Serial.println(m.temp);
// }

void setVelocity(uint8_t controller_id, float velocity_rpm){
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)velocity_rpm, &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void setAngleSingle(uint8_t controller_id, float velocity_rpm, float angle_deg, float RPA){
    // uint8_t data[8] = { 
    //     0xA6, 
    //     0x00, 
    //     (uint8_t)(velocity_dps), 
    //     (uint8_t)(velocity_dps>>8), 
    //     (uint8_t)(angle_deg_hundreth), 
    //     (uint8_t)(angle_deg_hundreth>>8), 
    //     0x00, 
    //     0x00};

    // // uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
    // CANFrame frame(canID, data, sizeof(data));
    // // frame.print("TX");
    // CAN.write(frame);
    int32_t send_index = 0;
    int32_t send_index1 = 4;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(angle_deg * 10000.0), &send_index);
    buffer_append_int16(buffer, velocity_rpm / 10.0, &send_index1);
    buffer_append_int16(buffer, RPA / 10.0, &send_index1);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}

// void setCurrent(uint16_t canID, int16_t current_hundreth){
//     uint8_t data[8] = { 
//         0xA1, 
//         0x00, 
//         0x00, 
//         0x00, 
//         (uint8_t)(current_hundreth), 
//         (uint8_t)(current_hundreth>>8), 
//         0x00, 
//         0x00};

//     // uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
//     CANFrame frame(canID, data, sizeof(data));
//     // frame.print("TX");
//     CAN.write(frame);
// }

void getFeedback(){
  uint8_t motor_id = id & 0xFF; // lower 8 bits
  if (motor_id == 0x64 && len == 8) {
    // Serial.println("putting into motor 1");
    motors[0].position = (int16_t)(data[1] | (data[0] << 8));
    motors[0].speed = (int16_t)(data[3] | (data[2] << 8));
    motors[0].current = (int16_t)(data[5] | (data[4] << 8));
    motors[0].temp = data[6];
    motors[0].error_code = data[7];
  }

  else if (motor_id == 0x0A && len == 8) {
    // Serial.println("putting into motor 2");
    motors[1].position = (int16_t)(data[1] | (data[0] << 8));
    motors[1].speed = (int16_t)(data[3] | (data[2] << 8));
    motors[1].current = (int16_t)(data[5] | (data[4] << 8));
    motors[1].temp = data[6];
    motors[1].error_code = data[7];
  }
}


uint16_t measureTicks()
{
  // Measure HIGH and LOW pulse durations in microseconds
  unsigned long highTime = pulseIn(PWMpin, HIGH);
  unsigned long lowTime  = pulseIn(PWMpin, LOW);

  unsigned long period = highTime + lowTime;
  if (period == 0) return 0xFFFF; // no signal detected, return max value as error

  // Duty cycle (0-1)
  float duty = (float)highTime / period;

  // AS5600 PWM duty range: 2.9% - 97.1%
  const float DUTY_MIN = 0.029;
  const float DUTY_MAX = 0.971;

  // Clamp duty to valid range
  if (duty < DUTY_MIN) duty = DUTY_MIN;
  if (duty > DUTY_MAX) duty = DUTY_MAX;

  // Map to ticks (0-4095)
  uint16_t ticks = round((duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 4095.0);

  return ticks;
}





















// #include <SPI.h>
// #include "RF24.h"

// RF24 radio(7, 8); // CE, CSN
// const byte address[] = "2Node";

// void setup() {
//   Serial.begin(115200);
//   if (!radio.begin()) {
//      Serial.println(F("radio hardware is not responding!!"));
//      while (1) {}  // hold in infinite loop
//   }
//   radio.setPALevel(RF24_PA_LOW);
//   radio.openReadingPipe(1, address);
//   radio.startListening();
//   Serial.println("--- Simple Receiver Started ---");
//   radio.printDetails(); // Check the output of this!
// }

// void loop() {
//   if (radio.available()) {
//     uint16_t message = 0;
//     radio.read(&message, sizeof(message));
//     Serial.print("Received: ");
//     Serial.println(message);
//   }
// }