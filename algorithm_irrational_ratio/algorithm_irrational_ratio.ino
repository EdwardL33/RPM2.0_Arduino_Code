/*
  This file contains the motor control for the Random Positioning Machine 2.0
  ---------------------------------------------------------------------------
  Standard Operating Procedure:
  The default profile is MOTOR_SIMPLIFIED_RANDOM
  If you want to change the profile, just enter the number corresponding to the profile you desire into the serial monitor
    0 - MOTOR_BOTH_OFF         
    1 - MOTOR_SIMPLIFIED_RANDOM, 
    2 - MOTOR_2D_CLINOSTAT, 
    3 - MOTOR_3D_CLINOSTAT,
    4 - MOTOR_IRRATIONAL,
    5 - MOTOR_CYCLOIDAL,
    6 - MOTOR_BRW
  Enter a to start your selected profile
  Enter r to end the profile
  You can only switch profiles once you've ended your current profile
  For best practice please wait for the motors to settle before starting a profile
*/

/*
  Libraries:
  autowp-mcp2515 library v1.3.1 from Arduino Libarary
  PID by Brett Beauregard
  RF24 by TMRh20
*/

/*
  Current Issues:
  - Gap between desired RPM and RPM reading that the PID controller cant close (possibly due to inability of setCurrent to do fine-grained control, or weight imbalances)
  - Still some amount of noise on the angular velocity reading
*/

/*
  TODO:
  - Make new version of printMotor
  - Verify radio connection for encoder
  - Clean up variables for different algorithms
*/

#include "mcp2515.h"
#include <PID_v1.h>
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

#define MAX_VELO_RPM_MECHANICAL 5 // change this for max RPM
#define POLE_PAIRS 14
#define MAX_VELO_RPM MAX_VELO_RPM_MECHANICAL*POLE_PAIRS // desired electrical RPM to be sent (post-gearbox);
#define GEAR_RATIO 6

/* Simplified Bounded Random Walk Constants*/
#define ACCEL_RAD_S_S 0.2
#define DT_MS 300
#define ANGLE_OF_ATTACK 15
#define SEED 2132138

/* Cycloidal Constants*/
#define CYCLOIDAL_TIME_PER_ROTATION 600000L
double velo_angle = 0;

struct can_frame canMsg;
struct can_frame canMsg1;
#define CAN_CS_PIN 53
MCP2515 mcp2515(CAN_CS_PIN);

#define MAX_CURRENT_AMPS 4.0 // rated working amps

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

enum MotorProfile{
  MOTOR_BOTH_OFF, 
  MOTOR_SIMPLIFIED_RANDOM, 
  MOTOR_2D_CLINOSTAT, 
  MOTOR_3D_CLINOSTAT,
  MOTOR_IRRATIONAL,
  MOTOR_CYCLOIDAL,
  MOTOR_BRW
}; 

MotorProfile currentProfile = MOTOR_SIMPLIFIED_RANDOM;

enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD
} CAN_PACKET_ID;

void setMotorProfile(MotorProfile profile);
void printMotor(Motor m, char c = '?');
void setVelocity(uint8_t controller_id, float velocity_rpm);
void setCurrent(uint8_t controller_id, float current);
void setAngle(uint8_t controller_id, float velocity_rpm, float angle_deg, float RPA);

Motor motorInner;
Motor motorOuter;

Motor motors[2] = {motorInner, motorOuter}; // This is creating copies, should fix

char mode = ' ';

// for CAN recieve
uint32_t id;
uint8_t data[8];
uint8_t len;

/* Simplified Bounded Random Walk Variables*/
double heading = 0;
uint32_t elapsed_time_send = 0;
uint32_t elapsed_time_sBRW = 0;
uint32_t elapsed_time_print = 0;
uint32_t prev_time_sBRW = 0;
uint32_t prev_time_send = 0;
uint32_t prev_time_print = 0;
int changeDT_sBRW = 300;
int send_interval = 25;
int print_interval = 50;

/* Current Control PID variables */
double inner_velocity_desired = 0;
double inner_velocity_reading = 0;
double inner_pid_output = 0;
double outer_velocity_desired = 0;
double outer_velocity_reading = 0;
double outer_pid_output = 0;

// Specify the links and initial tuning parameters
double Kpouter=0.03, Kiouter=0.001, Kdouter=0;
double Kpinner=0.02, Kiinner=0.01, Kdinner=0;
PID outerPID(&outer_velocity_reading, &outer_pid_output, &outer_velocity_desired, Kpouter, Kiouter, Kdouter, DIRECT); // input, output, setpoint
PID innerPID(&inner_velocity_reading, &inner_pid_output, &inner_velocity_desired, Kpinner, Kiinner, Kdinner, DIRECT); // input, output, setpoint 

/* Low Pass Filter Variables for PID Input and Output */
const float alpha = 0.025; // Smoothing factor (adjust as needed)
double filteredVeloOuter = 0;
double filteredVeloInner = 0;
double filteredCurrOuter = 0;
double filteredCurrInner = 0;


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
  outerPID.SetOutputLimits(-MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
  outerPID.SetMode(AUTOMATIC);
  outerPID.SetSampleTime(25); // sets the frequency, in Milliseconds with which the PID calculation is performed
  
  innerPID.SetOutputLimits(-MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
  innerPID.SetMode(AUTOMATIC);
  innerPID.SetSampleTime(5); // sets the frequency, in Milliseconds with which the PID calculation is performed

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
  // if (!radio.begin()) {
  //    Serial.println(F("radio hardware is not responding!!"));
  //    while (1) {}  // hold in infinite loop
  // } else {
  //   Serial.println("radio good");
  // }
  // radio.setPALevel(RF24_PA_LOW);
  // radio.openReadingPipe(1, address);
  // radio.startListening();

  pinMode(LED_PIN, OUTPUT);

  randomSeed(SEED);
}




void loop() {

  digitalWrite(CSN_PIN, HIGH);     // make sure radio is idle
  // if CAN message was successfully recieved, pass data into the motor struct
  // if (comm_can_recieve_eid(&id, data, &len)) {
  //   getFeedback();
  // }

  // Drain the CAN buffer completely so we use the freshest data for PID
  while (comm_can_recieve_eid(&id, data, &len)) {
    getFeedback();
  }
  // // grab inner angle
  // if (radio.available()) {
  //   radio.read(&payload, sizeof(payload));
  //   Serial.print("Received: ");
  //   inner_angle_ticks = payload[0];
  //   Serial.print("inner: ");
  //   Serial.println(inner_angle_ticks);
  // }

  // // grab outer angle
  // uint16_t outer_angle_ticks = measureTicks();
  // if (outer_angle_ticks != 0xFFFF) {
  //   // Print confirmation to Serial Monitor for debugging
  //     Serial.print("outer: ");
  //     Serial.println(outer_angle_ticks);
  // }

  uint32_t current_time = millis();

  /* read in commands from serial monitor */
  if(Serial.available()){
    char inChar = Serial.read();
    // if we are in idle mode, then we can change algorithms
    if (mode == ' ') {
      switch (inChar) {
        case '0':
          setMotorProfile(MOTOR_BOTH_OFF);
          Serial.println("Both off");
          break;
        case '1':
          setMotorProfile(MOTOR_SIMPLIFIED_RANDOM);
          Serial.println("srand");
          break;
        case '2':
          setMotorProfile(MOTOR_2D_CLINOSTAT);
          Serial.println("2d");
          break;
        case '3':
          setMotorProfile(MOTOR_3D_CLINOSTAT);
          Serial.println("3d");
          break;
        case '4':
          setMotorProfile(MOTOR_IRRATIONAL);
          Serial.println("irra");
          break;
        case '5':
          setMotorProfile(MOTOR_CYCLOIDAL);
          Serial.println("cyc");
          break;
        case '6':
          setMotorProfile(MOTOR_BRW);
          break;
        // IGNORE NEWLINES so they don't trigger "default"
        case '\r': 
        case '\n':
          break;
        default :
          setMotorProfile(currentProfile);
          Serial.println("default");
          break;
      }
    }
    if(inChar == 'a'){
      mode = 'a';
      /* PID reset */
      outerPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      outerPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      outerPID.SetOutputLimits(-MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
      innerPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      innerPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      innerPID.SetOutputLimits(-MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
    }else if(inChar == 'r'){
      mode = ' ';
      setCurrent(0x64, 0);
      setCurrent(0x0A, 0);
      /* PID reset */
      outerPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      outerPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      outerPID.SetOutputLimits(-MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
      innerPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      innerPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      innerPID.SetOutputLimits(-MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
    }else if(inChar == 'd'){
      Serial.print("#");
      Serial.print(motors[0].position);
      Serial.print("=");
      Serial.print(motors[1].position);
      Serial.print("=");
      Serial.print(motors[0].speed);
      Serial.print("=");
      Serial.print(motors[1].speed);
      Serial.print("=");
      Serial.print(motors[0].temp);
      Serial.print("=");
      Serial.println(motors[1].temp);
    }
  }

  /* begin running the algorithm */
  if(mode == 'a'){
    if (currentProfile == MOTOR_SIMPLIFIED_RANDOM) {
      elapsed_time_sBRW = current_time - prev_time_sBRW;
      if(elapsed_time_sBRW >= changeDT_sBRW){
        prev_time_sBRW = current_time;
        heading += (random(65536)/65536.0) * ANGLE_OF_ATTACK - (ANGLE_OF_ATTACK/2);
        double heading_rad = heading * 3.14159 / 180;

        inner_velocity_desired = cos(heading_rad)*MAX_VELO_RPM*GEAR_RATIO; // eRPM pregearbox
        outer_velocity_desired = sin(heading_rad)*MAX_VELO_RPM*GEAR_RATIO;
        
        digitalWrite(LED_PIN, HIGH);
      }else{
        digitalWrite(LED_PIN, LOW);
      }
    }

    else if(currentProfile == MOTOR_2D_CLINOSTAT) {
      inner_velocity_desired = 0;                         // eRPM pregearbox
      outer_velocity_desired = MAX_VELO_RPM * GEAR_RATIO;
    }

    else if(currentProfile == MOTOR_3D_CLINOSTAT) {
      inner_velocity_desired = MAX_VELO_RPM * GEAR_RATIO; // eRPM pregearbox
      outer_velocity_desired = MAX_VELO_RPM * GEAR_RATIO;
    }

    else if(currentProfile == MOTOR_BOTH_OFF) {
      inner_velocity_desired = 0; // eRPM pregearbox
      outer_velocity_desired = 0;
    }

    else if(currentProfile == MOTOR_IRRATIONAL) {
      double heading_rad = atan2(3.14159265358979,exp(1));
      inner_velocity_desired = cos(heading_rad)*MAX_VELO_RPM*GEAR_RATIO; // eRPM pregearbox
      outer_velocity_desired = sin(heading_rad)*MAX_VELO_RPM*GEAR_RATIO;
    } 
    
    else if(currentProfile == MOTOR_CYCLOIDAL) {
      int32_t elapsed_time_ms = millis();
      int32_t curr_cycle_time = elapsed_time_ms % CYCLOIDAL_TIME_PER_ROTATION;
      velo_angle = 2 * PI * (double)(curr_cycle_time) / CYCLOIDAL_TIME_PER_ROTATION + PI;
      double velo_angle_rad = velo_angle * 3.14159 / 180;
      inner_velocity_desired = cos(velo_angle_rad)*MAX_VELO_RPM*GEAR_RATIO; // eRPM pregearbox
      outer_velocity_desired = sin(velo_angle_rad)*MAX_VELO_RPM*GEAR_RATIO;
    }
    
    else {
      inner_velocity_desired = 0; // eRPM pregearbox
      outer_velocity_desired = 0;
    }
    
    // sending is seperate from calculating speeds, for PID reasons
    elapsed_time_send = current_time - prev_time_send;
    if (elapsed_time_send >= send_interval) {

      prev_time_send = current_time;
      inner_velocity_reading = motors[0].speed * 10.0f; // eRPM pregearbox
      outer_velocity_reading = motors[1].speed * 10.0f;

      filteredVeloOuter = alpha * outer_velocity_reading + (1-alpha) * filteredVeloOuter;
      filteredVeloInner = alpha * inner_velocity_reading + (1-alpha) * filteredVeloInner;

      outer_velocity_reading = filteredVeloOuter;
      inner_velocity_reading = filteredVeloInner;

      outerPID.Compute(); 
      innerPID.Compute();
      //       // // --- PROFILER START ---
      // static unsigned long profileLastTime = 0;
      // static long profileCount = 0;

      // profileCount++;

      // // Update every 1000ms (1 second)
      // if (millis() - profileLastTime >= 1000) {
      //     // Calculate frequency
      //     float loopFreq = (float)profileCount; 
      //     // Calculate average period in microseconds
      //     float loopPeriod = 1000000.0 / loopFreq; 

      //     Serial.print("Loop Freq: ");
      //     Serial.print(loopFreq);
      //     Serial.print(" Hz  |  Avg Period: ");
      //     Serial.print(loopPeriod);
      //     Serial.println(" us");

      //     profileCount = 0;
      //     profileLastTime = millis();
      // }
      // // // --- PROFILER END ---

      filteredCurrOuter = alpha * outer_pid_output + (1-alpha) * filteredCurrOuter;
      filteredCurrInner = alpha * inner_pid_output + (1-alpha) * filteredCurrInner;

      /* Commented out filter on output to prevent lag on PID Controller*/
      // outer_pid_output = filteredCurrOuter;
      // inner_pid_output = filteredCurrInner;

      // setVelocity(0x64, inner_velocity_desired);
      // setVelocity(0x0A, outer_velocity_desired);
      setCurrent(0x0A, outer_pid_output); // using outerPID
      setCurrent(0x64, inner_pid_output); // using innerPID
    }
  }

  /* Limit frequency of prints */
  elapsed_time_print = current_time - prev_time_print;
  if (elapsed_time_print >= print_interval) {
    prev_time_print = current_time;
    float outer_angle = (float)((motors[1].position)*0.1f);
    float inner_angle = (float)((motors[0].position)*0.1f);

    /* print mechanical RPM */
    // Serial.print((motors[0].speed * 10.0f)/(14.0f * 6));
    // Serial.print(" ");
    // Serial.print((motors[1].speed * 10.0f)/(14.0f * 6));

    /* print all values */
    Serial.print(current_time);
    Serial.print(" ");
    Serial.print(motors[1].speed * 10.0f); // raw reading
    Serial.print(" ");
    Serial.print(outer_velocity_reading); // filtered reading
    Serial.print(" ");
    Serial.print(outer_velocity_desired);
    Serial.print(" ");
    Serial.print(outer_pid_output); // commanded current
    Serial.print(" ");
    Serial.print((motors[1].current * 0.01f));
    Serial.print(" | ");
    Serial.print(motors[0].speed * 10.0f); // raw reading
    Serial.print(" ");
    Serial.print(inner_velocity_reading); // filtered reading
    Serial.print(" ");
    Serial.print(inner_velocity_desired);
    Serial.print(" ");
    Serial.print(inner_pid_output); // commanded current
    Serial.print(" ");
    Serial.println((motors[0].current * 0.01f));

    /* print readings and setpoints */
    // Serial.print(outer_velocity_reading); // filtered reading
    // Serial.print(",");
    // Serial.print(outer_velocity_desired);
    // Serial.print(",");
    // Serial.print(inner_velocity_reading); // filtered reading
    // Serial.print(",");
    // Serial.println(inner_velocity_desired);


  }
  // delay(1);
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


// The speed value is of int32 type, and the range -100000 to 100000 represents -100000 to 100000 electrical RPM
void setVelocity(uint8_t controller_id, float velocity_rpm){
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)velocity_rpm, &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// The current value is of int32 type, and the values -60000 to 60000 represent -60 to 60 A
void setCurrent(uint8_t controller_id, float current) {
    // Clamp for safety (for initial tests)
    if (current > 4.0) {
      current = 4.0;
    }
    if (current < -4.0) {
      current = -4.0;
    }

    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void setAngleSingle(uint8_t controller_id, float velocity_rpm, float angle_deg, float RPA){
    int32_t send_index = 0;
    int32_t send_index1 = 4;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(angle_deg * 10000.0), &send_index);
    buffer_append_int16(buffer, velocity_rpm / 10.0, &send_index1);
    buffer_append_int16(buffer, RPA / 10.0, &send_index1);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}

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

// set a desired profile
void setMotorProfile(MotorProfile profile) {
  currentProfile = profile;
}

// For AS5600 PWM reading
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