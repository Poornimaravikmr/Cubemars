/*
 * Project: Dual Cubemars Motor Sinusoidal Position - Velocity Control
 *
 * Description:
 * - Controls two Cubemars AK series motors via CAN using a Teensy 4.1.
 * - Receives the desired amplitude and frequency for each motor through the Serial Monitor.
 * - Generates independent sinusoidal position trajectories for both motors.
 * - Calculates the corresponding velocity and acceleration profiles.
 * - Converts velocity and acceleration to ERPM before sending Position-Velocity commands.
 * - Reads real-time position feedback from both motors via CAN.
 * - Transmits the commanded positions, measured positions, velocities, and accelerations over Serial3 for monitoring and data logging.
 *
 * Serial Input Format:
 * Amplitude,Frequency_Motor1,Frequency_Motor2
 *
 * Example:
 * 20,0.5,1.0
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// Motor IDS
#define MOTOR_1 0x03
#define MOTOR_2 0x02

// Variables
float motor_pos_1 = 0.0; 
float motor_pos_2 = 0.0;
float amplitude, freq_1, freq_2;

unsigned long startTime;
String buffer = "";

// Motor Modes
enum AKMode {
  AK_PWM = 0,
  AK_CURRENT,
  AK_CURRENT_BRAKE,
  AK_VELOCITY,
  AK_POSITION,
  AK_ORIGIN,
  AK_POSITION_VELOCITY,
};


uint32_t canId(int id, AKMode Mode_set) {
  return uint32_t(id | (Mode_set << 8));
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void comm_can_transmit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  msg.flags.extended = 1;  // Extended frame

  for (uint8_t i = 0; i < len; i++)
    msg.buf[i] = data[i];

  Can0.write(msg);
}

// Position control
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t idx = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &idx);
  comm_can_transmit_eid(canId(controller_id, AK_POSITION), buffer, idx);
}

// Position - velocity control
void comm_can_set_pos_spd(uint8_t id, float pos, float spd, float RPA) {
  int32_t idx = 0;
  int16_t idx2 = 4;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &idx);
  buffer_append_int16(buffer, (int16_t)(spd), &idx2);
  buffer_append_int16(buffer, (int16_t)(RPA), &idx2);
  comm_can_transmit_eid(canId(id, AK_POSITION_VELOCITY), buffer, idx2);
}

// Set Origin
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  int32_t idx = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)set_origin_mode, &idx);
  comm_can_transmit_eid(canId(controller_id, AK_ORIGIN), buffer, idx);
}

// Motor Feedback
void motor_receive(CAN_message_t* msg, float* motor_pos, float* motor_spd,
                   float* motor_cur, int8_t* motor_temp, int8_t* motor_error,
                   unsigned long* rx_id) {
  *rx_id = msg->id;
  uint8_t* buf = msg->buf;

  int16_t pos_int = buf[0] << 8 | buf[1];
  int16_t spd_int = buf[2] << 8 | buf[3];
  int16_t cur_int = buf[4] << 8 | buf[5];
  *motor_pos = pos_int * 0.1f;
  *motor_spd = spd_int * 10.0f;
  *motor_cur = cur_int * 0.01f;
  *motor_temp = buf[6];
  *motor_error = buf[7];
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  Can0.begin();
  Can0.setBaudRate(1000000);  // 1 Mbps
  Serial.println("CAN init OK!");

  comm_can_set_origin(MOTOR_1, 0);
  comm_can_set_origin(MOTOR_2, 0);
  
}

void loop() {
  
  // Enter the desired amplitude, freq of motor 1 and motor2 --> Eg : Amplitude, freq1, freq2  (20, 0.5, 0.5)
  while (Serial.available()) {

  char c = Serial.read();

  if (c == '\n') {

    int p1 = buffer.indexOf(',');
    int p2 = buffer.indexOf(',', p1 + 1);

    if (p1 > 0 && p2 > 0) {

      amplitude = buffer.substring(0, p1).toFloat();
      freq_1    = buffer.substring(p1 + 1, p2).toFloat();
      freq_2    = buffer.substring(p2 + 1).toFloat();

      startTime = millis();

    }

    buffer = "";
  }
  else {
    buffer += c;
  }
}
  float t = (millis() - startTime) / 1000.0;
  float pos_1 = amplitude * sin(2.0 * PI * freq_1 * t);
  float vel_1 = amplitude * 2.0 * PI * freq_1 * cos(2.0 * PI * freq_1 * t); 
  float acc_1 = -amplitude * pow(2.0 * PI * freq_1, 2) * sin(2.0 * PI * freq_1 * t);

  float pos_2 = amplitude * sin(2.0 * PI * freq_2 * t);
  float vel_2 = amplitude * 2.0 * PI * freq_2 * cos(2.0 * PI * freq_2 * t); 
  float acc_2 = -amplitude * pow(2.0 * PI * freq_2, 2) * sin(2.0 * PI * freq_2 * t);

  float SPD_1 = vel_1 * 31.5;
  float SPD_2 = vel_2 * 31.5;
  float APD_1 = acc_1 * 31.5; // convert deg/s to ERPM 
  float APD_2 = acc_2 * 31.5;
 
  // Send commands  (MOTOR_ID, POS, SPEED, ACCELERATION)
  comm_can_set_pos_spd(MOTOR_1, pos_1,SPD_1,APD_1);
  comm_can_set_pos_spd(MOTOR_2, pos_2,SPD_2,APD_2);

  // comm_can_set_pos_spd(MOTOR_1, pos_1,vel_1,acc_1);
  // comm_can_set_pos_spd(MOTOR_2, pos_2,vel_2,acc_2);
  

  
  CAN_message_t msg;
  while (Can0.read(msg)) {
      float motor_pos, motor_spd, motor_cur;
      int8_t motor_temp, motor_error;
      uint32_t rx_id;

      motor_receive(&msg, &motor_pos, &motor_spd, &motor_cur,
                    &motor_temp, &motor_error, &rx_id);

      uint8_t id = rx_id & 0xFF;

      if (id == MOTOR_1) {
          motor_pos_1 = motor_pos;
      }
      else if (id == MOTOR_2) {
          motor_pos_2 = motor_pos;
      }
  } 

  Serial3.print(pos_1);
  Serial3.print(",");
  Serial3.print(pos_2); 
  Serial3.print(","); 
  Serial3.print(motor_pos_1);
  Serial3.print(",");
  Serial3.print(motor_pos_2);
  Serial3.print(",");
  Serial3.print(vel_1);
  Serial3.print(",");
  Serial3.print(vel_2); 
  Serial3.print(","); 
  Serial3.print(acc_1);
  Serial3.print(",");
  Serial3.println(acc_2);
 
}
