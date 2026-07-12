/*
 * Project: Dual Cubemars Motor Sinusoidal Position Control 
 *
 * Description:
 * - Controls two Cubemars AK series motors via CAN using a Teensy 4.1.
 * - Generates sinusoidal position trajectories with a fixed amplitude and frequency.
 * - Sends Position-Velocity commands to both motors.
 * - Reads real-time motor position feedback via CAN.
 * - Transmits the commanded frequency, commanded positions, and measured motor positions
 *   over Serial3 for monitoring and data logging.
 */
 
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

#define MOTOR_1 0x03
#define MOTOR_2 0x02

float amplitude= 50;
float freq = 0.2;
float pos_1, pos_2;
unsigned long startTime;
float motor_pos_1 = 0.0;
float motor_pos_2 = 0.0;

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
// Position Control
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t idx = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &idx);
  comm_can_transmit_eid(canId(controller_id, AK_POSITION), buffer, idx);
}

// Position - Velocity Control
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
  startTime = millis();
}

void loop() {

  float t = (millis() - startTime) / 1000.0;
  pos_1 = amplitude * sin(2.0 * PI * freq * t);
  pos_2 = amplitude * sin(2.0 * PI * freq * t + (PI / 2));

  comm_can_set_pos(MOTOR_1, pos_1);
  comm_can_set_pos(MOTOR_2, pos_2);

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
  

  Serial3.print(freq);
  Serial3.print(",");
  Serial3.print(pos_1);
  Serial3.print(",");
  Serial3.print(pos_2); 
  Serial3.print(","); 
  Serial3.print(motor_pos_1);
  Serial3.print(",");
  Serial3.println(motor_pos_2);
 
}
