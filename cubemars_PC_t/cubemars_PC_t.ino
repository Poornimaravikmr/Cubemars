/*
 * Project : Dual Cubemars Motor Position Control Based on Time 
 *
 * Description:
 * - Controls two Cubemars AK series motors via CAN using a Teensy 4.1.
 * - Receives a target position and movement time from the serial monitor.
 * - Generates a linear trajectory from the current position to the target position.
 * - Sends synchronized position commands to both motors.
 * - Reads real-time position feedback from each motor via CAN.
 * - Transmits motor positions over Serial3 for monitoring or communication with a master controller.
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0; 

// Motor ID
#define MOTOR_1 0x03
#define MOTOR_2 0x02

// Motor Variables
float motor_pos_1 = 0;
float motor_pos_2 = 0;

float target_pos = 0;
float move_time = 1;

float cmd_pos = 0;
float start_pos = 0;

bool moving = false;
uint32_t start_time = 0;

String buffer = "";

//Motor modes
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
  uint32_t mode = Mode_set;
  return uint32_t(id | (mode << 8));
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

// CAN communication
void comm_can_transmit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = 1;
  msg.len = len;
  memcpy(msg.buf, data, len);
  Can0.write(msg);
}

//Set Origin 
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)set_origin_mode, &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_ORIGIN), buffer, send_index);
}

// Position Control
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION), buffer, send_index);
}

// Position Velocity Control
void comm_can_set_pos_spd(uint8_t controller_id, float pos, float spd, float RPA) {
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer, (int16_t)(spd), &send_index1);
  buffer_append_int16(buffer, (int16_t)(RPA), &send_index1);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION_VELOCITY), buffer, send_index1);
}

// Motor Feedback 
void motor_receive(CAN_message_t* msg,
                   float* motor_pos, float* motor_spd, float* motor_cur,
                   int8_t* motor_temp, int8_t* motor_error, uint32_t* rx_id) {

  *rx_id = msg->id;
  uint8_t* buf = msg->buf;

  int16_t pos_int = (buf[0] << 8) | buf[1];
  int16_t spd_int = (buf[2] << 8) | buf[3];
  int16_t cur_int = (buf[4] << 8) | buf[5];

  *motor_pos = pos_int * 0.1f;
  *motor_spd = spd_int * 10.0f;
  *motor_cur = cur_int * 0.01f;
  *motor_temp = buf[6];
  *motor_error = buf[7];
}


void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  while (!Serial) {}

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps

  Serial.println("CAN init OK!");
  comm_can_set_origin(MOTOR_1,0);
  comm_can_set_origin(MOTOR_2,0);

}

void loop() {
  while(Serial.available()){
    char c = Serial.read();
    // ENTER: POS (deg/s), TIME(s) ; EG: 40,60
    if (c == '\n') {
      int p1 = buffer.indexOf(',');
      
      if (p1 > 0) {
        target_pos = buffer.substring(0, p1).toFloat();
        move_time  = buffer.substring(p1 + 1).toFloat();

        start_pos = motor_pos_2;      // current position
        cmd_pos   = start_pos;

        moving = true;
        start_time = millis();
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }
   
   if (moving) {

    float elapsed = (millis() - start_time) / 1000.0;

    if (elapsed >= move_time) {
        cmd_pos = target_pos;
        moving = false;
    }
    else {
        float alpha = elapsed / move_time;

        cmd_pos = start_pos +
                 (target_pos - start_pos) * alpha;
    }

    comm_can_set_pos(MOTOR_1, cmd_pos);
    comm_can_set_pos(MOTOR_2, cmd_pos);
}
  
  // Read CAN Feedback
  CAN_message_t msg;
  while (Can0.read(msg)) {
    float motor_pos, motor_spd, motor_cur;
    int8_t motor_temp, motor_error;
    uint32_t rx_id;

    motor_receive(&msg, &motor_pos, &motor_spd, &motor_cur,
                  &motor_temp, &motor_error, &rx_id);

    if ((rx_id & 0xFF) == MOTOR_1) motor_pos_1 = motor_pos;
    if ((rx_id & 0xFF) == MOTOR_2) motor_pos_2 = motor_pos;
  }
  
  
  // Send data to Master teensy
  Serial3.print(motor_pos_1);
  Serial3.print(',');
  Serial3.println(motor_pos_2);
  
}
