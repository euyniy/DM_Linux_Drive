#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <vector>
#include <unordered_map>

namespace damiao
{

#pragma pack(1)

typedef struct
{
  uint8_t frameheader;
  uint8_t CMD;// Command 0x00: Heartbeat
              //         0x01: Receive fail 0x11: Receive success
              //         0x02: Send fail 0x12: Send success
              //         0x03: Set baudrate fail 0x13: Set baudrate success
              //         0xEE: Communication error, error code in the data field
              //               8: Overvoltage 9: Undervoltage A: Overcurrent B: MOS overtemperature C: Motor coil overtemperature D: Communication loss E: Overload
  uint8_t canDataLen: 6; // Data length
  uint8_t canIde: 1; // 0: Standard frame 1: Extended frame
  uint8_t canRtr: 1; // 0: Data frame 1: Remote frame
  uint32_t CANID; // Motor feedback ID
  uint8_t canData[8];
  uint8_t FrameEnd; // Frame end
} CAN_Recv_Frame;

typedef struct 
{
  uint8_t frameHeader[2] = {0x55, 0xAA}; // Frame header
  uint8_t frameLen = 0x1e; // Frame length
  uint8_t CMD = 0x01; // Command 1: Forward CAN data frame 2: PC and device handshake, device feedback OK 3: Non-feedback CAN forwarding, do not feedback send status
  uint32_t sendTimes = 1; // Number of sends
  uint32_t timeInterval = 10; // Time interval
  uint8_t IDType = 0; // ID type 0: Standard frame 1: Extended frame
  uint32_t CANID; // CAN ID, use motor ID as CAN ID
  uint8_t frameType = 0; // Frame type 0: Data frame 1: Remote frame
  uint8_t len = 0x08; // Length
  uint8_t idAcc;
  uint8_t dataAcc;
  uint8_t data[8];
  uint8_t crc; // Unparsed, any value

  void modify(const id_t id, const uint8_t* send_data)
  {
    CANID = id;
    std::copy(send_data, send_data+8, data);
  }
} CAN_Send_Frame;

#pragma pack()
typedef struct 
{
  struct{
    float Q_MIN = -12.5;
    float Q_MAX = 12.5;
    float DQ_MAX;
    float TAU_MAX;
  } params;
  struct {
    float kp;
    float kd;
    float q;
    float dq;
    float tau;
  } cmd;
  struct {
    float q;
    float dq;
    float tau;
    float t_mos;
    float t_rotor;
  } state;
} MotorParam;


/**
 * @brief Damiao Technology DM-J4310-2EC and DM-J4340-2EC Motor Control
 * 
 * Communication using USB to CAN, virtual serial port on Linux
 */
class Motor
{
public:
  Motor(SerialPort::SharedPtr serial = nullptr)
  : serial_(serial)
  {
    if (serial_ == nullptr) {
      serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    }
  }

  void enable(id_t id) { control_cmd(id, 0xFC); }
  void disbale(id_t id) { control_cmd(id, 0xFD); }
  void zero_position(id_t id) { control_cmd(id, 0xFE); }
  void reset(id_t id) { control_cmd(id, 0xFB); }
  std::unordered_map<id_t, std::shared_ptr<MotorParam>> motors;

  /**
   * @brief Add motor
   * 
   * Implement different MOTOR_ID and MASTER_ID pointing to the same MotorParam
   * Make sure MOTOR_ID and MASTER_ID are not used
   */
  void addMotor(id_t MOTOR_ID, id_t MASTER_ID, uint MOTOR_TYPE)
  {
    motors.insert({MOTOR_ID, std::make_shared<MotorParam>()});
    if (MOTOR_TYPE == 4310){
      motors[MOTOR_ID]->params.DQ_MAX = 30;
      motors[MOTOR_ID]->params.TAU_MAX = 10;
    }
    else if (MOTOR_TYPE == 4340){
      motors[MOTOR_ID]->params.DQ_MAX = 10;
      motors[MOTOR_ID]->params.TAU_MAX = 28;
    }
    motors[MASTER_ID] = motors[MOTOR_ID];
  }

  void control(id_t id, float kp, float kd, float q, float dq, float tau)
  {
    // Position, velocity, and torque are converted from floating-point data to signed fixed-point data using linear mapping
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
      float span = xmax - xmin;
      float data_norm = (x - xmin) / span;
      uint16_t data_uint = data_norm * ((1 << bits) - 1);
      return data_uint;
    };

    if(motors.find(id) == motors.end())
    {
      throw std::runtime_error("Motor id not found");
    }
    auto& m = motors[id];
    m->cmd = {kp, kd, q, dq, tau}; // Save control command

    // according to the manual, having kd=0 while controlling position will cause oscillation
    // setting kd serves as a damping factor
    if ((kp != 0) && (kd == 0)) 
    {
      std::cout << "kd should not be 0 when controlling position!" << std::endl;
      return;
    }
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    uint16_t q_uint = float_to_uint(q, m->params.Q_MIN, m->params.Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -m->params.DQ_MAX, m->params.DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -m->params.TAU_MAX, m->params.TAU_MAX, 12);

    std::array<uint8_t, 8> data_buf;
    data_buf[0] = (q_uint >> 8) & 0xff;                             // p_des
    data_buf[1] = q_uint & 0xff;                                    // p_des
    data_buf[2] = dq_uint >> 4;                                     // v_des
    data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);  // v_des + kp
    data_buf[4] = kp_uint & 0xff;                                   // kp
    data_buf[5] = kd_uint >> 4;                                     // kd
    data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf); // kd + t_ff
    data_buf[7] = tau_uint & 0xff;                                  // t_ff
    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Frame));
    this->recv();
  }

  void recv()
  {
    serial_->recv((uint8_t*)&recv_data, 0xAA, sizeof(CAN_Recv_Frame)); 

    if(recv_data.CMD == 0x11 && recv_data.FrameEnd == 0x55) // Receive success
    { 
      static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = float(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        return data;
      };

      auto & data = recv_data.canData;
      uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
      uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
      uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
      uint16_t t_mos = (uint16_t(data[6]));
      uint16_t t_rotor = (uint16_t(data[7]));

      if(motors.find(recv_data.CANID) == motors.end())
      {
        std::cout << "Unknown motor id: " << std::hex << recv_data.CANID << std::endl;
        return;
      }
      auto & m = motors[recv_data.CANID];
      m->state.q = uint_to_float(q_uint, m->params.Q_MIN, m->params.Q_MAX, 16);
      m->state.dq = uint_to_float(dq_uint, -m->params.DQ_MAX, m->params.DQ_MAX, 12);
      m->state.tau = uint_to_float(tau_uint, -m->params.TAU_MAX, m->params.TAU_MAX, 12);
      m->state.t_mos = t_mos;
      m->state.t_rotor = t_rotor;
      return;
    } 
    else if (recv_data.CMD == 0x01) // Receive fail
    {
      std::cout << "Receive fail" << std::endl;
    } 
    else if (recv_data.CMD == 0x02) // Send fail
    {
      std::cout << "Send fail" << std::endl;
    } 
    else if (recv_data.CMD == 0x03) // Send success
    {
      std::cout << "Send success" << std::endl;
    }
    else if (recv_data.CMD == 0xEE) // Communication error
    {
      std::cout << "Communication error" << std::endl;
    }
  }

private:
  void control_cmd(id_t id , uint8_t cmd)
  {
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    send_data.modify(id, data_buf.data());
    serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Frame));
    usleep(1000);
    recv();
  }

  SerialPort::SharedPtr serial_;
  CAN_Send_Frame send_data;
  CAN_Recv_Frame recv_data;
};

}; // namespace damiao

#endif // DAMIAO_H