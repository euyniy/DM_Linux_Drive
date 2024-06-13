#include "damiao.h"
#include <cmath>
#include <cassert>

auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
auto dm = std::make_shared<damiao::Motor>(serial);

// Reference torque
// T_ref = kp*(p_des - theta_m) + kd*(v_des - d_theta_m) + t_ff
// T_ref = kp*(q - state.q) + kd*(dq - state.dq) + tau
// iqref = T_ref / KT_OUT

/* When kp=0 and kd≠0, setting v_des will achieve constant speed rotation.
    There is a steady-state error during constant speed rotation. 
    Additionally, kd should not be too large as it can cause oscillations.
    assert kd not too big */
void velocity_control(int id, float kd, float dq)
{

  dm->control(id, 0, kd, 0, dq, 0);
}


/* When kp=0 and kd=0, a specified torque output can be achieved by setting t_ff. 
  In this case, the motor will continuously output a constant torque.
  However, when the motor is idling or under a light load, if t_ff is set too high,
  the motor will continue to accelerate until it reaches its maximum speed, 
  and at this point, it still will not achieve the target torque t_ff. */
void torque_control(int id, float tau)
{
  dm->control(id, 0, 0, 0, 0, tau);
}

// kd!=0, kp!=0 
void position_control(int id, float kp, float kd, float q)
{
  dm->control(id, kp, kd, q, 0, 0);
}

void print_motor_state(int id)
{
  auto & m = dm->motors[id];
  std::cout << "m" + std::to_string(id) + ": "<< m->state.q << " " << m->state.dq << " " << m->state.tau << std::endl;
  std::cout << "temp(mos): " << m->state.t_mos << " temp(rot)" << m->state.t_rotor << std::endl;
}

int main(int argc , char** argv)
{

  dm->addMotor(0x01, 0x11);
  dm->addMotor(0x02, 0x12);

  dm->enable(0x01);
  std::cout << "Motor 1 position: " << dm->motors[0x01]->state.q << std::endl;
  dm->enable(0x02);
  std::cout << "Motor 2 position: " << dm->motors[0x02]->state.q << std::endl;
 
  //        id, float kp, float kd, float q, float dq, float tau
  //            ID,   kp,       kd,     pos,      vel,    torque
  // dm->control(0x01, 0,        0,       0,        0,         0); 

  // velocity (dq) control (kp=0, kd!=0 (not too big))
  velocity_control(0x01, 1.0, 0.5); // kd, dq

  // torque (tau) control (kp=0, kd=0)
  torque_control(0x02, 0.1); // tau

  // position control (kp!=0, kd!=0)
  // float current_q = dm->motors[0x01]->state.q;
  // position_control(0x01, 1.0, 1.0, current_q + 1.57); // kp, kd, q

  print_motor_state(0x01);
  print_motor_state(0x02);
  return 0;
}