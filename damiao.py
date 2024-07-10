import numpy as np
import time
import serial

KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0

class Motor:
    def __init__(self, motor_id, motor_type):
        self.motor_id = motor_id
        if motor_type == "4310":
            self.params = {
                'Q_MIN': -12.5,
                'Q_MAX': 12.5,
                'DQ_MAX': 30.0, 
                'TAU_MAX': 10.0
            }
        elif motor_type == "4340":
            self.params = {
                'Q_MIN': -12.5,
                'Q_MAX': 12.5,
                'DQ_MAX':10.0,
                'TAU_MAX': 28.0
            }

        self.cmd = {
            'kp': None,
            'kd': None,
            'q': None,
            'dq': None,
            'tau': None
        }
        self.state = {
            'q': None,
            'dq': None,
            'tau': None,
            't_mos': None,
            't_rotor': None
        }

class CAN_Send_Frame:
    frame_header = [0x55, 0xAA]  # Frame header
    frame_len = 0x1e  # Frame length
    CMD = 0x01  # Command 1: Forward CAN data frame 2: PC and device handshake, device feedback OK 3: Non-feedback CAN forwarding, do not feedback send status
    send_times = 1  # Number of sends
    time_interval = 10  # Time interval
    IDType = 0  # ID type 0: Standard frame 1: Extended frame
    CANID = 0  # CAN ID, use motor ID as CAN ID
    frame_type = 0  # Frame type 0: Data frame 1: Remote frame
    len = 0x08  # Length
    idAcc = 0
    dataAcc = 0
    data = np.zeros(8, np.uint8)
    crc = 0  # Unparsed, any value

    def __init__(self) -> None:
        # TODO: Implement a proper serial packing and unpacking
        self.send_data = np.array([ 0x55, 0xAA,  # 0-1 Frame header 
                            0x1e,        # 2 Frame length
                            0x01,        # 3 Command 1: Forward CAN data frame 2: PC and device handshake, device feedback OK 3: Non-feedback CAN forwarding, do not feedback send status
                            0x01, 0x00, 0x00, 0x00, # 4 send times
                            0x0a, 0x00, 0x00, 0x00, # 8 send interval
                            0x00,  # 12 id type
                            0x00, 0x00, 0x00, 0x00, # 13 CAN ID
                            0x00, # 17 frame type
                            0x08, # 18 len
                            0x00, # 19 id Acc
                            0x00, # 20 data Acc
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, # 21 data buf
                            0x88], # 29 crc, anything works for now
                            np.uint8)

    def modify(self, id, data):
        self.send_data[13] = id
        self.send_data[21:29] = data

def float_to_uint(x, x_min, x_max,bits):
    span = x_max - x_min
    data_norm = (float(x) - x_min) / span
    return  (np.uint16(data_norm * ((1 << bits) - 1)))

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return float(x_int * span / ((1 << bits) - 1) + offset)

def usleep(us):
    time.sleep(us/1000000.0)

def control_motor(motor_id, send_frame, kp, kd, q, dq, tau):
    global motor_list
    motor = motor_list[motor_id]
    q_uint = float_to_uint(q, motor.params['Q_MIN'],
                           motor.params['Q_MAX'], 16)
    dq_uint = float_to_uint(dq, -motor.params['DQ_MAX'],
                            motor.params['DQ_MAX'], 12)
    kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    tau_uint = float_to_uint(tau, -motor.params["TAU_MAX"],
                             motor.params["TAU_MAX"],12)
    data_buf = np.zeros(8, np.uint8)
    data_buf[0] = (q_uint >> 8)  & 0xFF
    data_buf[1] = q_uint & 0xFF
    data_buf[2] = dq_uint >> 4
    data_buf[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
    data_buf[4] = kp_uint & 0xFF
    data_buf[5] = kd_uint >> 4
    data_buf[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
    data_buf[7] = tau_uint & 0xFF

    send_frame.modify(motor_id, data_buf)
    print("Sending data: ", data_buf)
    ser.write(bytes(send_frame.send_data))
    ser.flush()
    recv_data()

def add_motor(motor_id, master_id, motor_type):
    motor = Motor(motor_id, motor_type)
    motor_list[motor_id] = motor
    motor_list[master_id] = motor

def control_cmd(motor_id, send_frame, cmd):
    data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
    send_frame.modify(motor_id, data_buf)
    ser.write(bytes(send_frame.send_data))
    ser.flush()
    usleep(1000)
    recv_data()

def enable(motor_id, send_frame):
    control_cmd(motor_id, send_frame, 0xFC)

def disable(motor_id, send_frame):
    control_cmd(motor_id, send_frame, 0xFD)

def zero_position(motor_id, send_frame):
    control_cmd(motor_id, send_frame, 0xFE)

def reset(motor_id, send_frame):
    control_cmd(motor_id, send_frame, 0xFF)

def recv_data():
    global ser
    fds = [ser]
    for ser in fds:
        # TODO: Implement a proper serial packing and unpacking
        # Currently just read by bytes, may be incorrect for values that spans multiple bytes
        data = np.array(list(ser.read(ser.in_waiting)), np.uint8)
        for i in range(0, len(data), 16):
            data_arr = (data[i:i+16])
            if data_arr[0] == np.uint8(0xAA): 
                if data_arr[1] == np.uint8(0x11) and data_arr[-1] == np.uint8(0x55):
                    master_id = data_arr[7] # TODO may be problematic
                    data_buf = data_arr[7:15]
                    print(f"Received for {master_id}: {data_buf} ")

                    q_uint = (data_buf[1] << 8) | data_buf[2]
                    dq_uint = (data_buf[3] << 4) | (data_buf[4] >> 4)
                    tau_uint = (data_buf[4] & 0xf) << 8 | data_buf[5]
                    t_mos = data_buf[6]
                    t_rotor = data_buf[7]
                    q = uint_to_float(q_uint, motor_list[master_id].params['Q_MIN'], motor_list[master_id].params['Q_MAX'], 16)
                    dq = uint_to_float(dq_uint, -motor_list[master_id].params['DQ_MAX'], motor_list[master_id].params['DQ_MAX'], 12)
                    tau = uint_to_float(tau_uint, -motor_list[master_id].params["TAU_MAX"], motor_list[master_id].params["TAU_MAX"], 12)
                    motor_list[master_id].state['q'] = q
                    motor_list[master_id].state['dq'] = dq
                    motor_list[master_id].state['tau'] = tau
                    motor_list[master_id].state['t_mos'] = t_mos
                    motor_list[master_id].state['t_rotor'] = t_rotor

if __name__ == "__main__":  
    np.set_printoptions(formatter={'int':hex})
    ser = serial.Serial("/dev/ttyACM0", 921600, timeout=0.1)
    if ser.isOpen():
        print("Serial port opened successfully")

    # CAN baudrate
    # ser.write(bytes(np.array([0x55,0x05,0x00,0xAA,0x55],np.uint8)))
    # ser.flush()

    motor_list = {}
    motor_ids = [(np.uint8(0x0A), np.uint8(0x1A), "4340"),
                (np.uint8(0x0B), np.uint8(0x1B), "4340")]

    send_frame = CAN_Send_Frame()
    for motor_id, master_id, motor_type in motor_ids:
        add_motor(motor_id, master_id, motor_type)
        enable(motor_id, send_frame)

    while True:
        control_motor(motor_ids[0][0], send_frame, 0, 0, 0, 0, 0)
        q1 = motor_list[motor_ids[0][0]].state['q']
        control_motor(motor_ids[1][0], send_frame, 50, 1, q1, 0, 0)