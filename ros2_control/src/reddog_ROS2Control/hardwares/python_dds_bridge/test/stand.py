import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

def real_ang2mujoco_ang(dof_pos):
    flat_dof_pos = [0] * 12
    for i in range(3):
        for j in range(4):
            flat_dof_pos[4*i+j] = dof_pos[i][j]

    mujoco_order = ['frh', 'fru', 'frd', 
                    'flh', 'flu', 'fld',  
                    'rrh', 'rru', 'rrd',
                    'rlh', 'rlu', 'rld']
    

    motor_order = ['flh', 'frh', 'rlh', 'rrh',  # Hips
               'flu', 'fru', 'rlu', 'rru',  # Upper legs
               'fld', 'frd', 'rld', 'rrd']  # Lower legs
        
    index_map = [motor_order.index(name) for name in mujoco_order]

    reordered_dof_pos = [flat_dof_pos[i] for i in index_map]

    reordered_dof_pos = [-reordered_dof_pos[0], -reordered_dof_pos[1], -reordered_dof_pos[2],  
                         -reordered_dof_pos[3],  reordered_dof_pos[4],  reordered_dof_pos[5],  
                         -reordered_dof_pos[6], -reordered_dof_pos[7], -reordered_dof_pos[8], 
                         -reordered_dof_pos[9],  reordered_dof_pos[10], reordered_dof_pos[11]]
    
    return reordered_dof_pos

def mujoco_ang2real_ang(dof_pos):
    motor_order = ['flh', 'frh', 'rlh', 'rrh',  # Hips
               'flu', 'fru', 'rlu', 'rru',  # Upper legs
               'fld', 'frd', 'rld', 'rrd']  # Lower legs

    mujoco_order = ['frh', 'fru', 'frd', 
                    'flh', 'flu', 'fld',  
                    'rrh', 'rru', 'rrd',
                    'rlh', 'rlu', 'rld']
        
    index_map = [mujoco_order.index(name) for name in motor_order]

    reordered_dof_pos = [dof_pos[i] for i in index_map]

    reordered_dof_pos = [[-reordered_dof_pos[0], -reordered_dof_pos[1], -reordered_dof_pos[2], -reordered_dof_pos[3]],  
                         [ reordered_dof_pos[4], -reordered_dof_pos[5],  reordered_dof_pos[6], -reordered_dof_pos[7]], 
                         [ reordered_dof_pos[8], -reordered_dof_pos[9],  reordered_dof_pos[10],-reordered_dof_pos[11]]]
    
    return reordered_dof_pos

# default_joint_angles = np.array([ 0.1, 0.785,-1.57, #FR
#                                  -0.1, 0.785,-1.57, #FL
#                                   0.1,-0.785, 1.57, #RR
#                                  -0.1,-0.785, 1.57], #RL
#                                 dtype=float)

# [FR_hip -1.05~1.05, FR_thigh 0.1~1.57, FR_calf -2.72~-0.3,
#  FL_hip -1.05~1.05, FL_thigh 0.1~1.57, FL_calf -2.72~-0.3,
#  RR_hip -1.05~1.05, RR_thigh -1.57~-0.1, RR_calf 0.3~2.72,
#  RL_hip -1.05~1.05, RL_thigh -1.57~-0.1, RL_calf 0.3~2.72,]

class Go2Channel:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.sub = None
        self.sub0 = None
        self.low_cmd = None

        self.low_state = None
        self.quaternion = [0.0, 0.0, 0.0, 0.0]
        self.gyroscope = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.imu_temperature = 0.0

        self.q = []  # List of motor positions
        self.dq = []  # List of motor velocities
        self.ddq = []  # List of motor accelerations
        self.tau_est = []  # List of estimated torques
        self.motor_temperature = []  # List of motor temperatures
        self.motor_lost = []  # List of lost motor states
        self.motor_reserve = []  # List of motor reserves

        self.bms_version = (0, 0)
        self.bms_status = 0
        self.bms_soc = 0
        self.bms_current = 0
        self.bms_cycle = 0
        self.bms_cell_vol = [0] * 16

        self.foot_force = [0.0, 0.0, 0.0, 0.0]
        self.foot_force_est = [0.0, 0.0, 0.0, 0.0]

        self.tick = 0
        self.wireless_remote = b'\x00' * 32
        self.bit_flag = 0
        self.adc_reel = 0.0
        self.temperature_ntc1 = 0
        self.temperature_ntc2 = 0
        self.power_v = 0.0
        self.power_a = 0.0
        self.fan_frequency = [0] * 4
        self.reserve = 0
        self.crc = 0

        self.q = []
        self.dq = []
        self.tau = []
        self.kp = []
        self.kd = []
        self.motor_reserve = []

        self.version = None

        # if len(ether_name)>1:
        #     ChannelFactoryInitialize(0, ether_name)
        # else:
        #     ChannelFactoryInitialize(0)

        # # Create a publisher to publish the data defined in UserData class
        # self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        # self.pub.Init()

        # Create a subscriber to receive the latest robot state every certain seconds 
        # self.low_state = None
        # self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        # self.sub.Init(self.LowStateMessageHandler, 10)  

        # self.sub0 = ChannelSubscriber("rt/lowcmd", LowCmd_)
        # self.sub0.Init(self.LowCmdMessageHandler, 10)  

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        if self.low_state is not None:
            # Parse IMU state and store the quaternion, gyroscope, and accelerometer data
            self.quaternion = self.low_state.imu_state.quaternion
            self.gyroscope = self.low_state.imu_state.gyroscope
            self.accelerometer = self.low_state.imu_state.accelerometer
            self.rpy = self.low_state.imu_state.rpy
            self.imu_temperature = self.low_state.imu_state.temperature

            # Initialize the motor state lists
            self.q.clear()
            self.dq.clear()
            self.ddq.clear()
            self.tau_est.clear()
            self.motor_temperature.clear()
            self.motor_lost.clear()
            self.motor_reserve.clear()

            # Parse motor states and store relevant information for each motor
            for motor_state in self.low_state.motor_state:
                self.q.append(motor_state.q)
                self.dq.append(motor_state.dq)
                self.ddq.append(motor_state.ddq)
                self.tau_est.append(motor_state.tau_est)
                self.motor_temperature.append(motor_state.temperature)
                self.motor_lost.append(motor_state.lost)
                self.motor_reserve.append(motor_state.reserve)

            # Parse BMS state and store its values
            self.bms_version = (self.low_state.bms_state.version_high, self.low_state.bms_state.version_low)
            self.bms_status = self.low_state.bms_state.status
            self.bms_soc = self.low_state.bms_state.soc
            self.bms_current = self.low_state.bms_state.current
            self.bms_cycle = self.low_state.bms_state.cycle
            self.bms_cell_vol = self.low_state.bms_state.cell_vol

            # Parse foot force data
            self.foot_force = self.low_state.foot_force
            self.foot_force_est = self.low_state.foot_force_est

            # Parse tick, wireless remote, bit_flag, and other related fields
            self.tick = self.low_state.tick
            self.wireless_remote = self.low_state.wireless_remote
            self.bit_flag = self.low_state.bit_flag
            self.adc_reel = self.low_state.adc_reel
            self.temperature_ntc1 = self.low_state.temperature_ntc1
            self.temperature_ntc2 = self.low_state.temperature_ntc2
            self.power_v = self.low_state.power_v
            self.power_a = self.low_state.power_a
            self.fan_frequency = self.low_state.fan_frequency
            self.reserve = self.low_state.reserve
            self.crc = self.low_state.crc

    def LowCmdMessageHandler(self, msg: LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            # Initialize motor state lists
            self.q = []
            self.dq = []
            self.tau = []
            self.kp = []
            self.kd = []
            self.motor_reserve = []

            # Parse motor commands and store relevant information for each motor
            for motor_cmd in self.low_cmd.motor_cmd:
                self.q.append(motor_cmd.q)
                self.dq.append(motor_cmd.dq)
                self.tau.append(motor_cmd.tau)
                self.kp.append(motor_cmd.kp)
                self.kd.append(motor_cmd.kd)
                self.motor_reserve.append(motor_cmd.reserve)

            # Parse other fields
            self.head = self.low_cmd.head
            self.level_flag = self.low_cmd.level_flag
            self.frame_reserve = self.low_cmd.frame_reserve
            self.sn = self.low_cmd.sn
            self.version = self.low_cmd.version
            self.bandwidth = self.low_cmd.bandwidth
            self.bms_cmd_off = self.low_cmd.bms_cmd.off
            self.bms_cmd_reserve = self.low_cmd.bms_cmd.reserve
            self.wireless_remote = self.low_cmd.wireless_remote
            self.led = self.low_cmd.led
            self.fan = self.low_cmd.fan
            self.gpio = self.low_cmd.gpio
            self.reserve = self.low_cmd.reserve
            self.crc = self.low_cmd.crc

            # Parse BMS command (if applicable)
            self.bms_cmd = {
                "off": self.bms_cmd_off,
                "reserve": self.bms_cmd_reserve
            }

            # Parse wireless remote data
            self.wireless_remote_data = self.wireless_remote.hex()

            # Parse LED data (might be used for visual feedback)
            self.led_data = self.led.hex()

            # Parse fan control data
            self.fan_data = self.fan.hex()
    
def main():
    initial_joint_angles = np.array([
            [ 0.0,   0.0,  0.0,   0.0 ],
            [ 1.6, - 1.6, -1.6,   1.6 ], 
            [  -3,     3,    3,    -3 ], 
        ])
            
    default_joint_angles = np.array([
                            [  0.0,   0.0,   0.0, 0.0],     # Hip
                            [ 2.4, - 2.4, -2.4,   2.4 ],  # Upper leg
                            [ -2.7,   2.7,  2.7, -2.7]  # Lower leg
                        ])

    command_joint_angles = np.array([
                                [    0.1,   -0.1,    0.1,   -0.1],   # Hip
                                [  0.785, -0.785, -0.785,  0.785],  # Upper leg
                                [  -1.57,   1.57,   1.57,  -1.57]  # Lower leg
                            ])
    dt = 0.002
    runing_time = 0.0
    crc = CRC()

    input("Press enter to start")
    
    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
        ch = Go2Channel()
    else:
        ChannelFactoryInitialize(0, sys.argv[1])
        ch = Go2Channel(sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()    

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    initial_joint_angles = real_ang2mujoco_ang(initial_joint_angles)
    default_joint_angles = real_ang2mujoco_ang(default_joint_angles)
    # command_joint_angles = real_ang2mujoco_ang(command_joint_angles)
    command_joint_angles = [0.0, 1.6, -2.99, -0.0, 1.6, -2.99, -0.0, -1.6, 2.99, 0.7, -0.1, 0.1]

    # for i in range(12):
    #     cmd.motor_cmd[i].q = initial_joint_angles[i]
    #     cmd.motor_cmd[i].kp = 10
    #     cmd.motor_cmd[i].dq = 0.0
    #     cmd.motor_cmd[i].kd = 0.4
    #     cmd.motor_cmd[i].tau = 0

    # cmd.crc = crc.Crc(cmd)
    # pub.Write(cmd)
    print("Set to default angle")

    run_time = time.perf_counter()
    step_start = 0
    while (step_start - run_time) < 10:
        step_start = time.perf_counter()

        runing_time += dt

        # if (runing_time < 3.0):
        #     phase = np.tanh(runing_time / 2)
        #     for i in range(12):
        #         cmd.motor_cmd[i].q = phase * command_joint_angles[i] + (1 - phase) * default_joint_angles[i]
        #         cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
        #         cmd.motor_cmd[i].dq = 0.0
        #         cmd.motor_cmd[i].kd = 3.5
        #         cmd.motor_cmd[i].tau = 0
        # else:
        #     # Then stand down
        #     phase = np.tanh((runing_time - 3.0) / 2)
        #     for i in range(12):
        #         cmd.motor_cmd[i].q = phase * default_joint_angles[i] + (1 - phase) * command_joint_angles[i]
        #         cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
        #         cmd.motor_cmd[i].dq = 0.0
        #         cmd.motor_cmd[i].kd = 3.5
        #         cmd.motor_cmd[i].tau = 0

        if (runing_time < 3.0):
            phase = np.tanh(runing_time / 2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * command_joint_angles[i] + (1 - phase) * initial_joint_angles[i]
                cmd.motor_cmd[i].kp = 10
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 0.4
                cmd.motor_cmd[i].tau = 0
        else:
            # Then stand down
            phase = np.tanh((runing_time - 3.0) / 2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * initial_joint_angles[i] + (1 - phase) * command_joint_angles[i]
                cmd.motor_cmd[i].kp = 10
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 0.4
                cmd.motor_cmd[i].tau = 0

        # for i in range(12):
        #     cmd.motor_cmd[i].q = default_joint_angles[i]
        #     cmd.motor_cmd[i].kp = 10
        #     cmd.motor_cmd[i].dq = 0.0
        #     cmd.motor_cmd[i].kd = 0.4
        #     cmd.motor_cmd[i].tau = 0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

if __name__ == '__main__':
    main()