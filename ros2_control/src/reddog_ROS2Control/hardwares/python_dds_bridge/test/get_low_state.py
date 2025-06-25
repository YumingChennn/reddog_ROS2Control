import sys
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


class DDSHandler:
    def __init__(self, ether_name: str = ""):
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10)
        self.low_state = None

        self.imu = None
        self.motors = None

        self.imu_acc_x = [0]
        self.imu_acc_y = [0]
        self.imu_acc_z = [0]

        self.quat_w = [0]
        self.quat_x = [0]
        self.quat_y = [0]
        self.quat_z = [0]

        self.gyro_x = [0]
        self.gyro_y = [0]
        self.gyro_z = [0]

        self.rpy_roll = [0]
        self.rpy_pitch = [0]
        self.rpy_yaw = [0]

        self.q = [[0] for _ in range(12)]  # 12 motors

    def LowStateMessageHandler(self, msg: LowState_):
        if msg is not None:
            self.low_state = msg
            self.imu = msg.imu_state
            self.motors = msg.motor_state

            self.imu_acc_x.append(self.imu.accelerometer[0])
            self.imu_acc_y.append(self.imu.accelerometer[1])
            self.imu_acc_z.append(self.imu.accelerometer[2])

            self.quat_w.append(self.imu.quaternion[0])
            self.quat_x.append(self.imu.quaternion[1])
            self.quat_y.append(self.imu.quaternion[2])
            self.quat_z.append(self.imu.quaternion[3])

            self.gyro_x.append(self.imu.gyroscope[0])
            self.gyro_y.append(self.imu.gyroscope[1])
            self.gyro_z.append(self.imu.gyroscope[2])

            self.rpy_roll.append(self.imu.rpy[0])
            self.rpy_pitch.append(self.imu.rpy[1])
            self.rpy_yaw.append(self.imu.rpy[2])

            for i in range(12):
                # print(f"Motor {i} position: {self.motors.q[i]}")
                self.q[i].append(self.motors[i].q)


# === Main ===
if __name__ == "__main__":
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
        dds_handler = DDSHandler()
    else:
        ChannelFactoryInitialize(0, sys.argv[1])
        dds_handler = DDSHandler(sys.argv[1])


    try:
        while True:
            # print(dds_handler.low_state)
            # print(f"IMU Accelerometer: {dds_handler.imu_acc_x[-1]}, {dds_handler.imu_acc_y[-1]}, {dds_handler.imu_acc_z[-1]}")
            # print(f"Motors: {dds_handler.motors}")
            print(f"Motors: {dds_handler.q[0][-1]}")
            time.sleep(1/500)
    except KeyboardInterrupt:
        print("Exiting...")
        sys.exit(0)
