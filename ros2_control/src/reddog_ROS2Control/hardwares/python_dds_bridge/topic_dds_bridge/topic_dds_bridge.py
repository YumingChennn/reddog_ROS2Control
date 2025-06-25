import sys
import time
import threading
import traceback
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class JointStatePub(Node):
    def __init__(self):
        super().__init__('topicddsbridge_jointst_pub')
        self.publisher = self.create_publisher(
            JointState,
            'cmd/joint_states',
            10
        )

class PIDGainPub(Node):
    def __init__(self):
        super().__init__('topicddsbridge_pidgain_pub')
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'pid_gain',
            10
        )

class JointStateSub(Node):
    def __init__(self):
        super().__init__('topicddsbridge_jointst_sub')
        self.subscription = self.create_subscription(
            JointState,
            '/low_level_info/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to joint state topic.')
        self.jointAngle_data = {}
        self.jointVelocity_data = {}

        self.dof_pos = []
        self.dof_vel = []

    def joint_state_callback(self, msg):
        joint_names = msg.name
        positions = msg.position
        velocities = msg.velocity

        for i, name in enumerate(joint_names):
            self.jointAngle_data[name] = positions[i]
            self.jointVelocity_data[name] = velocities[i]

    def get_jointAngle_data(self):
        # determine the joint position and velocity is not empty dict
        if bool(self.jointAngle_data) and bool(self.jointVelocity_data):
            self.dof_pos = [self.jointAngle_data['flh'],  self.jointAngle_data['flu'],  self.jointAngle_data['fld'],  
                            self.jointAngle_data['frh'],  - self.jointAngle_data['fru'],  - self.jointAngle_data['frd'], 
                            -self.jointAngle_data['rlh'],  self.jointAngle_data['rlu'],  self.jointAngle_data['rld'],    
                            -self.jointAngle_data['rrh'],  - self.jointAngle_data['rru'],  - self.jointAngle_data['rrd']]
        
            self.dof_vel = [self.jointVelocity_data['flh'],  self.jointVelocity_data['flu'],  self.jointVelocity_data['fld'],  
                            self.jointVelocity_data['frh'],  - self.jointVelocity_data['fru'],  - self.jointVelocity_data['frd'], 
                            -self.jointVelocity_data['rlh'],  self.jointVelocity_data['rlu'],  self.jointVelocity_data['rld'],    
                            -self.jointVelocity_data['rrh'],  - self.jointVelocity_data['rru'],  - self.jointVelocity_data['rrd']]
        else:
            self.dof_pos = [-1] * 12
            self.dof_vel = [-1] * 12

        return self.dof_pos, self.dof_vel

class LinearASub(Node):
    def __init__(self):
        super().__init__('Imu_linear_acceleration_subscriber')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/acceleration_hr',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to linear acceleration topic.')
        self.linear_acceleration = None

    def joint_state_callback(self, msg):
        self.linear_acceleration = msg.vector
        # self.get_logger().info(f"Received linear acceleration: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

    def get_Imu_data(self):
        return self.linear_acceleration

class AngularVSub(Node):
    def __init__(self):
        super().__init__('Imu_angular_velocity_subscriber')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/angular_velocity_hr',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to angular velocity topic.')
        self.angular_velocity = None

    def joint_state_callback(self, msg):
        self.angular_velocity = msg.vector
        # self.get_logger().info(f"Received angular velocity: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

    def get_Imu_data(self):
        return self.angular_velocity

class QuaternionSub(Node):
    def __init__(self):
        super().__init__('Imu_quaternion_subscriber')
        self.subscription = self.create_subscription(
            QuaternionStamped,
            '/filter/quaternion',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to quaternion topic.')
        self.quaternion = None

    def joint_state_callback(self, msg):
        self.quaternion = msg.quaternion

    def get_Imu_data(self):
        return self.quaternion

class DDSHandler:
    def __init__(self, ether_name: str=""):
        # self.jointStateSub = JointStateSub()

        # self.pidGainPub = PIDGainPub()
        # self.jointStatePub = JointStatePub() 

        self.pub = None
        self.low_state = None
        self.pub = ChannelPublisher("rt/lowstate", LowState_)
        self.pub.Init()  

        self.sub = None
        self.low_cmd = None
        self.sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.sub.Init(self.LowCmdMessageHandler, 10) 

        self.joint_state_msg = JointState()
        self.last_joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'flh', 'frh', 'rlh', 'rrh',   # Hips
            'flu', 'fru', 'rlu', 'rru',  # Upper legs
            'fld', 'frd', 'rld', 'rrd'   # Lower legs
        ]

        self.pid_gain_msg = Float32MultiArray()
        self.pid_gain_msg.data = [float(0), float(0)]

    def mujoco_ang2real_ang(self, dof_pos):
        motor_order = ['flh', 'frh', 'rlh', 'rrh',  # Hips
                'flu', 'fru', 'rlu', 'rru',  # Upper legs
                'fld', 'frd', 'rld', 'rrd']  # Lower legs

        mujoco_order = ['frh', 'fru', 'frd', 
                        'flh', 'flu', 'fld',  
                        'rrh', 'rru', 'rrd',
                        'rlh', 'rlu', 'rld']
            
        index_map = [mujoco_order.index(name) for name in motor_order]

        reordered_dof_pos = [dof_pos[i] for i in index_map]

        reordered_dof_pos = [-reordered_dof_pos[0], -reordered_dof_pos[1], -reordered_dof_pos[2], -reordered_dof_pos[3],  
                             reordered_dof_pos[4], -reordered_dof_pos[5],  reordered_dof_pos[6], -reordered_dof_pos[7], 
                             reordered_dof_pos[8], -reordered_dof_pos[9],  reordered_dof_pos[10],-reordered_dof_pos[11]]
        
        return reordered_dof_pos

    def LowCmdMessageHandler(self, msg: LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            # print(self.low_cmd.motor_cmd)
            self.joint_state_msg.position = []
            dof_pos = []
            self.joint_state_msg.velocity = []
            self.joint_state_msg.effort = []
            i = 0
            for motor_cmd in self.low_cmd.motor_cmd:
                if i < 12:
                    dof_pos.append(motor_cmd.q)
                    self.pid_gain_msg.data[0] = motor_cmd.kp
                    self.pid_gain_msg.data[1] = motor_cmd.kd
                    i += 1
                else:
                    break

            self.joint_state_msg.position = self.mujoco_ang2real_ang(dof_pos)

            self.last_joint_state_msg = self.joint_state_msg
        else:
            self.joint_state_msg = self.last_joint_state_msg

class Controller:
    def __init__(self):
        if len(sys.argv) <2:
            ChannelFactoryInitialize(1, "lo")
            self.dds_handler = DDSHandler()
        else:
            ChannelFactoryInitialize(0, sys.argv[1])
            self.dds_handler = DDSHandler(sys.argv[1])

        rclpy.init()
        self.jointStateSub = JointStateSub()
        self.linearASub = LinearASub()
        self.angularVSub = AngularVSub()
        self.quaternionSub = QuaternionSub()
        self.jointStatePub = JointStatePub()
        self.pidGainPub = PIDGainPub()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.jointStateSub)
        self.executor.add_node(self.angularVSub)
        self.executor.add_node(self.linearASub)
        self.executor.add_node(self.quaternionSub)
        self.executor.add_node(self.jointStatePub)
        self.executor.add_node(self.pidGainPub)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # time.sleep(1)
        self.low_cmd2topic_thread = threading.Thread(target=self.low_cmd2topic) 
        self.update_low_cmd_thread = threading.Thread(target=self.update_low_cmd)

        self.topic2low_state_thread = threading.Thread(target=self.topic2low_state)

        self.is_running = True
        self.delay = 1/300

        self.run_low_cmd2topic_frequency = 200  # Hz
        self.run_topic2low_state_frequency = 1000  # Hz

        self.joint_state_msg = JointState()
        self.pid_gain_msg = Float32MultiArray()

    def run(self):
        self.is_running = True
        self.run_thread = threading.Thread(target=self._run)
        self.run_thread.start()

    def _run(self):
        while self.is_running:
            self.joint_state_msg = self.dds_handler.joint_state_msg
            self.pid_gain_msg = self.dds_handler.pid_gain_msg
            time.sleep(self.delay)
            self.jointStatePub.publisher.publish(self.joint_state_msg)
            self.pidGainPub.publisher.publish(self.pid_gain_msg)

            joint_dof_pos, joint_dof_vel = self.jointStateSub.get_jointAngle_data()

            quaternion = self.quaternionSub.get_Imu_data()
            angular_velocity = self.angularVSub.get_Imu_data()
            linear_acceleration = self.linearASub.get_Imu_data()
            if quaternion is None or angular_velocity is None or linear_acceleration is None:
                print("Error: IMU data is None")
                continue

            low_state = unitree_go_msg_dds__LowState_()
            low_state.head[0] = 0x00
            low_state.head[1] = 0x00
            low_state.level_flag = 0x00
            
            low_state.imu_state.quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
            low_state.imu_state.gyroscope = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
            low_state.imu_state.accelerometer = [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]

            for i in range(12):
                low_state.motor_state[i].q = joint_dof_pos[i]
                low_state.motor_state[i].dq = joint_dof_vel[i]
                low_state.motor_state[i].tau_est = 0.0

            # low_state.bms_state.crc = crc.Crc(low_state)
            self.dds_handler.pub.Write(low_state)       


    def reset(self):
        self.is_running = False
        joint_state_msg = JointState()
        joint_state_msg.name = [
            'flh', 'frh', 'rlh', 'rrh',   # Hips
            'flu', 'fru', 'rlu', 'rru',  # Upper legs
            'fld', 'frd', 'rld', 'rrd'   # Lower legs
        ]
        joint_state_msg.position = []
        self.jointStatePub.publisher.publish(joint_state_msg)

        pid_gain_msg = Float32MultiArray()
        pid_gain_msg.data = [0.0, 0.0]
        self.pidGainPub.publisher.publish(pid_gain_msg)

    def stop(self):
        self.is_running = False
        if self.low_cmd2topic_thread.is_alive():
            self.low_cmd2topic_thread.join()
        if self.topic2low_state_thread.is_alive():
            self.topic2low_state_thread.join()

        self.low_cmd2topic_thread = threading.Thread(target=self.low_cmd2topic)
        self.topic2low_state_thread = threading.Thread(target=self.topic2low_state)

    def run_low_cmd2topic(self):
        self.is_running = True
        self.low_cmd2topic_thread.start()
        self.update_low_cmd_thread.start()

    def run_topic2low_state(self):
        self.is_running = True
        self.topic2low_state_thread.start()

    def update_low_cmd(self):
        while self.is_running:
            self.joint_state_msg = self.dds_handler.joint_state_msg
            self.pid_gain_msg = self.dds_handler.pid_gain_msg

    def low_cmd2topic(self):
        while self.is_running:
            st_time = time.perf_counter()
            # joint_state_msg = self.dds_handler.joint_state_msg
            # pid_gain_msg = self.dds_handler.pid_gain_msg
            time.sleep(self.delay)
            self.jointStatePub.publisher.publish(self.joint_state_msg)
            self.pidGainPub.publisher.publish(self.pid_gain_msg)
            # print(f"Running frequency of topic2low_cmd: {1 / (time.perf_counter() - st_time):.2f} Hz")  
            # runing_hz = 1 / (time.perf_counter() - st_time)

            # if runing_hz > self.run_low_cmd2topic_frequency:
            #     time.sleep(1 / self.run_low_cmd2topic_frequency - (time.perf_counter() - st_time))
  

    def topic2low_state(self):
        while self.is_running:
            # st_time = time.perf_counter()
            joint_dof_pos, joint_dof_vel = self.jointStateSub.get_jointAngle_data()

            quaternion = self.quaternionSub.get_Imu_data()
            angular_velocity = self.angularVSub.get_Imu_data()
            linear_acceleration = self.linearASub.get_Imu_data()
            if quaternion is None or angular_velocity is None or linear_acceleration is None:
                print("Error: IMU data is None")
                return -1

            low_state = unitree_go_msg_dds__LowState_()
            low_state.head[0] = 0x00
            low_state.head[1] = 0x00
            low_state.level_flag = 0x00
            
            low_state.imu_state.quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
            low_state.imu_state.gyroscope = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
            low_state.imu_state.accelerometer = [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]

            for i in range(12):
                low_state.motor_state[i].q = joint_dof_pos[i]
                low_state.motor_state[i].dq = joint_dof_vel[i]
                low_state.motor_state[i].tau_est = 0.0

            # low_state.bms_state.crc = crc.Crc(low_state)
            self.dds_handler.pub.Write(low_state)

            # print(f"Running frequency of topic2low_state: {1 / (time.perf_counter() - st_time):.2f} Hz")

    def test_pub(self):
        default_joint_angles = np.array([
                                  0.0,   0.0,   0.0, 0.0,     # Hip
                                 2.4, - 2.4, -2.4,   2.4 ,  # Upper leg
                                -2.7,   2.7,  2.7, -2.7  # Lower leg
                            ])

        command_joint_angles = np.array([
                                       0.1,   -0.1,    0.1,   -0.1,   # Hip
                                     0.785, -0.785, -0.785,  0.785,  # Upper leg
                                     -1.57,   1.57,   1.57,  -1.57 # Lower leg
                                ])

        run_time = time.perf_counter()
        runing_time = 0
        step_start = 0
        while (step_start - run_time) < 10:
            step_start = time.perf_counter()

            joint_state_msg = JointState()
            joint_state_msg.name = [
                'flh', 'frh', 'rlh', 'rrh',   # Hips
                'flu', 'fru', 'rlu', 'rru',  # Upper legs
                'fld', 'frd', 'rld', 'rrd'   # Lower legs
            ]
            joint_state_msg.position = []
            joint_state_msg.velocity = []
            joint_state_msg.effort = []

            pid_gain_msg = Float32MultiArray()
            pid_gain_msg.data = [10.0, 0.4]

            runing_time += dt
            if (runing_time < 3.0):
                phase = np.tanh(runing_time / 2)
                for i in range(12):
                    joint_state_msg.position.append(phase * command_joint_angles[i] + (1 - phase) * default_joint_angles[i])
            else:
                # Then stand down
                phase = np.tanh((runing_time - 3.0) / 2)
                for i in range(12):
                    joint_state_msg.position.append(phase * default_joint_angles[i] + (1 - phase) * command_joint_angles[i])
                
            self.jointStatePub.publisher.publish(joint_state_msg)
            self.pidGainPub.publisher.publish(pid_gain_msg)

            time_until_next_step = dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


dt = 0.002
runing_time = 0.0
crc = CRC()
def main():
    controller = Controller()

    # start_time = time.perf_counter()
    # now_time = time.perf_counter()
    # while (now_time - start_time) < 360:
    #     now_time = time.perf_counter()
    
        # controller.low_cmd2topic()
        # controller.topic2low_state()
        # controller.test_pub()
    # controller.run()

    cmd_dict = {
        'run': controller.run,
        'lc2t': controller.run_low_cmd2topic,
        't2ls': controller.run_topic2low_state,
        'stop': controller.stop,
        'reset': controller.reset,
    }
    print("Commands:  run, lc2t, t2ls, stop, reset, exit")
    try:
        while True:
            cmd = input("Enter command: ").strip()
            if cmd in cmd_dict:
                cmd_dict[cmd]()
            elif cmd == "exit":
                print("Exiting...")
                controller.is_running = False
                break
            else:
                print("Unknown command. Available commands: run, lc2t, t2ls, stop, exit")
    except KeyboardInterrupt:
        print("Exiting...")
        controller.is_running = False
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()

if __name__ == '__main__':
    main()