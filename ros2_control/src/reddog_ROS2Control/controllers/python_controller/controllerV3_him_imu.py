import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped
from scipy.signal import butter, filtfilt
import numpy as np
import time
import torch
import yaml
import matplotlib.pyplot as plt
import argparse
import threading

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0

        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.get_logger().info('Vel Subscriber has been started')

    def listener_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.linear_z = msg.linear.z

        self.angular_x = msg.angular.x
        self.angular_y = msg.angular.y
        self.angular_z = msg.angular.z

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('low_level_info_motor_subscriber')
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
        self.dof_pos = [self.jointAngle_data['flh'],  self.jointAngle_data['flu'],  self.jointAngle_data['fld'],  
                        self.jointAngle_data['frh'],  - self.jointAngle_data['fru'],  - self.jointAngle_data['frd'], 
                        -self.jointAngle_data['rlh'],  self.jointAngle_data['rlu'],  self.jointAngle_data['rld'],    
                        -self.jointAngle_data['rrh'],  - self.jointAngle_data['rru'],  - self.jointAngle_data['rrd']]
    
        self.dof_vel = [self.jointVelocity_data['flh'],  self.jointVelocity_data['flu'],  self.jointVelocity_data['fld'],  
                        self.jointVelocity_data['frh'],  - self.jointVelocity_data['fru'],  - self.jointVelocity_data['frd'], 
                        -self.jointVelocity_data['rlh'],  self.jointVelocity_data['rlu'],  self.jointVelocity_data['rld'],    
                        -self.jointVelocity_data['rrh'],  - self.jointVelocity_data['rru'],  - self.jointVelocity_data['rrd']]
    
        return self.dof_pos, self.dof_vel
    
class AngularVSubscriber(Node):
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

class QuaternionSubscriber(Node):
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

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(JointState, 'cmd/joint_states', 10)
        self.kp_kq_publisher_ = self.create_publisher(Float32MultiArray, 'pid_gain', 10)
        self.timer_joint = self.create_timer(0.02, self.publish_joint_states)
        self.timer_pidgain = self.create_timer(0.02, self.publish_kp_kq)

        self.anguv_sub = AngularVSubscriber()
        self.qua_sub = QuaternionSubscriber()
        self.joint_state_sub = JointStateSubscriber()
        self.cmd_vel = CmdVelSubscriber()

        self.initial_joint_angles = np.array([
            [ 0.0,   0.0,  0.0,   0.0 ],
            [ 1.6, - 1.6, -1.6,   1.6 ], 
            [  -3,     3,    3,    -3 ], 
        ])

        self.default_joint_angles = np.array([
            [  0.0,   0.0,   0.0, 0.0],     # Hip
            [  2.4,  -2.4, -2.4,  2.4],  # Upper leg
            [ -2.7,   2.7,  2.7, -2.7]  # Lower leg
        ])

        # self.default_joint_angles = np.array([
        #     [  0.0,   0.0,   0.0, 0.0],     # Hip
        #     [  2.39,  -2.39, -2.39,  2.39],  # Upper leg
        #     [ -2.69,   2.69,  2.69, -2.69]  # Lower leg
        # ])

        self.command_joint_angles = np.array([
            [    0.1,   -0.1,    0.1,   -0.1],   # Hip
            [  0.785, -0.785, -0.785,  0.785],  # Upper leg
            [  -1.57,   1.57,   1.57,  -1.57]  # Lower leg
        ])

        self.current_angles = self.initial_joint_angles.copy()
        self.target_angles = None
        self.start_time = None 
        self.transition_duration = 1.2

        self.dof_pos = []
        self.dof_vel = []

        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.joint_state_sub)
        executor.add_node(self.anguv_sub)
        executor.add_node(self.qua_sub)
        executor.add_node(self.cmd_vel)
        spin_thread = threading.Thread(target= executor.spin, daemon=True)
        spin_thread.start()
        print("thread start")

        ChannelFactoryInitialize(1, "lo")
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()  

    def get_vel_data(self):
        self.horizontal_velocity = np.array([self.cmd_vel.linear_x, self.cmd_vel.linear_y]) * 4
        self.yaw_rate = self.cmd_vel.angular_z * 1.5
    
    def convert_joint_angles(self, dof_pos):
        mapping = ['flh', 'frh', 'rlh', 'rrh',  # Hips
                'flu', 'fru', 'rlu', 'rru',  # Upper legs
                'fld', 'frd', 'rld', 'rrd']  # Lower legs

        format1_order = ['flh', 'flu', 'fld',  
                        'frh', 'fru', 'frd', 
                        'rlh', 'rlu', 'rld',
                        'rrh', 'rru', 'rrd']
        
        index_map = [format1_order.index(name) for name in mapping]

        reordered_dof_pos = [dof_pos[i] for i in index_map]

        reordered_dof_pos = [ reordered_dof_pos[0],   reordered_dof_pos[1], -reordered_dof_pos[2], -reordered_dof_pos[3],
                            reordered_dof_pos[4], - reordered_dof_pos[5], reordered_dof_pos[6], -reordered_dof_pos[7],
                            reordered_dof_pos[8], - reordered_dof_pos[9], reordered_dof_pos[10], -reordered_dof_pos[11]]
        
        return reordered_dof_pos
    
    def publish_joint_states(self):
        msg = JointState()
        msg.name = [
            'flh', 'frh', 'rlh', 'rrh',   # Hips
            'flu', 'fru', 'rlu', 'rru',  # Upper legs
            'fld', 'frd', 'rld', 'rrd'   # Lower legs
        ]
        msg.position = np.array(self.current_angles).flatten().tolist()
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)

    def publish_kp_kq(self):
        msg = Float32MultiArray()
        msg.data = [float(self.kp), float(self.kq)]
        self.kp_kq_publisher_.publish(msg)

    def real_ang2mujoco_ang(self, dof_pos):
        # flat_dof_pos = [0.0] * 12
        # for i in range(3):
        #     for j in range(4):
        #         print(dof_pos)
        #         flat_dof_pos[4*i+j] = dof_pos[i][j]

        mujoco_order = ['frh', 'fru', 'frd', 
                        'flh', 'flu', 'fld',  
                        'rrh', 'rru', 'rrd',
                        'rlh', 'rlu', 'rld']
        

        motor_order = ['flh', 'frh', 'rlh', 'rrh',  # Hips
                'flu', 'fru', 'rlu', 'rru',  # Upper legs
                'fld', 'frd', 'rld', 'rrd']  # Lower legs
            
        index_map = [motor_order.index(name) for name in mujoco_order]

        reordered_dof_pos = [dof_pos[i] for i in index_map]

        reordered_dof_pos = [-reordered_dof_pos[0], -reordered_dof_pos[1], -reordered_dof_pos[2],  
                            -reordered_dof_pos[3],  reordered_dof_pos[4],  reordered_dof_pos[5],  
                            -reordered_dof_pos[6], -reordered_dof_pos[7], -reordered_dof_pos[8], 
                            -reordered_dof_pos[9],  reordered_dof_pos[10], reordered_dof_pos[11]]
        
        return reordered_dof_pos

    def send_cmd(self):
        self.publish_joint_states()
        self.publish_kp_kq()
        # crc = CRC()
        # cmd = unitree_go_msg_dds__LowCmd_()
        # cmd.head[0] = 0xFE
        # cmd.head[1] = 0xEF
        # cmd.level_flag = 0xFF
        # cmd.gpio = 0

        # position = np.array(self.current_angles).flatten().tolist()
        # position = self.real_ang2mujoco_ang(position)
        # for i in range(12):
        #     cmd.motor_cmd[i].q = position[i]
        #     cmd.motor_cmd[i].kp = self.kp
        #     cmd.motor_cmd[i].dq = 0.0
        #     cmd.motor_cmd[i].kd = self.kq
        #     cmd.motor_cmd[i].tau = 0

        # cmd.crc = crc.Crc(cmd)
        # self.pub.Write(cmd)

    def move_to_default_pos(self):
        self.kp = 3
        self.kq = 0.1
        self.current_angles = self.default_joint_angles.copy()    
        start_time = time.time()  

        while time.time() - start_time < 20:
            self.send_cmd()
            time.sleep(0.02)  # 50 Hz

        print("Moving to default pos completed.")

    def ready_to_standup(self):
        self.target_angles = self.command_joint_angles
        self.start_time = time.time() 
        self.kp = 10
        self.kq = 0.4

        while True:
            elapsed_time = time.time() - self.start_time
            phase = np.tanh(elapsed_time / self.transition_duration)

            self.current_angles = phase * self.target_angles + (1 - phase) * self.default_joint_angles
            self.send_cmd()

            if np.allclose(self.current_angles, self.target_angles, atol=0.01):
                self.current_angles = self.target_angles
                self.target_angles = None
                self.get_logger().info("Movement completed, ready for next command.")
                return  
            time.sleep(0.02)  # 50 Hz
        
    def keep_pos(self):
        start_time = time.time()  

        while time.time() - start_time < 10:
            self.send_cmd()
            time.sleep(0.02)  # 50 Hz
            
        print("Keeping the pos completed.")

        
    def get_gravity_orientation(self, quaternion):
        qw = quaternion[0]
        qx = quaternion[1]
        qy = quaternion[2]
        qz = quaternion[3]

        gravity_orientation = np.zeros(3)

        gravity_orientation[0] = 2 * (qx * qz - qw * qy)
        gravity_orientation[1] = -2 * (qy * qz + qw * qx)
        gravity_orientation[2] = -1 - 2 * (qx**2 + qy**2)

        return gravity_orientation

    def run(self, config_file):    
        with open(f"{config_file}", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            print("Config loaded successfully. Keys:", config.keys())

        policy_path = config["policy_path"]
        try:
            policy = torch.jit.load(policy_path)
            print("Policy loaded successfully")
        except Exception as e:
            print("Failed to load policy:", e)
            return

        default_angles = np.array(config["default_angles"], dtype=np.float32)
        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        one_step_obs_size = config["one_step_obs_size"]
        obs_buffer_size = config["obs_buffer_size"]

        cmd = np.array(config["cmd_init"], dtype=np.float32)

        # target_dof_pos = default_angles.copy()
        action = np.zeros(num_actions, dtype=np.float32)
        obs = np.zeros(num_obs, dtype=np.float32)

        time_step = 0  
        time_steps_list = []
        ang_vel_data_list = []
        gravity_b_list = []
        joint_vel_list = []
        action_list = []

        interval = 1.0 / 50  # 150 Hz -> 每次執行間隔 6.67 ms
        
        self.kp = 13
        self.kq = 0.4

        # kps = np.array(config["kps"], dtype=np.float32)
        # kds = np.array(config["kds"], dtype=np.float32)

        print("Entering main loop...")
        try:
            while True:
                start_time = time.time()
                
                angular_velocity = self.anguv_sub.get_Imu_data()
                orientation= self.qua_sub.get_Imu_data()
                self.dof_pos, self.dof_vel = self.joint_state_sub.get_jointAngle_data()
                self.get_vel_data()

                qpos = np.array(self.dof_pos, dtype=np.float32)
                qvel = np.array(self.dof_vel, dtype=np.float32)
                ang_vel_I = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z], dtype=np.float32)
                # ang_vel_I = np.array([  0,  0,  0], dtype=np.float32)
                gravity_b = self.get_gravity_orientation(np.array([orientation.w, orientation.x, orientation.y, orientation.z], dtype=np.float32))
                # gravity_b = np.array([0, 0, -1], dtype=np.float32)
                # cmd_vel = np.array(config["cmd_init"], dtype=np.float32)
                cmd_vel = np.array([self.horizontal_velocity[0], self.horizontal_velocity[1], self.yaw_rate], dtype=np.float32)

                print(f"cmd_vel: {cmd_vel}")

                # 記錄數據
                time_steps_list.append(time_step)
                ang_vel_data_list.append(ang_vel_I * ang_vel_scale *0.3)
                gravity_b_list.append(gravity_b )
                joint_vel_list.append(qvel * dof_vel_scale )
                action_list.append(action)

                obs_list = [
                    cmd_vel * cmd_scale,
                    ang_vel_I * ang_vel_scale*0.3,
                    gravity_b,
                    (qpos - default_angles) * dof_pos_scale,
                    qvel * dof_vel_scale,
                    action
                ]

                obs_list = [torch.tensor(obs, dtype=torch.float32) if isinstance(obs, np.ndarray) else obs for obs in obs_list]
                obs_tensor_buf = torch.zeros((1, one_step_obs_size * obs_buffer_size))
                obs = torch.cat(obs_list, dim=0).unsqueeze(0)
                obs_tensor = torch.clamp(obs, -100, 100)

                obs_tensor_buf = torch.cat([
                    obs_tensor,
                    obs_tensor_buf[:, :obs_buffer_size * one_step_obs_size - one_step_obs_size]
                ], dim=1)

                action = policy(obs_tensor_buf).detach().numpy().squeeze()
                current_angles = action * action_scale + default_angles
                # current_angles = default_angles.copy()
                self.current_angles = self.convert_joint_angles(current_angles)
                self.send_cmd()

                time_step += 1

                sleep_time = interval - (time.time() - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nSimulation interrupted. Plotting data...")

            # **畫圖**
            plt.figure(figsize=(14, 16))

            # plt.subplot(3, 2, 1)
            # for i in range(3): 
            #     plt.plot(time_steps_list, [step[i] for step in ang_vel_data_list], label=f"Angular Velocity {i}")
            # plt.title("History Angular Velocity", fontsize=10, pad=10)
            # plt.xlabel("Time Step")
            # plt.legend()

            # plt.subplot(3, 2, 2)
            # for i in range(3):
            #     plt.plot(time_steps_list, [step[i] for step in gravity_b_list], label=f"Project Gravity {i}")
            # plt.title("History Project Gravity", fontsize=10, pad=10)
            # plt.xlabel("Time Step")
            # plt.legend()

            # plt.subplot(3, 2, 3)
            # for i in range(2):
            #     plt.plot(time_steps_list, [step[i] for step in joint_vel_list], label=f"Joint Velocity {i}")
            # plt.title("History Joint Velocity", fontsize=10, pad=10)
            # plt.xlabel("Time Step")
            # plt.legend()

            # plt.subplot(3, 2, 4)
            # for i in range(2):
            #     plt.plot(time_steps_list, [step[i] for step in action_list], label=f"Velocity Command {i}")
            # plt.title("History Torque Command", fontsize=10, pad=10)
            # plt.xlabel("Time Step")
            # plt.legend()
            # plt.tight_layout()

            plt.subplot(4, 1, 1) #action
            for i in range(3):
                plt.plot(time_steps_list, [step[i] for step in action_list], label=f"Action {i}")
            plt.title("History Action 1-3", fontsize=10, pad=10)
            plt.xlabel("Time Step")
            plt.legend()
            plt.tight_layout()
            plt.subplot(4, 1, 2)
            for i in range(3, 6):
                plt.plot(time_steps_list, [step[i] for step in action_list], label=f"Action {i}")
            plt.title("History Action 4-6", fontsize=10, pad=10)
            plt.xlabel("Time Step")
            plt.legend()
            plt.subplot(4, 1, 3)
            for i in range(6, 9):
                plt.plot(time_steps_list, [step[i] for step in action_list], label=f"Action {i}")
            plt.title("History Action 7-9", fontsize=10, pad=10)
            plt.xlabel("Time Step")
            plt.legend()
            plt.subplot(4, 1, 4)
            for i in range(9, 12):
                plt.plot(time_steps_list, [step[i] for step in action_list], label=f"Action {i}")
            plt.title("History Action 10-12", fontsize=10, pad=10)
            plt.xlabel("Time Step")
            plt.legend()
            plt.tight_layout()

            plt.savefig("simulation_plot.png", dpi=300)  # 解析度 300，避免模糊
            plt.close() 
            print("Plot saved as 'simulation_plot.png'. Exiting.")

def main(args=None):
    parser = argparse.ArgumentParser(description='Controller for Quadruped.')
    parser.add_argument('config_file', type=str, help='Path to configuration file.')
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    
    print(f"Using config file: {args.config_file}")

    controller = MotorController()
    
    controller.move_to_default_pos()

    controller.ready_to_standup()

    controller.keep_pos()

    while True:
        try:
            controller.run(args.config_file)
        except KeyboardInterrupt:
            break
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
