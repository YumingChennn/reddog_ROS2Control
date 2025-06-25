from controllerV3_him_imu import MotorController
import rclpy

def main(args=None):
    rclpy.init()

    controller = MotorController()
    
    controller.move_to_default_pos()

    controller.ready_to_standup()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()