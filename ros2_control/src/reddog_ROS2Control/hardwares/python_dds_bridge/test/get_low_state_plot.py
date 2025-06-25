import sys
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import numpy as np

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

    def LowStateMessageHandler(self, msg: LowState_):
        if msg is not None:
            self.low_state = msg


# === Data Buffers ===
N = 500

imu_acc_x, imu_acc_y, imu_acc_z = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
quat_w, quat_x, quat_y, quat_z = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
gyro_x, gyro_y, gyro_z = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
rpy_roll, rpy_pitch, rpy_yaw = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
q_history = [deque(maxlen=N) for _ in range(12)]

# === Plot Setup ===
fig, axs = plt.subplots(5, 1, figsize=(14, 12))

# Subplot 1: Accelerometer
line_acc_x, = axs[0].plot([], [], label='Acc X')
line_acc_y, = axs[0].plot([], [], label='Acc Y')
line_acc_z, = axs[0].plot([], [], label='Acc Z')
axs[0].set_title("IMU Accelerometer")
axs[0].set_ylim(-10, 10)
axs[0].set_xlim(0, N)
axs[0].legend()
axs[0].grid(True)

# Subplot 2: Quaternion
line_qw, = axs[1].plot([], [], label='w')
line_qx, = axs[1].plot([], [], label='x')
line_qy, = axs[1].plot([], [], label='y')
line_qz, = axs[1].plot([], [], label='z')
axs[1].set_title("IMU Quaternion")
axs[1].set_ylim(-1.1, 1.1)
axs[1].set_xlim(0, N)
axs[1].legend()
axs[1].grid(True)

# Subplot 3: Gyroscope
line_gyro_x, = axs[2].plot([], [], label='Gyro X')
line_gyro_y, = axs[2].plot([], [], label='Gyro Y')
line_gyro_z, = axs[2].plot([], [], label='Gyro Z')
axs[2].set_title("IMU Gyroscope")
axs[2].set_ylim(-3.0, 3.0)
axs[2].set_xlim(0, N)
axs[2].legend()
axs[2].grid(True)

# Subplot 4: RPY
line_rpy_r, = axs[3].plot([], [], label='Roll')
line_rpy_p, = axs[3].plot([], [], label='Pitch')
line_rpy_yaw, = axs[3].plot([], [], label='Yaw')
axs[3].set_title("IMU RPY")
axs[3].set_ylim(-3.0, 3.0)
axs[3].set_xlim(0, N)
axs[3].legend()
axs[3].grid(True)

# Subplot 5: Motor q
lines_q = [axs[4].plot([], [], label=f'Motor {i}')[0] for i in range(12)]
axs[4].set_title("Motor q (First 12 Motors)")
axs[4].set_ylim(-5.0, 5.0)
axs[4].set_xlim(0, N)
axs[4].legend(ncol=4)
axs[4].grid(True)

def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (qx * qz - qw * qy)
    gravity_orientation[1] = -2 * (qy * qz + qw * qx)
    gravity_orientation[2] = -1 - 2 * (qx**2 + qy**2)

    return gravity_orientation

# === Update Function ===
def update(frame):
    if dds_handler.low_state is not None:
        imu = dds_handler.low_state.imu_state
        motors = dds_handler.low_state.motor_state

        # imu_acc_x.append(imu.accelerometer[0])
        # imu_acc_y.append(imu.accelerometer[1])
        # imu_acc_z.append(imu.accelerometer[2])
        
        quat_w.append(imu.quaternion[0])
        quat_x.append(imu.quaternion[1])
        quat_y.append(imu.quaternion[2])
        quat_z.append(imu.quaternion[3])

        gravity_orientation = get_gravity_orientation([imu.quaternion[0], imu.quaternion[1],\
                                                        imu.quaternion[2],imu.quaternion[3]])
        imu_acc_x.append(gravity_orientation[0])
        imu_acc_y.append(gravity_orientation[1])
        imu_acc_z.append(gravity_orientation[2])

        gyro_x.append(imu.gyroscope[0])
        gyro_y.append(imu.gyroscope[1])
        gyro_z.append(imu.gyroscope[2])

        rpy_roll.append(imu.rpy[0])
        rpy_pitch.append(imu.rpy[1])
        rpy_yaw.append(imu.rpy[2])

        for i in range(12):
            q_history[i].append(motors[i].q)

        # 更新所有 plot
        line_acc_x.set_data(range(len(imu_acc_x)), imu_acc_x)
        line_acc_y.set_data(range(len(imu_acc_y)), imu_acc_y)
        line_acc_z.set_data(range(len(imu_acc_z)), imu_acc_z)

        line_qw.set_data(range(len(quat_w)), quat_w)
        line_qx.set_data(range(len(quat_x)), quat_x)
        line_qy.set_data(range(len(quat_y)), quat_y)
        line_qz.set_data(range(len(quat_z)), quat_z)

        line_gyro_x.set_data(range(len(gyro_x)), gyro_x)
        line_gyro_y.set_data(range(len(gyro_y)), gyro_y)
        line_gyro_z.set_data(range(len(gyro_z)), gyro_z)

        line_rpy_r.set_data(range(len(rpy_roll)), rpy_roll)
        line_rpy_p.set_data(range(len(rpy_pitch)), rpy_pitch)
        line_rpy_yaw.set_data(range(len(rpy_yaw)), rpy_yaw)

        for i in range(12):
            lines_q[i].set_data(range(len(q_history[i])), q_history[i])

    return [line_acc_x, line_acc_y, line_acc_z,
            line_qw, line_qx, line_qy, line_qz,
            line_gyro_x, line_gyro_y, line_gyro_z,
            line_rpy_r, line_rpy_p, line_rpy_yaw] + lines_q


# === Ctrl+T to Save Plot ===
def on_key(event):
    if event.key == 'ctrl+t':
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"plot_snapshot_{timestamp}.png"
        fig.savefig(filename)
        print(f"[Saved] Plot image saved as: {filename}")


# === Main ===
if __name__ == "__main__":
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
        dds_handler = DDSHandler()
    else:
        ChannelFactoryInitialize(0, sys.argv[1])
        dds_handler = DDSHandler(sys.argv[1])

    fig.canvas.mpl_connect("key_press_event", on_key)

    try:
        ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"plot_on_interrupt_{timestamp}.png"
        fig.savefig(filename)
        print(f"\n[Interrupted] Ctrl+C detected. Plot saved as: {filename}")
        sys.exit(0)


    
# LowState_(head=b'\x00\x00', level_flag=0, frame_reserve=0, sn=[0, 0], version=[0, 0], bandwidth=0, 
# imu_state=IMUState_(quaternion=[0.9996321201324463, 0.015511810779571533, -0.02163284830749035, 0.0051937163807451725], 
# gyroscope=[-0.0001742635213304311, -9.784360736375675e-05, -6.0489830502774566e-05], 
# accelerometer=[0.4258614182472229, 0.3020247519016266, 9.796096801757812], 
# rpy=[0.030821429565548897, -0.04342455416917801, 0.00972179975360632], temperature=0), 
# motor_state=[MotorState_(mode=0, q=-0.11337494850158691, dq=-0.00021185945661272854, ddq=0.0, tau_est=-0.15919378399848938, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.9452186226844788, dq=-0.0005204982589930296, ddq=0.0, tau_est=-1.7013518810272217, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=-1.7090243101119995, dq=-0.0002621781313791871, ddq=0.0, tau_est=0.36366739869117737, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.07650746405124664, dq=0.0005940292030572891, ddq=0.0, tau_est=-0.005187178496271372, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.8210528492927551, dq=1.0329009683118784e-06, ddq=0.0, tau_est=-0.2088882476091385, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=-1.6487224102020264, dq=-0.0003121480986010283, ddq=0.0, tau_est=0.6047272086143494, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=-0.1943359524011612, dq=-0.000360094360075891, ddq=0.0, tau_est=-0.28172004222869873, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=-0.7836742401123047, dq=0.00031377520645037293, ddq=0.0, tau_est=0.7830197811126709, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=1.767593264579773, dq=0.0002636394347064197, ddq=0.0, tau_est=-0.509963870048523, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.12369734048843384, dq=0.0009386098245158792, ddq=0.0, tau_est=0.4533087909221649, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=-0.9751432538032532, dq=0.000260710425209254, ddq=0.0, tau_est=1.25597083568573, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=1.7236597537994385, dq=0.00032439822098240256, ddq=0.0, tau_est=-0.668921709060669, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0])], bms_state=BmsState_(version_high=0, version_low=0, status=0, soc=0, current=0, cycle=0, bq_ntc=b'\x00\x00', mcu_ntc=b'\x00\x00', cell_vol=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), foot_force=[0, 0, 0, 0], foot_force_est=[0, 0, 0, 0], tick=0, wireless_remote=b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', bit_flag=0, adc_reel=0.0, temperature_ntc1=0, temperature_ntc2=0, power_v=0.0, power_a=0.0, fan_frequency=[0, 0, 0, 0], reserve=0, crc=0)

# LowState_(head=b'\x00\x00', level_flag=0, frame_reserve=0, sn=[0, 0], version=[0, 0], bandwidth=0, 
# imu_state=IMUState_(quaternion=[0.9999268054962158, 0.0012539775343611836, -0.011961858719587326, 0.001311571802943945], 
# gyroscope=[0.00012894677638541907, -0.002401430858299136, 6.284729897743091e-05], 
# accelerometer=[0.23471005260944366, 0.02429163083434105, 9.807119369506836], 
# rpy=[0.002477105474099517, -0.023927539587020874, 0.0025936970487236977], temperature=0), 
# motor_state=[MotorState_(mode=0, q=-0.040008485317230225, dq=-0.0022069658152759075, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=1.3361247777938843, dq=0.003181088250130415, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-2.7252697944641113, dq=-2.069205720545142e-06, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.032111700624227524, dq=0.0017918695230036974, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=1.334530234336853, dq=0.003135704668238759, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-2.725266933441162, dq=-3.024983243449242e-06, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-0.12651534378528595, dq=-0.010507806204259396, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-1.3283535242080688, dq=0.007631213869899511, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=2.7255630493164062, dq=3.140018452540971e-05, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.11855226755142212, dq=0.010214352048933506, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-1.3632495403289795, dq=0.0069922409020364285, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=2.7255263328552246, dq=2.9872029699617997e-05, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0])], 
# bms_state=BmsState_(version_high=0, version_low=0, status=0, soc=0, current=0, cycle=0, bq_ntc=b'\x00\x00', mcu_ntc=b'\x00\x00', cell_vol=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), foot_force=[0, 0, 0, 0], foot_force_est=[0, 0, 0, 0], tick=0, wireless_remote=b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', bit_flag=0, adc_reel=0.0, temperature_ntc1=0, temperature_ntc2=0, power_v=0.0, power_a=0.0, fan_frequency=[0, 0, 0, 0], reserve=0, crc=0)


# LowState_(head=b'\x00\x00', level_flag=0, frame_reserve=0, sn=[0, 0], version=[0, 0], bandwidth=0, 
#           imu_state=IMUState_(quaternion=[0.9116963744163513, -0.023538483306765556, -0.002827354008331895, -0.41017988324165344],
#                               gyroscope=[-0.014356017112731934, 0.006681561470031738, -0.0014181137084960938], 
#                               accelerometer=[0.20816612243652344, -0.2864503860473633, 9.766772270202637], 
#                               rpy=[0.0, 0.0, 0.0], temperature=0), 
#           motor_state=[
# MotorState_(mode=0, q=0.13866636157035828, dq=0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.7856488823890686, dq=0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-1.647783637046814, dq=0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-0.02689402550458908, dq=-0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.6837949156761169, dq=0.021978022530674934, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-1.7847332954406738, dq=0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0013351644156500697, dq=0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-0.6830319762229919, dq=-0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=1.7813000679016113, dq=-0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-0.135233074426651, dq=-0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=-0.7726787328720093, dq=0.007326007355004549, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=1.664568543434143, dq=-0.021978022530674934, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
# MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0])], 
# bms_state=BmsState_(version_high=0, version_low=0, status=0, soc=0, current=0, cycle=0, bq_ntc=b'\x00\x00', mcu_ntc=b'\x00\x00', cell_vol=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), foot_force=[0, 0, 0, 0], foot_force_est=[0, 0, 0, 0], tick=0, wireless_remote=b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', bit_flag=0, adc_reel=0.0, temperature_ntc1=0, temperature_ntc2=0, power_v=0.0, power_a=0.0, fan_frequency=[0, 0, 0, 0], reserve=0, crc=0)

# /imu/acceleration_hr
# ---
# header:
#   stamp:
#     sec: 1747829132
#     nanosec: 866844195
#   frame_id: imu_link
# vector:
#   x: 0.0774688720703125
#   y: -0.13129711151123047
#   z: 9.774410247802734
# ---

# ros2 topic echo /imu/angular_velocity_hr
# ---
# header:
#   stamp:
#     sec: 1747829163
#     nanosec: 181144195
#   frame_id: imu_link
# vector:
#   x: 0.006785273551940918
#   y: 0.0037497282028198242
#   z: 0.0021625757217407227
# ---

# ros2 topic echo /filter/quaternion
# ---
# header:
#   stamp:
#     sec: 1747829194
#     nanosec: 411344195
#   frame_id: imu_link
# quaternion:
#   x: -0.00674049323424697
#   y: -0.0037784304004162554
#   z: -0.030931644141674042
#   w: 0.9994917511940004
# ---

# position:
# - 0.042747583240270615
# - -0.13331672549247742
# - -0.13966602087020874
# - 0.16589407622814178
# - 0.6946954131126404
# - -0.563389241695404
# - -0.8099269866943359
# - 0.6965544819831848
# - -1.5012891292572021
# - 1.5138225555419922
# - 1.625341534614563
# - -1.4632474184036255
# velocity: []
# effort: []
