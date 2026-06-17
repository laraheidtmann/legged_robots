import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py

def read_pose_topic(bag_path, topic_name):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    times, xs, ys = [], [], []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            times.append(t * 1e-9)  # ns -> s
            xs.append(msg.point.x)
            ys.append(msg.point.y)

    return np.array(times), np.array(xs), np.array(ys)


#bag_path = "/home/ubuntu/legged_robots/rosbag2_2026_06_12-14_47_00"  # the folder, not the .db3 file
#bag_path= "rosbag2_2026_06_12-14_47_00"
#bag_path="rosbag2_2026_06_12-15_03_36"
bag_path="rosbags/no_control_position"
t_zmp, x_zmp, y_zmp = read_pose_topic(bag_path, "/zmp")
t_cmp, x_cmp, y_cmp = read_pose_topic(bag_path, "/cmp")
t_cp,  x_cp,  y_cp  = read_pose_topic(bag_path, "/cp")
t_com, x_com, y_com = read_pose_topic(bag_path, "/com")

# normalize time to start at 0
t0 = t_zmp[0]
t_zmp -= t0; t_cmp -= t0; t_cp -= t0; t_com -= t0

fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

axes[0].plot(t_com, x_com, label="CoM x")
axes[0].plot(t_zmp, x_zmp, label="ZMP x")
axes[0].plot(t_cmp, x_cmp, label="CMP x")
axes[0].plot(t_cp,  x_cp,  label="CP/DCM x")
axes[0].set_ylabel("x [m]")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t_com, y_com, label="CoM y")
axes[1].plot(t_zmp, y_zmp, label="ZMP y")
axes[1].plot(t_cmp, y_cmp, label="CMP y")
axes[1].plot(t_cp,  y_cp,  label="CP/DCM y")
axes[1].set_xlabel("time [s]")
axes[1].set_ylabel("y [m]")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig("ground_reference_points.png")
plt.show()