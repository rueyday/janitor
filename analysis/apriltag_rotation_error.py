import rclpy
import rosbag2_py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion  # <-- requires tf-transformations

def extract_data_from_bag(bag_path, tf_topic, target_frame, start_time, end_time, label, ground_truth_roll):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if tf_topic not in type_map:
        print(f"Topic {tf_topic} not found in bag: {bag_path}")
        return pd.DataFrame()

    reader.set_filter(rosbag2_py.StorageFilter(topics=[tf_topic]))

    data = []
    first_timestamp = None

    while reader.has_next():
        topic, raw_data, timestamp = reader.read_next()

        try:
            msg = deserialize_message(raw_data, TFMessage)

            for transform in msg.transforms:
                if transform.child_frame_id == target_frame:
                    time_sec = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9

                    if first_timestamp is None:
                        first_timestamp = time_sec

                    relative_time = time_sec - first_timestamp

                    if start_time <= relative_time <= end_time:
                        q = transform.transform.rotation
                        quaternion = [q.x, q.y, q.z, q.w]

                        # Extract roll (rotation around x-axis)
                        roll, pitch, yaw = euler_from_quaternion(quaternion)
                        error = roll - ground_truth_roll  # Assuming ground truth is 0.0 unless otherwise specified

                        data.append({
                            'time': relative_time,
                            'measured': roll,
                            'error': error,
                            'label': label
                        })

        except Exception as e:
            print(f"Error processing message: {e}")
            continue

    return pd.DataFrame(data)

def main():
    rclpy.init()

    bags = [
        # (bag_path, label, ground_truth_roll in radians)
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_24_26', 'Bag 3 (0 degrees)', 0.0),
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_25_47', 'Bag 3 (30 degrees)', 0.5235),
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_30_47', 'Bag 3 (60 degrees)', 1.0472),
    ]

    tf_topic = '/tf'
    target_frame = 'tag_1'
    start_time = 5.0
    end_time = 10.0

    all_data = []

    for path, label, gt_roll in bags:
        print(f"\n🔍 Processing {label}...")
        print(f"Time window: {start_time}s to {end_time}s")
        df = extract_data_from_bag(path, tf_topic, target_frame, start_time, end_time, label, gt_roll)
        if df.empty:
            print(f"No data extracted from {label}. Skipping.")
        else:
            print(f"\nStatistics for {len(df)} measurements (time window {start_time}s to {end_time}s):")
            print(f"Time range: {df['time'].min():.3f}s to {df['time'].max():.3f}s")
            print(f"Mean roll: {np.degrees(df['measured'].mean()):.6f}°")
            print(f"Mean error: {np.degrees(df['error'].mean()):.6f}°")
            print(f"Std deviation: {np.degrees(df['error'].std()):.6f}°")
            print(f"Min error: {np.degrees(df['error'].min()):.6f}°")
            print(f"Max error: {np.degrees(df['error'].max()):.6f}°")
            all_data.append(df)

    if not all_data:
        print("No valid data extracted from any bag.")
        rclpy.shutdown()
        return

    combined_df = pd.concat(all_data, ignore_index=True)

    # Plot roll error
    plt.figure(figsize=(12, 8))
    for label in combined_df['label'].unique():
        df_label = combined_df[combined_df['label'] == label]
        plt.plot(df_label['time'], np.degrees(df_label['error']), label=label, alpha=0.7)

    plt.axhline(0.0, color='black', linestyle='--', alpha=0.5, label='Zero Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Roll Error (degrees)')
    plt.title(f'AprilTag Rotation (Roll) Error Over Time ({start_time}-{end_time}s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
