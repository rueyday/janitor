import rclpy
import rosbag2_py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

def extract_data_from_bag(bag_path, tf_topic, target_frame, start_time, end_time, label, ground_truth_z):
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
                        z_measured = transform.transform.translation.z
                        error = z_measured - ground_truth_z

                        data.append({
                            'time': relative_time,
                            'measured': z_measured,
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
        # (bag_path, label, ground_truth_z)
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_16_05', 'Bag 1 (1m)', 1.0),
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_24_26', 'Bag 2 (2m)', 1.98),
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_29_11', 'Bag 3 (3m)', 2.96),
    ]

    tf_topic = '/tf'
    target_frame = 'tag_1'
    start_time = 5.0
    end_time = 10.0

    all_data = []

    for path, label, gt_z in bags:
        print(f"\n🔍 Processing {label}...")
        print(f"Time window: {start_time}s to {end_time}s")
        df = extract_data_from_bag(path, tf_topic, target_frame, start_time, end_time, label, gt_z)
        if df.empty:
            print(f"No data extracted from {label}. Skipping.")
        else:
            print(f"\nStatistics for {len(df)} measurements (time window {start_time}s to {end_time}s):")
            print(f"Time range: {df['time'].min():.3f}s to {df['time'].max():.3f}s")
            print(f"Mean position: {df['measured'].mean():.6f} meters")
            print(f"Mean error: {df['error'].mean():.6f} meters")
            print(f"Std deviation: {df['error'].std():.6f} meters")
            print(f"Min error: {df['error'].min():.6f} meters")
            print(f"Max error: {df['error'].max():.6f} meters")
            all_data.append(df)


    if not all_data:
        print("No valid data extracted from any bag.")
        rclpy.shutdown()
        return

    combined_df = pd.concat(all_data, ignore_index=True)

    # Plot error
    plt.figure(figsize=(12, 8))
    for label in combined_df['label'].unique():
        df_label = combined_df[combined_df['label'] == label]
        plt.plot(df_label['time'], df_label['error'], label=label, alpha=0.7)

    plt.axhline(0.0, color='black', linestyle='--', alpha=0.5, label='Zero Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (Measured - Ground Truth) [m]')
    plt.title(f'AprilTag Error Over Time ({start_time}-{end_time}s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
