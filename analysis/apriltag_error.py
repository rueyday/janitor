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
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_24_26', 'Bag 2 (2m)', 2.0),
        ('/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_29_11', 'Bag 3 (3m)', 3.0),
    ]

    tf_topic = '/tf'
    target_frame = 'tag_1'
    start_time = 5.0
    end_time = 10.0

    all_data = []

    for path, label, gt_z in bags:
        print(f"\n🔍 Processing {label}...")
        df = extract_data_from_bag(path, tf_topic, target_frame, start_time, end_time, label, gt_z)
        if df.empty:
            print(f"⚠️ No data extracted from {label}. Skipping.")
        else:
            print(f"✅ Extracted {len(df)} points from {label}")
            all_data.append(df)

    if not all_data:
        print("❌ No valid data extracted from any bag.")
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

# def main():
#     # Initialize rclpy (required for message deserialization)
#     rclpy.init()
    
#     bag_path = '/home/ruey/ros2_ws/rosbag/apriltag/rosbag2_2025_07_25-15_16_05'
#     tf_topic = '/tf'  # or the actual topic used
    
#     # Create reader
#     reader = rosbag2_py.SequentialReader()
#     storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = rosbag2_py.ConverterOptions(
#         input_serialization_format='cdr', 
#         output_serialization_format='cdr'
#     )
    
#     reader.open(storage_options, converter_options)
    
#     # Get topic information
#     topic_types = reader.get_all_topics_and_types()
#     type_map = {t.name: t.type for t in topic_types}
    
#     # Set filter for the TF topic
#     reader.set_filter(rosbag2_py.StorageFilter(topics=[tf_topic]))
    
#     data = []
#     TARGET_FRAME = 'tag_1'  # Replace with actual child_frame_id you want to track
    
#     # Time window constraints (in seconds)
#     START_TIME = 5.0
#     END_TIME = 10.0
    
#     print(f"Processing rosbag data for target frame: {TARGET_FRAME}")
#     print(f"Looking for topic: {tf_topic}")
#     print(f"Time window: {START_TIME}s to {END_TIME}s")
    
#     # Check if the topic exists
#     if tf_topic not in type_map:
#         print(f"Error: Topic {tf_topic} not found in bag file!")
#         print("Available topics:")
#         for topic_info in topic_types:
#             print(f"  - {topic_info.name} ({topic_info.type})")
#         return
    
#     message_count = 0
#     matching_transforms = 0
#     filtered_transforms = 0
    
#     # Get the first timestamp to establish time baseline
#     first_timestamp = None
    
#     while reader.has_next():
#         topic, raw_data, timestamp = reader.read_next()
        
#         try:
#             # Deserialize the message
#             msg = deserialize_message(raw_data, TFMessage)
#             message_count += 1
            
#             # Process each transform in the TF message
#             for transform in msg.transforms:
#                 if transform.child_frame_id == TARGET_FRAME:
#                     matching_transforms += 1
                    
#                     # Convert timestamp
#                     time_sec = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    
#                     # Set baseline time from first message if not set
#                     if first_timestamp is None:
#                         first_timestamp = time_sec
#                         print(f"Baseline timestamp set to: {first_timestamp}")
                    
#                     # Calculate relative time from start of bag
#                     relative_time = time_sec - first_timestamp
                    
#                     # Filter by time window
#                     if START_TIME <= relative_time <= END_TIME:
#                         filtered_transforms += 1
                        
#                         # Extract position
#                         x_measured = transform.transform.translation.x
#                         y_measured = transform.transform.translation.y
#                         z_measured = transform.transform.translation.z
                        
#                         # Calculate error (assuming ground truth is x=1.0)
#                         error = z_measured - 1.0
                        
#                         data.append({
#                             'time': relative_time,
#                             'absolute_time': time_sec,
#                             'x_measured': x_measured,
#                             'y_measured': y_measured,
#                             'z_measured': z_measured,
#                             'error': error
#                         })
                    
#         except Exception as e:
#             print(f"Error processing message: {e}")
#             continue
    
#     print(f"Processed {message_count} TF messages")
#     print(f"Found {matching_transforms} transforms for target frame '{TARGET_FRAME}'")
#     print(f"Filtered to {filtered_transforms} transforms in time window {START_TIME}-{END_TIME}s")
    
#     if not data:
#         print(f"No data found for target frame '{TARGET_FRAME}' in time window {START_TIME}-{END_TIME}s!")
#         print("Make sure the frame name is correct and the time window contains data.")
#         if matching_transforms > 0:
#             print(f"Note: Found {matching_transforms} total transforms, but none in specified time window.")
#         return
    
#     # Create DataFrame
#     df = pd.DataFrame(data)
    
#     # Calculate statistics
#     print(f"\nStatistics for {len(df)} measurements (time window {START_TIME}-{END_TIME}s):")
#     print(f"Time range: {df['time'].min():.3f}s to {df['time'].max():.3f}s")
#     print(f"Mean position: {df['z_measured'].mean():.6f} meters")
#     print(f"Mean error: {df['error'].mean():.6f} meters")
#     print(f"Std deviation: {df['error'].std():.6f} meters")
#     print(f"Min error: {df['error'].min():.6f} meters")
#     print(f"Max error: {df['error'].max():.6f} meters")
    
#     # Create plots
#     fig, ax2 = plt.subplots(1, 1, figsize=(12, 8))
    
#     ax2.plot(df['time'], df['error'], 'r-', alpha=0.7, label='Real Error')
#     ax2.axhline(0.0, color='black', linestyle='--', alpha=0.5, label='Zero Error')
#     ax2.set_xlabel('Time (s)')
#     ax2.set_ylabel('Error (m)')
#     ax2.set_title(f'Coordinate Error Over Time ({START_TIME}-{END_TIME}s)')
#     ax2.grid(True, alpha=0.3)
#     ax2.legend()
    
#     plt.tight_layout()
#     plt.show()
    
#     # Save data to CSV for further analysis
#     output_file = f'apriltag_analysis_{START_TIME}s-{END_TIME}s.csv'
#     df.to_csv(output_file, index=False)
#     print(f"\nFiltered data ({START_TIME}-{END_TIME}s) saved to {output_file}")
    
#     # Shutdown rclpy
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()