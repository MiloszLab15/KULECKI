import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import rosbag2_py
import matplotlib.pyplot as plt

def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader

def main():
    x_coords = []
    y_coords = []
    
    # Iterate over all messages from the bag file
    for topic, msg, timestamp in read_messages('trajectory_bag_20250413_140420'):
        # Assuming msg has pose.position.x and pose.position.y
        x_coords.append(msg.pose.position.x)
        y_coords.append(msg.pose.position.y)
    
    # Plot the trajectory
    plt.plot(x_coords, y_coords)
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()