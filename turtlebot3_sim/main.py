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

    bagi = ['trajectory_bag_2.53_0.67_param4_8_1_2','trajectory_bag_2.53_0.67_param4_4_1_2','trajectory_bag_2.53_0.67_param4_4_10_2',
            'trajectory_bag_2.53_0.67_param4_4_1_20']
    for bags in bagi:
        x1_coords = []
        y1_coords = []
        label = '_'.join(bags.split('_')[-3:])
        # Iterate over all messages from the bag file
        for topic, msg, timestamp in read_messages('trajectory/'+bags):
            # Assuming msg has pose.position.x and pose.position.y
            x1_coords.append(msg.pose.position.x)
            y1_coords.append(msg.pose.position.y)
        # Iterate over all messages from the bag file
    
    # Plot the trajectory
        plt.plot(x1_coords, y1_coords,label=label)
    plt.legend()
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Trajectory x:2.53 y:0.67 ThetaStarPlanner')
    plt.grid(True)
    plt.savefig('Robot Trajectory x:2.53 y:0.67 ThetaStarPlanner.png')
    plt.show()

if __name__ == "__main__":
    main()