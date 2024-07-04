from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import pandas as pd
import sys


def get_lidar_feature_labels():
    # Label each feature for readability
    frame_id_label = "frame_id"
    timestamp_labels = ["time_sec", "time_nanosec"]
    data_labels = [
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
        "ranges",
        "intensities",
    ]
    return [frame_id_label, *timestamp_labels, *data_labels]


def get_odom_feature_labels():
    # Label each feature for readability
    frame_id_label = "frame_id"
    timestamp_labels = ["time_sec", "time_nanosec"]

    position_labels = ["position_x", "position_y", "position_z"]
    orientation_labels = [
        "orientation_x",
        "orientation_y",
        "orientation_z",
        "orientation_w",
    ]

    linear_vel_labels = ["velocity_x", "velocity_y", "velocity_z"]
    angular_vel_labels = ["omega_x", "omega_y", "omega_z"]

    # cov_sublabels = ["x", "y", "z", "xrot", "yrot", "zrot"]
    # pos_cov_labels = [
    #     i + "_" + j + "_POS" for i in cov_sublabels for j in cov_sublabels
    # ]
    # vel_cov_labels = [
    #     i + "_" + j + "_VEL" for i in cov_sublabels for j in cov_sublabels
    # ]

    return [
        frame_id_label,
        *timestamp_labels,
        *position_labels,
        *linear_vel_labels,
        *orientation_labels,
        *angular_vel_labels,
        "covariance_pos",
        "covariance_vel",
    ]


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Error: Invalid input.")
        print("Expected: python rosbag_to_csv.py <rosbag_filepath>")
        sys.exit(-1)

    # Use this filepath
    rosbag_filepath = sys.argv[1]

    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)
    # Create reader instance and open for reading.
    with Reader(rosbag_filepath) as reader:
        # Topic and msgtype information is available on .connections list.
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)

        # Topics we want to record
        lidar_dataset: list[list] = list(list())
        odom_dataset: list[list] = list(list())

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/scan":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                # Get the desired feature information
                frame_id = msg.header.frame_id
                timestamp = msg.header.stamp  # fields = sec, nanosec

                ranges = msg.ranges
                data_point = [
                    frame_id,
                    timestamp.sec,
                    timestamp.nanosec,
                    msg.angle_min,
                    msg.angle_max,
                    msg.angle_increment,
                    msg.time_increment,
                    msg.scan_time,
                    msg.range_min,
                    msg.range_max,
                    msg.ranges,
                    msg.intensities,
                ]

                lidar_dataset.append(data_point)

            if connection.topic == "/odom":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                # Get the desired feature information
                frame_id = msg.header.frame_id
                timestamp = msg.header.stamp  # fields = sec, nanosec

                position = msg.pose.pose.position  # fields = x, y, z
                linear_vel = msg.twist.twist.linear  # fields = x, y, z

                orientation = msg.pose.pose.orientation  # fields = x, y, z, w
                angular_vel = msg.twist.twist.angular  # fields = x, y, z

                pos_cov = (
                    msg.pose.covariance  # fields = (x, y, z, rot_x, rot_y, rot_z) -> array of 36 entries
                )
                vel_cov = (
                    msg.twist.covariance  # fields = (x, y, z, rot_x, rot_y, rot_z) -> array of 36 entries
                )

                # Create data point
                data_point = [
                    frame_id,
                    timestamp.sec,
                    timestamp.nanosec,
                    position.x,
                    position.y,
                    position.z,
                    linear_vel.x,
                    linear_vel.y,
                    linear_vel.z,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                    angular_vel.x,
                    angular_vel.y,
                    angular_vel.z,
                    pos_cov,
                    vel_cov,
                ]

                # Add data point to the dataframe
                odom_dataset.append(data_point)

        # Create pandas dataframes
        lidar_df = pd.DataFrame(lidar_dataset, columns=get_lidar_feature_labels())
        # print(lidar_df)
        lidar_df.to_csv(str(rosbag_filepath).strip("/") + "_lidar.csv")

        odom_df = pd.DataFrame(odom_dataset, columns=get_odom_feature_labels())
        # print(odom_df)
        odom_df.to_csv(str(rosbag_filepath).strip("/") + "_odom.csv")
