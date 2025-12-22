#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET
import sys
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# 🚨 SET THIS TO THE EXACT PATH OF YOUR track.dae FILE
# Example: "/home/ubuntu/catkin_ws/src/my_pkg/meshes/track.dae"
DAE_FILE_PATH = "/home/shellsim/src/shell_car_model/track/meshes" 

# TRANSFORM SETTINGS
SCALE = 1.0          # Adjust if the track is too small/big (1.0 = original size)
Z_HEIGHT = 0.0       # Force path to this height (0 = ground)
FLATTEN_TO_2D = True # Ignors Z height from file and uses Z_HEIGHT instead
# ==============================================================================

def parse_dae_points(dae_path):
    """Parses the DAE file to extract geometry vertices."""
    points = []
    
    if not os.path.exists(dae_path):
        rospy.logerr("ERROR: Could not find file at: {}".format(dae_path))
        return []

    try:
        tree = ET.parse(dae_path)
        root = tree.getroot()
        
        # XML Namespace for Collada
        ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
        
        # Find the geometry mesh data (looking for float_array inside mesh)
        # This searches for ANY geometry in the file.
        float_arrays = root.findall(".//c:float_array", ns)
        
        if not float_arrays:
            rospy.logerr("ERROR: No geometry data found in DAE file.")
            return []

        # We assume the first float_array containing positions is the one we want
        # Typically positions are stored as X Y Z X Y Z ...
        raw_data = float_arrays[0].text.split()
        data = [float(x) for x in raw_data]
        
        # Group into triplets (x, y, z)
        for i in range(0, len(data), 3):
            if i + 2 < len(data):
                points.append((data[i], data[i+1], data[i+2]))
                
        rospy.loginfo("Successfully extracted {} points from DAE.".format(len(points)))
        return points

    except Exception as e:
        rospy.logerr("CRITICAL ERROR parsing DAE file: {}".format(e))
        return []

def publish_path():
    rospy.init_node('dae_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('track_path', Path, queue_size=10, latch=True)
    
    # Parse points from file
    raw_points = parse_dae_points(DAE_FILE_PATH)
    
    if not raw_points:
        rospy.logerr("No points to publish. Stopping.")
        return

    path_msg = Path()
    path_msg.header.frame_id = "map" 

    for i, (x, y, z) in enumerate(raw_points):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.frame_id = "map"
        
        # Apply scaling
        x = x * SCALE
        y = y * SCALE
        z = z * SCALE
        
        # Flatten to 2D if requested
        if FLATTEN_TO_2D:
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = Z_HEIGHT
        else:
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

        pose.pose.orientation.w = 1.0 # Identity rotation
        path_msg.poses.append(pose)

    # Publish
    rospy.loginfo("Publishing path...")
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass