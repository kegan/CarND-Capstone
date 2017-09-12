#!/usr/bin/env python

import rospy

import tf
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from styx_msgs.msg import Lane, Waypoint

import math
from collections import namedtuple

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

Rotations = namedtuple("Rotations", ['roll', 'pitch', 'yaw'])

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.waypoints = None

        self.publish()
        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    """
    Get rotations on roll, pitch and yaw axes (global) from quaternion
    """
    def rotations_from_quaternion(self, q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[:3]
        return Rotations(roll, pitch, yaw)

    """
    Get rotations on roll, pitch and yaw axes (global) from vector
    """
    def rotations_from_vector(self, vector):
        x, y, z = vector[:3]
        roll = 0
        pitch = math.atan2(z, math.sqrt(x ** 2 + y ** 2))
        yaw = math.atan2(y, x)
        return Rotations(roll, pitch, yaw)

    """
    Get distance between position A and position B
    """
    def get_distance(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2  + (a.z - b.z) ** 2)

    """
    Get vector from position A to position B
    """
    def get_vector(self, a, b):
        return [b.x - a.x, b.y - a.y, b.z - a.z]

    """
    Absolute difference between 2 angles
    """
    def angles_delta(self, angle1, angle2, unit='radian'):
        if unit == 'radian':
            return math.pi - math.fabs(math.fabs(angle1 - angle2) - math.pi)
        else:
            return 180 - math.fabs(math.fabs(angle1 - angle2) - 180)

    """
    Get the waypoint closest to the pose
    """
    def get_closest_waypoint(self, position, waypoints):
        dl = lambda wp: self.get_distance(wp.pose.pose.position, position)
        waypoint_i = waypoints.index(min(waypoints, key=dl))
        return waypoint_i

    """
    Compute vectors for forward and backward direction on the waypoint
    """
    def get_fwd_bwd_vectors(self, waypoint_i, waypoints):
        get_position = lambda wp: wp.pose.pose.position

        if waypoint_i == 0:
            vector_fwd = self.get_vector(get_position(waypoints[0]), get_position(waypoints[1]))
            vector_bwd = self.get_vector(get_position(waypoints[1]), get_position(waypoints[0]))
        elif waypoint_i == len(waypoints) - 1:
            vector_fwd = self.get_vector(get_position(waypoints[-2]), get_position(waypoints[-1]))
            vector_bwd = self.get_vector(get_position(waypoints[-1]), get_position(waypoints[-2]))
        else:
            vector_fwd = self.get_vector(get_position(waypoints[waypoint_i]), get_position(waypoints[waypoint_i + 1]))
            vector_bwd = self.get_vector(get_position(waypoints[waypoint_i]), get_position(waypoints[waypoint_i - 1]))
        return vector_fwd, vector_bwd

    """
    Look ahead waypoints in the direction based on the vehicle current heading
    """
    def look_ahead_waypoints(self, pose, waypoints, count=LOOKAHEAD_WPS):
        waypoint_i = self.get_closest_waypoint(pose.position, waypoints)

        vector_fwd, vector_bwd = self.get_fwd_bwd_vectors(waypoint_i, waypoints)

        fwd_yaw = self.rotations_from_vector(vector_fwd).yaw * 180 / math.pi
        bwd_yaw = self.rotations_from_vector(vector_bwd).yaw * 180 / math.pi
        current_yaw = self.rotations_from_quaternion(pose.orientation).yaw * 180 / math.pi

        fwd_yaw_diff = self.angles_delta(fwd_yaw, current_yaw)
        bwd_yaw_diff = self.angles_delta(bwd_yaw, current_yaw)

        fwd = True
        if bwd_yaw_diff < fwd_yaw_diff:
            waypoints = waypoints[::-1]
            waypoint_i = len(waypoints) - waypoint_i - 1
            fwd = False

        #rospy.logout('waypoint_i: %d, yaw %f, fwd_yaw %f, bwd_yaw %f, %s', waypoint_i, current_yaw, fwd_yaw, bwd_yaw, 'Forward'  if fwd else 'Backward')

        # Loop to start if waypoints are depleted
        waypoints_ahead = waypoints[waypoint_i:waypoint_i + count]
        while len(waypoints_ahead) < count:
            waypoints_ahead.extend(waypoints[:count - len(waypoints_ahead)])

        return waypoints_ahead

    """
    Only look ahead waypoints in the forward direction
    """
    def look_ahead_waypoints2(self, pose, waypoints, count=LOOKAHEAD_WPS):
        waypoint_i = self.get_closest_waypoint(pose.position, waypoints)

        if waypoint_i == len(waypoints) - 1:
            return [waypoints[waypoint_i]]

        current_rotations = self.rotations_from_quaternion(pose.orientation)
        current_yaw = current_rotations.yaw

        vector = self.get_vector(pose.position, waypoints[waypoint_i].pose.pose.position)
        new_rotations = self.rotations_from_vector(vector)
        new_yaw = new_rotations.yaw

        delta = self.angles_delta(current_yaw, new_yaw)

        vector_fwd = self.get_vector(waypoints[waypoint_i].pose.pose.position, waypoints[waypoint_i + 1].pose.pose.position)
        proposed_yaw = self.rotations_from_vector(vector_fwd).yaw

        if delta > math.pi / 4:
            waypoint_i = waypoint_i + 1

        #rospy.logout('waypoint_i: %d, current_yaw %f, new_yaw %f, proposed_yaw %f', waypoint_i, current_yaw, new_yaw, proposed_yaw)
        #rospy.logout('waypoint_i: %d, current_yaw %f, new_yaw %f', waypoint_i, current_yaw, new_yaw)

        # Loop to start if waypoints are depleted
        waypoints_ahead = waypoints[waypoint_i:waypoint_i + count]
        while len(waypoints_ahead) < count:
            waypoints_ahead.extend(waypoints[:count - len(waypoints_ahead)])

        return waypoints_ahead

    def get_next_waypoints(self):
        if not self.current_pose or not self.waypoints:
            return None

        #waypoints_ahead = self.look_ahead_waypoints(self.current_pose, self.waypoints)
        waypoints_ahead = self.look_ahead_waypoints2(self.current_pose, self.waypoints)

        next_waypoints = waypoints_ahead

        return next_waypoints

    def publish(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            next_waypoints = self.get_next_waypoints()
            if next_waypoints:
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = next_waypoints
                self.final_waypoints_pub.publish(lane)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
