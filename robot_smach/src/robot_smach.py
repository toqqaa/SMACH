#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import actionlib
from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseArray
import tf

# Global variable to store the waypoint pose
waypoint_pose = None

# Define the home position for the robot
home_pose = PoseStamped()
home_pose.header.frame_id = "map"
home_pose.pose.position.x = 0.0
home_pose.pose.position.y = 0.0
home_pose.pose.position.z = 0.0
home_pose.pose.orientation.w = 1.0

# Function to convert yaw angle to quaternion
def yaw_to_quaternion(yaw):
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return q

# Callback to receive waypoints from the /waypoints topic
def waypoint_callback(msg):
    global waypoint_pose
    if len(msg.poses) > 0:
        waypoint_pose = PoseStamped()
        waypoint_pose.header = msg.header  # Set header from the message
        waypoint_pose.pose = msg.poses[0]  # Take the first pose in PoseArray
        rospy.loginfo(f"Received waypoint at x={waypoint_pose.pose.position.x}, y={waypoint_pose.pose.position.y}")
    else:
        rospy.logwarn("Received empty PoseArray message.")

# State: Wait for a Waypoint
class WaitForWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['waypoint_received'])
    
    def execute(self, userdata):
        global waypoint_pose
        rospy.loginfo("Waiting for waypoint...")
        rate = rospy.Rate(1)  # 1 Hz
        while waypoint_pose is None:  # Wait until a waypoint is received
            rate.sleep()
        rospy.loginfo("Waypoint received. Proceeding to MOVE_TO_WAYPOINT state.")
        return 'waypoint_received'

# State: Move to Waypoint
class MoveToWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['arrived', 'not_arrived'])
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

    def execute(self, userdata):
        global waypoint_pose

        if waypoint_pose is None:
            rospy.logerr("No waypoint available!")
            return 'not_arrived'

        # Send the move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header = waypoint_pose.header
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint_pose.pose

        rospy.loginfo(f"Moving to waypoint at x={waypoint_pose.pose.position.x}, y={waypoint_pose.pose.position.y}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at waypoint!")
            return 'arrived'
        else:
            rospy.logerr(f"Failed to reach waypoint. State: {state}")
            return 'not_arrived'

# State: Wait at Waypoint
class WaitAtWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("Waiting at waypoint for 2 minutes...")
        rospy.sleep(120)  # Wait for 2 minutes
        return 'done'

# State: Return to Home
class ReturnToHome(State):
    def __init__(self):
        State.__init__(self, outcomes=['arrived_home', 'not_arrived'])
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        global home_pose

        # Prepare and send the move_base goal to return home
        goal = MoveBaseGoal()
        goal.target_pose.header = home_pose.header
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = home_pose.pose

        rospy.loginfo("Returning to home ...")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at home position!")
            return 'arrived_home'
        else:
            rospy.logerr(f"Failed to return to home position. State: {state}")
            return 'not_arrived'

# State: Reset Waypoint
class ResetWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['reset_done'])
    
    def execute(self, userdata):
        global waypoint_pose
        rospy.loginfo("Resetting waypoint...")
        waypoint_pose = None  
        return 'reset_done'

def main():
    global home_pose

    rospy.init_node('robot_smach_state_machine')

    # Subscribe to the /waypoints topic to receive waypoints
    rospy.Subscriber('/waypoints', PoseArray, waypoint_callback)

    home_pose.header.stamp = rospy.Time.now()

    sm = smach.StateMachine(outcomes=['failed'])

    with sm:
        smach.StateMachine.add('WAIT_FOR_WAYPOINT', WaitForWaypoint(),
                               transitions={'waypoint_received': 'MOVE_TO_WAYPOINT'})

        smach.StateMachine.add('MOVE_TO_WAYPOINT', MoveToWaypoint(),
                               transitions={'arrived': 'WAIT_AT_WAYPOINT', 'not_arrived': 'failed'})

        smach.StateMachine.add('WAIT_AT_WAYPOINT', WaitAtWaypoint(),
                               transitions={'done': 'RETURN_TO_HOME'})

        smach.StateMachine.add('RETURN_TO_HOME', ReturnToHome(),
                               transitions={'arrived_home': 'RESET_WAYPOINT', 'not_arrived': 'failed'})

        smach.StateMachine.add('RESET_WAYPOINT', ResetWaypoint(),
                               transitions={'reset_done': 'WAIT_FOR_WAYPOINT'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_GoToWaypoint')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
