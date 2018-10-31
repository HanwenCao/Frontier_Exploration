import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

#Wait for the availability of this service
rospy.wait_for_service('/move_base/make_plan')
#Get a proxy to execute the service
make_plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

# set start and goal
start = PoseStamped()
start.header.frame_id = "map"
start.pose = self.robot_pose  #msg.pose.pose
goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose = eachCandidate
# set tolerance
tolerance = 0.0

try:
    plan_response = make_plan(start=start, goal=goal, tolerance=tolerance)
except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))



plan_response.plan
plan_response.plan.poses
