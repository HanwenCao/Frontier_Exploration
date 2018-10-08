#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import String
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import numpy as np
import cv2
import tf.transformations as transformations
import math
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
import tf2_ros
from gazebo_msgs.srv import SetModelState
import sys
from kobuki_msgs.msg import ButtonEvent
from sensor_msgs.msg import Imu
import numpy as np
import actionlib
import move_base_msgs.msg as move_base_msgs
from pprint import pprint
import tf
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, WheelDropEvent
import math
import math
import numpy as np
#import dippykit as dip
from skimage import measure
from skimage.measure import label, regionprops

class DemoResetter():
    def __init__(self):
        rospy.init_node('Prototype')
        # self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        #self.clear_costmap_srv = None
        # self.publishMap()
        #self.pub = rospy.Publisher("/mobile_base/commands/reset_odometry", std_msgs.Empty, queue_size=1)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.MoveBaseAction)
        print "waiting for server"
        self.client.wait_for_server()
        print "Done!"
        self.stop = False
        self.flag_stuck=0
        # rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.ButtonEventCallback)
        # rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.WheelDropEventCallback)
        self.initialGoals()
        self.navigate()
        
        rospy.spin()

    def callback_map(self, OccupancyGrid):
        info = OccupancyGrid.info
        data = OccupancyGrid.data
        rospy.loginfo(rospy.get_caller_id() + 'I heard the map')
        rawmap = np.array(data)
        self.rawmap = rawmap.reshape(info.height, info.width)
        np.savetxt("rawmap.txt", self.rawmap, fmt='%d');
        self.p0 = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z, info.resolution])
        # p0 is the origin and resolution of the map
        # p0 = p0.reshape([4,1])
        np.savetxt("info.txt", self.p0, delimiter=' ')

    def callback_odom(self, Odometry):
        position_x = Odometry.pose.pose.position.x
        position_y = Odometry.pose.pose.position.y
        position_z = Odometry.pose.pose.position.z
        orientation_z = Odometry.pose.pose.orientation.z
        orientation_w = Odometry.pose.pose.orientation.w
        #rospy.loginfo(rospy.get_caller_id() + 'I heard odom')
        #print 'odom x: %s' % position_x
        #print 'odom y: %s' % position_y
        #print 'odom orientation: %s' % orientation
        self.odom = np.array([position_x, position_y, orientation_z, orientation_w])
        np.savetxt("odom.txt", self.odom, delimiter=' ')

    def initialGoals(self):	
        self.goals = []
        goalA = Pose()
        goalA.position.x = 1000
        goalA.position.y = 1000
        goalA.orientation.w = 1
        goalB = Pose()
        goalB.position.x = 0
        goalB.position.y = 0
        goalB.orientation.w = 1
        self.goals.append(goalA)
        self.goals.append(goalB)
        print "initial goals!"

    def setupGoals(self, next_x, next_y):
	if abs(self.goals[1].position.x - next_x)<0.1 and abs(self.goals[1].position.y - next_y)<0.1:
		print "Almost same to last goal, may get stuck"
                self.flag_stuck = 1
        goalB = Pose()   # goalB is next frontier
        goalB.position.x = next_x
        goalB.position.y = next_y
        goalB.orientation.w = 1
        self.goals[1] = goalB 
        print "exploration goal is set!"



    def navigate(self):

        #get orientation
        print "rotate to get a whole view1"
        orientation_z = self.odom[2] #read current orientation
        orientation_w = self.odom[3]
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]    #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = orientation_w
        goal_rot.orientation.w = 1-orientation_w
        self.navigateToGoal(goal_pose=goal_rot)    # rotate 180'
        print "rotate to get a whole view2"
        #get orientation
        orientation_z = self.odom[2] #read current orientation
        orientation_w = self.odom[3]
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]  #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = orientation_w
        goal_rot.orientation.w = 1-orientation_w
        self.navigateToGoal(goal_pose=goal_rot)    # rotate 180'
        

        while not rospy.is_shutdown():   

            if not self.stop:

                try:
                    print "rotate to get a whole view3"
                    #get orientation
                    orientation_z = self.odom[2] #read current orientation
                    orientation_w = self.odom[3]
                    #set a rotation goal
                    goal_rot = Pose()
                    goal_rot.position.x = self.odom[0]  #read current odom
                    goal_rot.position.y = self.odom[1]
                    goal_rot.orientation.z = orientation_w
                    goal_rot.orientation.w = 1-orientation_w
                    self.navigateToGoal(goal_pose=goal_rot)    # rotate 180'


                    next_goal = self.process(info=self.p0, grid=self.rawmap, odom=self.odom[0:2])
                   

                    self.setupGoals(next_x=next_goal[0],next_y=next_goal[1])
		    
                    self.navigateToGoal(goal_pose=self.goals[1])  # go to next frontier
                    print "go to: ", self.goals[1]                    
                    # self.resetCostmaps()



                except Exception, e:

                    print e

                    pass

                    rospy.sleep(.1)

            else:

                rospy.sleep(.2)





    def navigateToGoal(self, goal_pose):



        # Create the goal point

        goal = move_base_msgs.MoveBaseGoal()

        goal.target_pose.pose = goal_pose

        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.header.frame_id = "odom"



        # Send the goal!

        print "sending goal"

        self.client.send_goal(goal)

        print "waiting for result"



        r = rospy.Rate(5)



        start_time = rospy.Time.now()



        keep_waiting = True

        while keep_waiting:

            state = self.client.get_state()

            #print "State: " + str(state)

            if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:

                keep_waiting = False

            else:

                r.sleep()



        state = self.client.get_state()

        if state == GoalStatus.SUCCEEDED:

            return True



        return True



    def process_test(self, info, grid, odom):
        next_goal = odom
        next_goal[0] = next_goal[0]+0.2
        next_goal[1] = next_goal[1]+0.2
        return next_goal



    def process(self, info, grid, odom):


        print "processing!"
        resolution = info[3]

        origin_x = info[0]

        origin_y = info[1]

        # origin_z = info[2]

        size = grid.shape

        robot_pose_world = odom  # robot initial position in world

        robot_pose_pixel = [0,0]  # robot initial position in grid (pixel in image)

        robot_pose_pixel[0] = (robot_pose_world[0] - origin_x) / resolution

        robot_pose_pixel[1] = (robot_pose_world[1] - origin_y) / resolution



        # --------------------------------------------- open cells ---------------------



        thresh_low = 0

        thresh_high = 10

        dst = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0  # threshold
        dst_open = dst
        #np.savetxt("dst_open.txt", dst_open, fmt='%d');
        # "1" in dst_open are the open cells

        # "0" in dst_open are the unvisited cells and occupied cells



        # detect contours

        contours_open = measure.find_contours(dst, 0.5)

        contours_open_cell = list()

        for ele in contours_open:

            for cell in ele:

                contours_open_cell.append(cell.tolist())

        # print(contours_open_cell)



        # --------------------------------------------- unvisited cells ---------------------

        thresh_low = -1

        thresh_high = -1

        dst = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0  # threshold

        # "1" in dst are the unvisited cells

        # "0" in dst are the open cells and occupied cells



        # find contours

        contours_unvisited = measure.find_contours(dst, 0.5)

        contours_unvisited_cell = list()

        for ele in contours_unvisited:

            for cell in ele:

                contours_unvisited_cell.append(cell.tolist())

        # print(contours_unvisited_cell)



        # ----------------------------------------------------------------

        # frontier detection ! ! !

        frontier_cells = [x for x in contours_unvisited_cell if x in contours_open_cell]

        # print('frontier: ')

        # print(frontier_cells)  # to find the same elements in both lists



        grid_frontier = np.zeros(size)

        for ele in frontier_cells:

            grid_frontier[math.floor(ele[0]), math.floor(ele[1])] = 1



        # group them!

        #grid_frontier_img = dip.float_to_im(grid_frontier)
        #conected_frontier, label_num = measure.label(grid_frontier_img, return_num=True, connectivity=2)

        #conected_frontier, label_num = measure.label(grid_frontier, return_num=True, connectivity=2)
        conected_frontier, label_num = measure.label(grid_frontier, return_num=True)

        #print("num of frontiers: %d" % label_num)

        #conected_frontier = dip.float_to_im(conected_frontier / label_num)
        #conected_frontier = (conected_frontier / label_num) * 255


        # delete small frontiers

        # image_label_overlay = label2rgb(conected_frontier, image=grid_frontier_img)

        # fig, ax = plt.subplots(figsize=(10, 6))

        # ax.imshow(image_label_overlay)



        manh_dist = []  # stores distances

        cents = []  # stores centers of frontiers



        for region in regionprops(conected_frontier):

            # take regions with large enough areas

            if region.area >= 10:  # do not consider small frontier groups

                # print the centroid of each valid region

                cen_y = region.centroid[0]  # Centroid coordinate tuple (row, col)

                cen_x = region.centroid[1]  # Centroid coordinate tuple (row, col)

                cents.append([cen_x, cen_y])

                manh = abs(cen_x - robot_pose_pixel[0]) + abs(

                    cen_y - robot_pose_pixel[1])  # Manhattan Distance from robot to each frontier center

                manh_dist.append(manh)

                # print(region.centroid)   # Centroid coordinate tuple (row, col)

                # draw rectangle around segmented coins

                # minr, minc, maxr, maxc = region.bbox

                # rect = mpatches.Rectangle((minc, minr), maxc - minc, maxr - minr,

                #                         fill=False, edgecolor='red', linewidth=1)

                # ax.add_patch(rect)

        # ax.set_axis_off()

        # plt.tight_layout()

        # plt.show()


        
        #next_goal = cents[manh_dist.index(min(manh_dist))]
      

      	# sort two list: centers of each frontiers according to the man_distance
        cents_sorted = [x for _,x in sorted(zip(manh_dist, cents))]
        if self.flag_stuck==0:
            next_goal_pixel = cents_sorted[0]
        if self.flag_stuck==1:
            next_goal_pixel = cents_sorted[1] # try a further frontier if get stuck into the closest one
            self.flag_stuck=0
       
		
        # transform into real world
        next_goal_world = [0,0]

        next_goal_world[0] = next_goal_pixel[0] * resolution + origin_x

        next_goal_world[1] = next_goal_pixel[1] * resolution + origin_y

	#while abs(self.goals[1].position.x - next_goal[0])<0.5 and abs(self.goals[1].position.y - next_goal[1])<0.5 :
	#	cents.remove(manh_dist.index(max(manh_dist)))
	#	manh_dist.remove(manh_dist.index(max(manh_dist)))
	#	next_goal = cents[manh_dist.index(max(manh_dist))]
	#	next_goal[0] = next_goal[0] * resolution + origin_x
	#	next_goal[1] = next_goal[1] * resolution + origin_y

        print 'next_goal: ', next_goal_world



        return next_goal_world

		




if __name__ == '__main__':

    try:
	DemoResetter()

    except rospy.ROSInterruptException:

        rospy.loginfo("exception")
