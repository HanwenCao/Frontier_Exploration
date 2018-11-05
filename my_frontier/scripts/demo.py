#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import String, Float64
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
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
import numpy as np
#import dippykit as dip
from skimage import measure
from skimage.measure import label, regionprops
import random
import datetime


class DemoResetter():
    def __init__(self):
        rospy.init_node('Prototype')
        # self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.clear_costmap_srv = None
        self.p_occpu = 0.1  # the probability of occpancy foe each unknown cell
        self.pose_entropy_gmapping = 4.377   # an initial value of ~entropy published by gmapping
        # self.publishMap()
        self.pub = rospy.Publisher("/mobile_base/commands/reset_odometry", std_msgs.Empty, queue_size=1)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.sub_entropy_gmapping = rospy.Subscriber('slam_gmapping/entropy', Float64, self.callback_entropy_gmapping)
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
        #rospy.loginfo(rospy.get_caller_id() + 'I heard the map')
        rawmap = np.array(data)
        self.rawmap = rawmap.reshape(info.height, info.width)
        np.savetxt("rawmap.txt", self.rawmap, fmt='%d');
        self.p0 = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z, info.resolution])
        # p0 is the origin and resolution of the map
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


    def callback_entropy_gmapping(self, data):
        print 'pose entropy estimated by gmapping: ', data.data
        self.pose_entropy_gmapping = data.data



    def initialGoals(self):	
        self.goals = []
        goalA = Pose()
        goalA.position.x = 100
        goalA.position.y = 100
        goalA.orientation.w = 1
        goalB = Pose()
        goalB.position.x = 0
        goalB.position.y = 0
        goalB.orientation.w = 1
        self.goals.append(goalA)
        self.goals.append(goalB)
        print "initial goals!"

    def setupGoals(self, next_x, next_y):
	if abs(self.goals[1].position.x - next_x)<0.02 and abs(self.goals[1].position.y - next_y)<0.02:
		print "Almost same to last goal, may get stuck"
                self.flag_stuck = 1
        goalB = Pose()   # goalB is next frontier
        goalB.position.x = next_x
        goalB.position.y = next_y
        goalB.orientation.z = self.odom[2]
        goalB.orientation.w = self.odom[3]
        self.goals[1] = goalB 
        #print "exploration goal is set!"

    def Rotate(self):	
        #get orientation
        print "rotate to get a whole view1"
        #orientation_z = self.odom[2] #read current orientation
        #orientation_w = self.odom[3]
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]    #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = 0#orientation_w
        goal_rot.orientation.w = 1#1-orientation_w
        self.navigateToGoal(goal_pose=goal_rot)    # theta=0 # rotate 90'

        print "rotate to get a whole view2"
        #get orientation
        #orientation_z = self.odom[2] #read current orientation
        #orientation_w = self.odom[3]
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]  #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = math.sin(math.pi/4)#orientation_w
        goal_rot.orientation.w = math.cos(math.pi/4)#1-orientation_w
        self.navigateToGoal(goal_pose=goal_rot)    # rotate 90'

        print "rotate to get a whole view3"
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]  #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = 1
        goal_rot.orientation.w = 0
        self.navigateToGoal(goal_pose=goal_rot)    # rotate 90'

        print "rotate to get a whole view4"
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]  #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = math.sin(math.pi/4)#orientation_w
        goal_rot.orientation.w = -math.cos(math.pi/4)#1-orientation_w
        self.navigateToGoal(goal_pose=goal_rot)    # rotate 180'
        
        print "rotate to get a whole view5"
        #set a rotation goal
        goal_rot = Pose()
        goal_rot.position.x = self.odom[0]    #read current odom
        goal_rot.position.y = self.odom[1]
        goal_rot.orientation.z = 0#orientation_w
        goal_rot.orientation.w = 1#1-orientation_w
        self.navigateToGoal(goal_pose=goal_rot)    # theta=0 # rotate 90'



    def covariance(self, P0, d_d, theta_k):
        #d_d : delta_d (odometry reading: translation)
        #theta_k : theta_k (current heading)
        i=0
        a = 1.5
        V = a*np.array([[0.05**2, 0],[0, math.radians(0.5)**2]])  # odom noise variance, with standard deviation 5cm and 0.5 degree
        
        for dd in d_d:
            theta = theta_k[i]
            Fx = np.array([[1,0,-dd*math.sin(theta)],[0,1,dd*math.cos(theta)],[0,0,1]])  #jacobian matrix
            Fv = np.array([[math.cos(theta), 0],[math.sin(theta), 0],[0,1]])

            Pk = np.dot(np.dot(Fx, P0), np.transpose(Fx)) + np.dot(np.dot(Fv, V), np.transpose(Fv))
            P0 = Pk
            i=i+1

        return Pk



    def navigate(self):

        self.Rotate()
        while not rospy.is_shutdown():   
            if not self.stop:


                try:
                    #self.Rotate()
                    next_goal = self.process(info=self.p0, grid=self.rawmap, odom=self.odom[0:2])                  
                    self.setupGoals(next_x=next_goal[0],next_y=next_goal[1])
                    print "go to: ", self.goals[1]
                    self.navigateToGoal(goal_pose=self.goals[1])  # go to next frontier
                    self.resetCostmaps()


                except Exception, e:
                    print e
                    pass
                    rospy.sleep(.1)
            else:
                rospy.sleep(.2)


    def resetCostmaps(self):
        if self.clear_costmap_srv is None:
            rospy.wait_for_service('/move_base/clear_costmaps')
            self.clear_costmap_srv = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
        self.clear_costmap_srv()
        print "reset costmaps"


    def navigateToGoal(self, goal_pose):



        # Create the goal point

        goal = move_base_msgs.MoveBaseGoal()

        goal.target_pose.pose = goal_pose

        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.header.frame_id = "odom"  #odom,  map



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

        opencell = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0  # threshold
            
        # "1" in dst_open are the open cells
        # "0" in dst_open are the unvisited cells and occupied cells

        # detect contours

        contours_open = measure.find_contours(opencell, 0.5)

        contours_open_cell = list()

        for ele in contours_open:

            for cell in ele:

                contours_open_cell.append(cell.tolist())

        # print(contours_open_cell)



        # --------------------------------------------- unvisited cells ---------------------

        thresh_low = -1

        thresh_high = -1

        unknown = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0  # threshold

        # "1" in unknown are the unvisited cells

        # "0" in unknown are the open cells and occupied cells


        # detect contours

        contours_unvisited = measure.find_contours(unknown, 0.5)

        contours_unvisited_cell = list()

        for ele in contours_unvisited:

            for cell in ele:

                contours_unvisited_cell.append(cell.tolist())

        # print(contours_unvisited_cell)


        # -----------------occupied cells---------------------
        thresh_low = 90
        thresh_high = 100
        occup = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0


        # ----------------------------------------------------------------

        # frontier detection 

        frontier_cells = [x for x in contours_unvisited_cell if x in contours_open_cell]


        grid_frontier = np.zeros(size)
        for ele in frontier_cells:

            #grid_frontier[math.floor(ele[0]), math.floor(ele[1])] = 1
            grid_frontier[int(ele[0]), int(ele[1])] = 1



        # group them!
        conected_frontier, label_num = measure.label(grid_frontier, return_num=True)

        
        manh_dist = []  # stores distances
        cents = []  # stores centers of frontiers
        for region in regionprops(conected_frontier):

            # take regions with large enough areas

            if region.area >= 60:  # do not consider small frontier groups
                # the centroid of each valid region
                cen_y = region.centroid[0]  # Centroid coordinate tuple (row, col)
                cen_x = region.centroid[1]  # Centroid coordinate tuple (row, col)
                cents.append([cen_x, cen_y])  #cents[col,row]
                manh = abs(cen_x - robot_pose_pixel[0]) + abs(
                    cen_y - robot_pose_pixel[1])  # Manhattan Distance from robot to each frontier center
                manh_dist.append(manh)
      

      	# sort two list: centers of each frontiers according to the man_distance
        cents_sorted = [x for _,x in sorted(zip(manh_dist, cents))]   # a sorted record of all the candidate goals (close-far)
        



        # calculate entropy of each candidates
        entropy_shannon = list()  # corresponding Shannon's entropy of each candidate goals
        for center in cents_sorted:  
            num_unknown = ray_casting(size, unknown, occup, center, 80)   


            #Shannon's entropy
            entropy_shannon_i = self.cal_entropy_shannon(num_unknown)  # shannon's entropy at this location
    
            entropy_shannon.append(entropy_shannon_i)    # laser range = 4m
            print 'Shannon Entropy = ', entropy_shannon
 



        print "candidates(pixel col,row): ", cents_sorted
        # pixel to world
        for ele in cents_sorted:
            ele[0] = ele[0] * resolution + origin_x
            ele[1] = ele[1] * resolution + origin_y
        print "candidates(world xy): ", cents_sorted





        # generate trajectory to each frontier
        # Wait for the availability of this service
        rospy.wait_for_service('/move_base/make_plan')

        # Get a proxy to execute the service
        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        for center in cents_sorted:

            plan_start, plan_goal, plan_tolerance = self.set_plan_goal(center)  #set up a planning goal 
            plan_response = self.make_plan(start=plan_start, goal=plan_goal, tolerance=plan_tolerance) # plan

            plan_len = len(plan_response.plan.poses)  # length of trajectory
            print "plan size: %f" %(plan_len)
            if plan_len == 0:  # fail to make a plan
                print 'cannot generate trajectory'
                continue

            plan_dd, plan_theta = self.get_plan_trajectory(plan_response, plan_len)  # get delta_d and theta_k for uncertainty propogation


            # covariance
            P0 = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.05]])  # initial covariance
            uncertainty = math.sqrt(np.linalg.det(P0))
            #print 'initial ucertainty: ', uncertainty

            # propogate
            Pk = self.covariance( P0, plan_dd, plan_theta)
            #predicted uncertainty
            print(Pk)
            uncertainty = math.sqrt(np.linalg.det(Pk))
            print 'final ucertainty: ', uncertainty

            # Renyi's entropy
            alpha = 1 + 1/uncertainty
            H_renyi_one = (1/(1-alpha)) * math.log((self.p_occpu**alpha + (1-self.p_occpu)**alpha), 2)

        




        if self.flag_stuck==0:
            #next_goal_pixel = cents_sorted[0]
            #print "try the closest goal!!"
            next_goal_world = cents_sorted[ entropy_shannon.index(max(entropy_shannon)) ]
            print "try the max-info-gain goal!!"
        if self.flag_stuck==1:
            #next_goal_pixel = cents_sorted[1] # try a further frontier if get stuck into the closest one
            #cent_rand = sample(cents_sorted,  1)
             # assume there are more than 4 frontiers
            print "try another goal!!"
            next_goal_world = cents_sorted[random.randint(1,3)] # try a random frontier if get stuck into the closest one
            self.flag_stuck=0
       
		
        # transform into real world
        #next_goal_world = [0,0]

        #next_goal_world[0] = next_goal_pixel[0] * resolution + origin_x

        #next_goal_world[1] = next_goal_pixel[1] * resolution + origin_y

        #print 'next_goal: ', next_goal_world

        return next_goal_world



    def cal_entropy_shannon(self, num_unknown):
            # Shannon's entropy of one cell
            H_shannon_one = - ( self.p_occpu*math.log(self.p_occpu,2) + (1-self.p_occpu)*math.log((1-self.p_occpu),2) ) 
            # total Shannon's entrooy
            entropy_shannon = num_unknown * H_shannon_one 
            return entropy_shannon  


    def set_plan_goal(self, center):
        # make a plan and generate a trajectory to the "center" without real action
        
        # set start and goal
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x = self.odom[0]
        start.pose.position.y = self.odom[1]
        start.pose.orientation.z = self.odom[2]
        start.pose.orientation.w = self.odom[3]

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = center[0]
        goal.pose.position.y = center[1]
        goal.pose.orientation.z = self.odom[2]
        goal.pose.orientation.w = self.odom[3]

        # set tolerance
        tolerance = 0.1
        return start, goal, tolerance


    def get_plan_trajectory(self, plan_response, plan_len):
        plan_x = np.empty((len(plan_response.plan.poses),1))
        plan_y = np.empty((len(plan_response.plan.poses),1))
        plan_w = np.empty((len(plan_response.plan.poses),1))
        i = 0
        for plan_pose in plan_response.plan.poses:
            plan_x[i] = plan_pose.pose.position.x  #x
            plan_y[i] = plan_pose.pose.position.y  #y
            plan_w[i] = plan_pose.pose.orientation.w  #w
            i = i+1
        plan_x_next = np.zeros((plan_len,1))
        plan_x_next[0:plan_len-1] = plan_x[1:plan_len]
        plan_x_next[plan_len-1] = plan_x[plan_len-1]
        plan_dx = plan_x_next - plan_x
    
        plan_y_next = np.zeros((plan_len,1))
        plan_y_next[0:plan_len-1] = plan_y[1:plan_len]
        plan_y_next[plan_len-1] = plan_y[plan_len-1]
        plan_dy = plan_y_next - plan_y

        plan_dd = np.sqrt(plan_dx**2 + plan_dy**2)  # displacement (delta_d)
        #print plan_dd

        plan_theta = np.angle(plan_dx + plan_dy*1j,deg=True)  # current heading (theta_k)
        return plan_dd, plan_theta 



def angle_point(p1, p2):  # calculation based on index
    dif_y = -(p1[0] - p2[0])
    dif_x = p1[1] - p2[1]
    ang = np.arctan2(dif_y, dif_x)
    return np.rad2deg(ang % (2 * np.pi))


def angle_points(p1, robot):  # calculation based on index
    dif_yx = p1 - robot
    dif_yx[:,0] = - dif_yx[:,0]
    ang = np.arctan2(dif_yx[:,0], dif_yx[:,1])
    return np.rad2deg(ang % (2 * np.pi))


def ray_casting(size, unknown, occup, cent, laser_range):
    start = datetime.datetime.now()
    robot = [0,0]
    robot[0] = int(cent[1]) #robot[row,col]
    robot[1] = int(cent[0])
    # -----------------------------------------------------
    # special situation: angle=0
    exist_right_obstacle = list()  # initialize flag
    for col in range(robot[1] + 1, size[1]):
        if occup[robot[0], col] == 1:
            exist_right_obstacle.append(col)  # set the flag (there exist obstacle cell on the right of robot)

    # the closest "right" obstacle is at (robot[0], exist_right_obstacle)
    # if exist_right_obstacle = 0, then there is no such situation
    # print(exist_right_obstacle)

    occup[robot[0], robot[1]:] = 0  # clear all obstacles on the right

    # -------------group the obstacles!------------------

    conected_obstacle, label_num = measure.label(occup, return_num=True, connectivity=2)
    # print("num of obstacles: %d" %(label_num))
    # conected_obstacle = dip.float_to_im(conected_obstacle/label_num)
    # dip.im_write(conected_frontier, "conected_frontier.tif")
    # plt.imshow(conected_obstacle)
    # plt.show()

    # maybe detect contours here #
    # but in occupancy grid, obstacle is like "contour"

    # -------------coordinates of obstacles------------------
    coords = list()
    for region in regionprops(conected_obstacle):
        coords.append(region.coords)  # a record of coordinates of all occupied cells that belong to the same obstacle
    # coords[0] ... coords[label_num-1]

    # -------------distances of obstacles------------------
    dist = list()  # a record of distance of all occupied cells that belong to the same obstacle
    # dist[0] ... dist[label_num-1]
    dist_obst_i = list()
    for i in range(label_num):  # for every obstacle
        # print('Obstacle No.',i,':')
        for coor in coords[i]:  # take out coordinate of each occupied cell that belongs to the same obstacle
            # print('coordinate:',coor)
            d = np.linalg.norm(coor - robot)
            # print('distance between this cell to robot:',d)
            dist_obst_i.append(d)
        dist.append(dist_obst_i)
        dist_obst_i = list()

    # print(dist[0])
    # print(dist[1])
    # print('----')

    # -------------angles of obstacles------------------
    angle = list()  # a record of angle of all occupied cells that belong to the same obstacle
    # angle[0] ... angle[label_num-1]
    angle_obst_i = list()
    for i in range(label_num):
        # print('Obstacle No.', i, ':')
        for coor in coords[i]:  # take out coordinate of each occupied cell that belongs to the same obstacle
            # print('coordinate:', coor)
            ang = angle_point(coor, robot)
            # print('angle: ',ang)
            angle_obst_i.append(ang)
        angle.append(angle_obst_i)
        angle_obst_i = list()  # delete
    # print('angles: ',angle)

    # angle range of obstacle i:
    angle_range = list()
    for i in range(label_num):
        # print('Obstacle No.', i, ':')
        angle_min = min(angle[i])
        angle_max = max(angle[i])
        if exist_right_obstacle != list():  # if I have cleared some obstacles
            if angle_min <= 10:
                angle_min = 0  # then I have to fill the blank
            if angle_max >= 360 - 10:
                angle_max = 360  # fill the blank
        # print('min angle:',angle_min)
        # print('max angle:', angle_max)
        # agr = [angle_min,angle_max]
        angle_range.append([angle_min, angle_max])
    #print 'angle_range of obstacle i:',angle_range

    # -------------coordinates of unknown cells------------------
    # unknown_coor = list()  # a record of coordinates of all unknown cells
    unknown_index = list()  # a record of index of all unknown cells
    unknown_coors = np.nonzero(unknown)
    # unknown_coors_y = unknown_coors[0]  # change x/y
    # unknown_coors_x = unknown_coors[1]  # change x/y
    for i in range(unknown_coors[0].size):
        #    unk = [unknown_coors_x[i],unknown_coors_y[i]]
        unknown_index.append([unknown_coors[0][i], unknown_coors[1][i]])
    #    unknown_coor.append(unk)
    # print('coordinates of unknown cells:',unknown_coor)
    # print('index of unknown cells:',unknown_index)



    # -------------distances and angles of unknown cells------------------

    unknown_index = np.array(unknown_index, dtype='int32')  # a record of index of all unknown cells
    index_diff = unknown_index - robot
    dists = np.sqrt(index_diff[:,0]**2 + index_diff[:,1]**2)  # a record of distances of all unknown cells
    unknown_index_inrange = unknown_index[dists < laser_range]
    unknown_dist = dists[dists < laser_range]   # a record of distances of all unknown cells within range
    unknown_angle = angle_points(unknown_index_inrange,robot)   # a record of angle of all unknown cells within range

    # -------------cells that can be seen (faster algorithm)-------------
    unknown_seen = np.zeros(size)
    for ele in unknown_index_inrange:
        unknown_seen[ele[0], ele[1]] = 1  # first only use cells that within laser range

    # unknown_seen_img = dip.float_to_im(unknown_seen)
    # dip.imshow(unknown_seen_img)
    # dip.show()



    for i in range(label_num):
        agr = angle_range[i]  # angle range of this obstacle
        #print 'obstacle i=', i
        #print 'range=', agr

        unknown_angle_obs_i = unknown_angle[ (unknown_angle>=agr[0]) & (unknown_angle<=agr[1]) ]
        unknown_dist_obs_i = unknown_dist[ (unknown_angle>=agr[0]) & (unknown_angle<=agr[1]) ]
        unknown_index_obs_i = unknown_index_inrange[ (unknown_angle>=agr[0]) & (unknown_angle<=agr[1]) ]
        dist_closest_obs = list()


        for ele in unknown_angle_obs_i:  # if the angle range of obstacle i is large, then this loop is slow
            angle_closest_obs = min(angle[i], key=lambda x: abs(x - ele))
            ind = angle[i].index(angle_closest_obs)
            dist_closest_obs.append(dist[i][ind])

        unknown_behind = unknown_index_obs_i[unknown_dist_obs_i > dist_closest_obs]


        for ele in unknown_behind:
            unknown_seen[ele[0], ele[1]] = 0

    return np.sum(unknown_seen == 1)








if __name__ == '__main__':

    try:
	DemoResetter()

    except rospy.ROSInterruptException:

        rospy.loginfo("exception")
