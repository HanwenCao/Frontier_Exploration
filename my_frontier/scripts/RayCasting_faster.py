import numpy as np
import dippykit as dip
import matplotlib.pyplot as plt
from scipy import ndimage as ndi
from skimage import morphology, color, data, filters
from matplotlib import cm
from scipy import ndimage as ndi
import matplotlib.pyplot as plt
from skimage.morphology import watershed, disk
from skimage import data
from skimage.filters import rank
from skimage.util import img_as_ubyte
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from skimage import data, filters, segmentation, measure, morphology, color
import math
from PIL import Image
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from skimage.segmentation import clear_border
from skimage.measure import label, regionprops
from skimage.morphology import closing, square
from skimage.color import label2rgb
import datetime


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

def ray_casting(size, unknown, occup, robot, laser_range):
    start = datetime.datetime.now()
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
    print('----')

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
    # print('angle_range of obstacle i:',angle_range)

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
        print('obstacle i=', i)
        print('range=', agr)

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



    unknown_seen_img = dip.float_to_im(unknown_seen)
    dip.imshow(unknown_seen_img)
    dip.show()

    # entropy
    entropy_Shannon = np.sum(unknown_seen == 1)  # assume p(unknown)=0.5
    # print('Shannon entropy of map: ', entropy_Shannon,' bits')

    end = datetime.datetime.now()
    print(end - start)
    return entropy_Shannon


if __name__ == "__main__":
    # suppose the robot is here:
    robot = [303, 239]  # index [y,x]
    # suppose the range of laser is (pixel):
    laser_range = 200  # 200 pixel == 10m
    grid = np.loadtxt('rawmap.txt')
    # ------------------- unknown cells ---------------------
    thresh_low = -1
    thresh_high = -1
    unknown = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0
    # -----------------occupied cells---------------------
    thresh_low = 90
    thresh_high = 100
    occup = ((grid <= thresh_high) & (grid >= thresh_low)) * 1.0

    entropy_shannon = ray_casting(grid.shape, unknown, occup, robot, laser_range)
    print('Shannon Entropy = ', entropy_shannon, 'bits')


