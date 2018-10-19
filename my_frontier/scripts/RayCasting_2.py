import numpy as np
import dippykit as dip
import matplotlib.pyplot as plt
from scipy import ndimage as ndi
from skimage import morphology,color,data, filters
from matplotlib import cm
from scipy import ndimage as ndi
import matplotlib.pyplot as plt
from skimage.morphology import watershed, disk
from skimage import data
from skimage.filters import rank
from skimage.util import img_as_ubyte
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from skimage import data,filters,segmentation,measure,morphology,color
import math
from PIL import Image
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from skimage.segmentation import clear_border
from skimage.measure import label, regionprops
from skimage.morphology import closing, square
from skimage.color import label2rgb


def angle_between(p1, p2):  # calculation based on index
    dif_y = -(p1[0] - p2[0])
    dif_x = p1[1] - p2[1]
    ang = np.arctan2(dif_y,dif_x)
    return np.rad2deg(ang % (2 * np.pi))

grid = np.loadtxt('rawmap.txt')
#grid = np.array([[-1,-1,-1,-1,100,-1],[-1,-1,-1,-1,100,-1],[-1,-1,-1,-1,100,-1],[-1,-1,100,100,100,-1],[-1,-1,100,-1,-1,-1],[-1,-1,-1,-1,-1,-1]])

# suppose the robot is here:
robot = [261,303]

# ------------------- unknown cells ---------------------
thresh_low = -1
thresh_high = -1
unknown =((grid <= thresh_high) & (grid >= thresh_low))*1.0
# -----------------occupied cells---------------------
thresh_low = 90
thresh_high = 100
occup =((grid <= thresh_high) & (grid >= thresh_low))*1.0
# "1" in dst are the open cells
# "0" in dst are the unvisited cells and occupied cells
occup_img = dip.float_to_im(occup)
dip.im_write(occup_img, "occup_img.tif")

# special situation: angle=0
exist_right_obstacle = list()  # initialize flag
for col in range(robot[1]+1,grid.shape[1]):
    if occup[robot[0],col]==1:
        exist_right_obstacle.append(col)  # set the flag (there exist obstacle cell on the right of robot)

# the closest "right" obstacle is at (robot[0], exist_right_obstacle)
# if exist_right_obstacle = 0, then there is no such situation
#print(exist_right_obstacle)

occup[robot[0],robot[1]:] = 0  # clear obstacle on the right

# -------------group them!------------------
conected_obstacle, label_num = measure.label(occup, return_num=True, connectivity=2)
#print("num of obstacles: %d" %(label_num))
#conected_obstacle = dip.float_to_im(conected_obstacle/label_num)
#dip.im_write(conected_frontier, "conected_frontier.tif")
#plt.imshow(conected_frontier)
#plt.show()

# -------------coordinates------------------
coords = list()
for region in regionprops(conected_obstacle):
    coords.append(region.coords)  # a record of coordinates of all occupied cells that belong to the same obstacle
#coords[0] ... coords[label_num-1]


dist = list()  # a record of distance of all occupied cells that belong to the same obstacle
# dist[0] ... dist[label_num-1]
dist_obst_i = list()
for i in range(label_num):  # for every obstacle
    #print('Obstacle No.',i,':')
    for coor in coords[i]:  # take out coordinate of each occupied cell that belongs to the same obstacle
        #print('coordinate:',coor)
        d = np.linalg.norm(coor - robot)
        #print('distance between this cell to robot:',d)
        dist_obst_i.append(d)
    dist.append(dist_obst_i)
    dist_obst_i = list()

#print(dist[0])
#print(dist[1])
print('----')

angle = list()  # a record of angle of all occupied cells that belong to the same obstacle
# angle[0] ... angle[label_num-1]
angle_obst_i = list()
for i in range(label_num):
    #print('Obstacle No.', i, ':')
    for coor in coords[i]:  # take out coordinate of each occupied cell that belongs to the same obstacle
        #print('coordinate:', coor)
        ang = angle_between(coor, robot)
        #print('angle: ',ang)
        angle_obst_i.append(ang)
    angle.append(angle_obst_i)
    angle_obst_i = list() #delete
#print('angles: ',angle)

# angle range of obstacle i:  (not perfect!!)
angle_range = list()
for i in range(label_num):
    #print('Obstacle No.', i, ':')
    angle_min = min(angle[i])
    angle_max = max(angle[i])
    if exist_right_obstacle != list():  # if I have cleared some obstacles
        if angle_min<=10:
            angle_min = 0  # then I have to fill the blank
        if angle_max>=360-10:
            angle_max = 360  # fill the blank

    #print('min angle:',angle_min)
    #print('max angle:', angle_max)
    agr = [angle_min,angle_max]
    angle_range.append([angle_min,angle_max])
#print('angle_range of obstacle i:',angle_range)

# unknown cells' coordinates:
#unknown_coor = list()  # a record of coordinates of all unknown cells
unknown_index = list()  # a record of index of all unknown cells
unknown_coors = np.nonzero(unknown)
#unknown_coors_y = unknown_coors[0]  # change x/y
#unknown_coors_x = unknown_coors[1]  # change x/y
for i in range(unknown_coors[0].size):
#    unk = [unknown_coors_x[i],unknown_coors_y[i]]
     unknown_index.append([unknown_coors[0][i],unknown_coors[1][i]])
#    unknown_coor.append(unk)
#print('coordinates of unknown cells:',unknown_coor)
#print('index of unknown cells:',unknown_index)

# unknown cells' distances and angles:
unknown_dist = list()  # a record of distances of all unknown cells
unknown_angle = list()  # a record of angle of all unknown cells
for ele in unknown_index:
    d = np.linalg.norm(np.array(ele) - robot)  # distance of unknown cell
    unknown_dist.append(d)
    ang = angle_between(ele, robot)  # angle of unknown cell
    unknown_angle.append(ang)
#print('distances of unknown cells to robot:',unknown_dist)
#print('angles of unknown cells:',unknown_angle)


# cells that cannot be seen
shadow = np.zeros(grid.shape)
for i in range(label_num):
    agr = angle_range[i]  # angle range of this obstacle
    print('obstacle i=', i)
    print('range=', agr)
    count = 0
    for ele in unknown_angle:
        if agr[0] <= ele <= agr[1]:
            #print('ele:', ele)
            angle_closest_obs = min(angle[i], key=lambda x: abs(x - ele))
            #print('nearest angle:', angle_closest_obs)
            ind = angle[i].index(angle_closest_obs)
            #print('index:', ind)
            dist_closest_obs = dist[i][ind]
            #print('distance of this angle:', dist_closest_obs)

            #print('that is:',count, ele)
            #print('index:',unknown_index[count])
            ind=unknown_index[count]
            if unknown_dist[count]>dist_closest_obs:
                shadow[ind[0],ind[1]]=1
        count = count + 1

shadow[robot[0],robot[1]]=0.5
shadow_img = dip.float_to_im(shadow)
dip.imshow(shadow_img)
dip.show()



aa=0




