#Setup
import math
from controller import Robot
import numpy as np
import pandas as pd

TIME_STEP = 64
MAX_SPEED = 6.28
robot = Robot()

waypoint = ((10,10), (55, 35), (33, 35), (10,35), (55,10), (33,10))
occupancy_grid = pd.read_table('../../libraries/SampleTestMap1.csv', dtype=float, header=None, sep=',').fillna(0).values

#To see what angle (in degrees) relitive to X axis bot is facing
def ang_to_X(xf,yf,xb,yb):
    m = (yf-yb)/(xf-xb)
    c = yf - (m * xf)
    x_intercept = -c/m
    if (yf < yb) & (xf < xb):
        return math.degrees(math.atan(yf/(xf - x_intercept))) - 180
    elif (yf > yb) & (xf < xb):
        return math.degrees(math.atan(yf/(xf - x_intercept))) + 180
    else:
        return math.degrees(math.atan(yf/(xf - x_intercept)))
    
def next_grid(curr_x_f, curr_y_f, curr_x_b, curr_y_b):
    next_grid_x = int((curr_x_f - curr_x_b)/4) + curr_x_f
    next_grid_y = int((curr_y_f - curr_y_b)/4) + curr_y_f
    
    next_grid_x_r = int((curr_y_f - curr_y_b)/4) + curr_x_f
    next_grid_y_r = int((curr_x_f - curr_x_b)/4) + curr_y_f
    
    next_grid_x_l = -1*int((curr_y_f - curr_y_b)/4) + curr_x_f
    next_grid_y_l = -1*int((curr_x_f - curr_x_b)/4) + curr_y_f
    
    if next_grid_y > 40 or next_grid_x > 60:
        return True
    elif next_grid_y < 0 or next_grid_x < 0:
        return True
        
    occ1 = occupancy_grid[next_grid_y][next_grid_x]
    occ2 = occupancy_grid[next_grid_y_r][next_grid_x_r]
    occ3 = occupancy_grid[next_grid_y_l][next_grid_x_l]
    
    if occ1 or occ2 or occ3:
        return True
    
    next_grid_x = int((curr_x_f - curr_x_b)/3) + curr_x_f
    next_grid_y = int((curr_y_f - curr_y_b)/3) + curr_y_f
    
    next_grid_x_r = int((curr_y_f - curr_y_b)/3) + curr_x_f
    next_grid_y_r = int((curr_x_f - curr_x_b)/3) + curr_y_f
    
    next_grid_x_l = -1*int((curr_y_f - curr_y_b)/3) + curr_x_f
    next_grid_y_l = -1*int((curr_x_f - curr_x_b)/3) + curr_y_f
    
    if next_grid_y > 40 or next_grid_x > 60:
        return True
    elif next_grid_y < 0 or next_grid_x < 0:
        return True
        
    occ1 = occupancy_grid[next_grid_y][next_grid_x]
    occ2 = occupancy_grid[next_grid_y_r][next_grid_x_r]
    occ3 = occupancy_grid[next_grid_y_l][next_grid_x_l]
    
    if occ1 or occ2 or occ3:
        return True
        
    next_grid_x = int((curr_x_f - curr_x_b)/2) + curr_x_f
    next_grid_y = int((curr_y_f - curr_y_b)/2) + curr_y_f
    
    next_grid_x_r = int((curr_y_f - curr_y_b)/2) + curr_x_f
    next_grid_y_r = int((curr_x_f - curr_x_b)/2) + curr_y_f
    
    next_grid_x_l = -1*int((curr_y_f - curr_y_b)/2) + curr_x_f
    next_grid_y_l = -1*int((curr_x_f - curr_x_b)/2) + curr_y_f
    
    if next_grid_y > 40 or next_grid_x > 60:
        return True
    elif next_grid_y < 0 or next_grid_x < 0:
        return True
        
    occ1 = occupancy_grid[next_grid_y][next_grid_x]
    occ2 = occupancy_grid[next_grid_y_r][next_grid_x_r]
    occ3 = occupancy_grid[next_grid_y_l][next_grid_x_l]
    
    if occ1 or occ2 or occ3:
        return True
        
    return False

def left_grid(curr_x_f, curr_y_f, curr_x_b, curr_y_b):
    x = int((curr_grid_x_f - curr_grid_x_b)/4)
    y = int((curr_grid_y_f - curr_grid_y_b)/4)
    left_x = y + curr_grid_x_f
    left_y = -1*x + curr_grid_y_f
    
    if left_y > 40 or left_x > 60:
        return True
    elif left_y < 0 or left_x < 0:
        return True
    elif occupancy_grid[left_y][left_x]:
        return True
        
    x = int((curr_grid_x_f - curr_grid_x_b)/3)
    y = int((curr_grid_y_f - curr_grid_y_b)/3)
    left_x = y + curr_grid_x_f
    left_y = -1*x + curr_grid_y_f
    
    if left_y > 40 or left_x > 60:
        return True
    elif left_y < 0 or left_x < 0:
        return True
    elif occupancy_grid[left_y][left_x]:
        return True
    
    x = int((curr_grid_x_f - curr_grid_x_b)/2)
    y = int((curr_grid_y_f - curr_grid_y_b)/2)
    left_x = y + curr_grid_x_f
    left_y = -1*x + curr_grid_y_f
    
    if left_y > 40 or left_x > 60:
        return True
    elif left_y < 0 or left_x < 0:
        return True
    elif occupancy_grid[left_y][left_x]:
        return True
        
    x = int((curr_grid_x_f - curr_grid_x_b))
    y = int((curr_grid_y_f - curr_grid_y_b))
    left_x = y + curr_grid_x_f
    left_y = -1*x + curr_grid_y_f
    
    if left_y > 40 or left_x > 60:
        return True
    elif left_y < 0 or left_x < 0:
        return True
    elif occupancy_grid[left_y][left_x]:
        return True
        
    return False
     
# Wheels 1 and 2 are front left and right wheels respectivly
# Wheels 3 and 4 are back left and right wheels respectivly
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
 
#GPS setup
gps_front = robot.getGPS('gps_front')
gps_front.enable(TIME_STEP)   

gps_back = robot.getGPS('gps_back')
gps_back.enable(TIME_STEP) 

waypoints_hit = 0
#Main code runs here
while robot.step(TIME_STEP) != -1:

    #Read GPS
    gps_front_value = gps_front.getValues()
    gps_back_value = gps_back.getValues()
    
    #To print GPS values  
    msg = "GPS Values:   Front ="
    for each_val in gps_front_value:
        msg += " {0:0.5f}".format(each_val)
    msg += "    Back = "
    for each_val in gps_back_value:
        msg += " {0:0.5f}".format(each_val)
    msg += "    Ang = "
    ang = ang_to_X(gps_front_value[0],gps_front_value[1],gps_back_value[0],gps_back_value[1])
    msg += str(ang)
    #print(msg)  
    
    curr_grid_x_f = abs(math.floor(gps_front_value[0]/0.1)) + 1
    curr_grid_y_f = abs(math.ceil(gps_front_value[1]/0.1))

    curr_grid_x_b = abs(math.floor(gps_back_value[0]/0.1)) + 1
    curr_grid_y_b = abs(math.ceil(gps_back_value[1]/0.1))
    
    x = int((curr_grid_x_f - curr_grid_x_b)/4)
    y = int((curr_grid_y_f - curr_grid_y_b)/4)
    left_x = y + curr_grid_x_f
    left_y = -1*x + curr_grid_y_f
    
    object_in_way = next_grid(curr_grid_x_f, curr_grid_y_f, curr_grid_x_b, curr_grid_y_b)
    object_to_left = left_grid (curr_grid_x_f, curr_grid_y_f, curr_grid_x_b, curr_grid_y_b)
    
    waypoints_hit = waypoints_hit%len(waypoint)

    distance = math.sqrt((((waypoint[waypoints_hit][1]-curr_grid_y_f)**2)+((waypoint[waypoints_hit][0]-curr_grid_x_f))**2))
    
    expected_angle = math.asin((waypoint[waypoints_hit][1]-curr_grid_y_f)/distance)
    expected_angle = expected_angle * (180/math.pi)
        
    if curr_grid_x_f == waypoint[waypoints_hit][0] and curr_grid_y_f > waypoint[waypoints_hit][1]:
       expected_angle = 90
    elif curr_grid_x_f == waypoint[waypoints_hit][0] and curr_grid_y_f < waypoint[waypoints_hit][1]:
        expected_angle = -90
    elif curr_grid_x_f > waypoint[waypoints_hit][0] and curr_grid_y_f == waypoint[waypoints_hit][1]:
       expected_angle = 180
    elif curr_grid_x_f < waypoint[waypoints_hit][0] and curr_grid_y_f == waypoint[waypoints_hit][1]:
       expected_angle = 0
    elif curr_grid_x_f < waypoint[waypoints_hit][0] and curr_grid_y_f > waypoint[waypoints_hit][1]:
       expected_angle = -1*expected_angle
    elif curr_grid_x_f < waypoint[waypoints_hit][0] and curr_grid_y_f < waypoint[waypoints_hit][1]:
       expected_angle = -1*expected_angle
    elif curr_grid_x_f > waypoint[waypoints_hit][0] and curr_grid_y_f > waypoint[waypoints_hit][1]:
       expected_angle = 180 + expected_angle
    elif curr_grid_x_f > waypoint[waypoints_hit][0] and curr_grid_y_f < waypoint[waypoints_hit][1]:
       expected_angle = -180 + expected_angle
       
    if expected_angle < 0:
        norm_exp_ang = 2*abs(180-abs(expected_angle)) + abs(expected_angle)
    else:
        norm_exp_ang = expected_angle
    if ang < 0:
        norm_ang = 2*abs(180-abs(ang)) + abs(ang)
    else:
        norm_ang = ang
        
    if distance < 5:
       right_speed = 0
       left_speed = 0
       waypoints_hit = waypoints_hit+1
    elif abs(norm_exp_ang - norm_ang) < 8:
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
    elif math.sin(math.radians(norm_exp_ang - norm_ang)) > 0:
        right_speed = MAX_SPEED
        left_speed = -1*MAX_SPEED
    else:
        right_speed = -1*MAX_SPEED
        left_speed = MAX_SPEED
    
    print(object_to_left)

    if object_in_way:
        right_speed = -.8*MAX_SPEED
        left_speed = MAX_SPEED
    elif object_to_left:
        right_speed = .5*MAX_SPEED
        left_speed = MAX_SPEED
        
           
    #Give speed to wheel motors  
    wheels[0].setVelocity(left_speed) 
    wheels[1].setVelocity(right_speed) 
    wheels[2].setVelocity(left_speed) 
    wheels[3].setVelocity(right_speed) 
        
pass