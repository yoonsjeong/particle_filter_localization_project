#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample, normal, choice
import math

from random import randint, random, sample, uniform
from likelihood_field import LikelihoodField
import random

from copy import deepcopy

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

def draw_random_sample(particles, weights, num):
    count = 0
    bins = []

    # create bins
    for w in weights:
        count += w
        bins.append(count)
    
    # select num
    out = []
    for _ in range(num):
        val = uniform(0, count)
        for j,b in enumerate(bins):
            if val < b:
                new_p = Particle(particles[j].pose, particles[j].w)
                out.append(new_p)
                break
    return out

class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    def __str__(self):
        theta = euler_from_quaternion([
            self.pose.orientation.x, 
            self.pose.orientation.y, 
            self.pose.orientation.z, 
            self.pose.orientation.w])[2]
        return ("Particle: [" + str(self.pose.position.x) + ", " + str(self.pose.position.y) + ", " + str(theta) + "]")



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        #initialize likelihood field
        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        rospy.sleep(2)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True

    def draw_random_sample(self):
        """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
        We recommend that you fill in this function using random_sample.
        """
        prob_weights = []
        positions = []
        for p in self.particle_cloud:
            prob_weights.append(p.w)
            positions.append(p.pose)
        randomList = random.choices(positions, weights = prob_weights, k=self.num_particles)
        return deepcopy(randomList)

    def get_map(self, data):

        self.map = data
    
    def get_particle(self): # This function randomly draws coordinates from the grid width and height and checks the occupancy grid that they are valid, 
    # and then converts them to coordinates on the map
        """
        Gets particles in the form
            [x, y, yaw]
        """
        width, height = self.map.info.width - 1, self.map.info.height - 1
        grid = self.map.data

        resolution = 20 / width
        
        coords = []
        while len(coords) < self.num_particles: # draws coordinates from the grid width and height until we have a valid set of particle coordinates of a size specified. 
        # Converts the coordinates according to the formula found here: https://answers.ros.org/question/312326/conversion-from-cell-grid-to-map-point/
            x_coord, y_coord = randint(0, width), randint(0, height) 
            x_mod = (x_coord * self.map.info.resolution) + self.map.info.origin.position.x + self.map.info.resolution /2
            y_mod = (y_coord * self.map.info.resolution) + self.map.info.origin.position.y + self.map.info.resolution/2

            if grid[x_coord + y_coord * (width + 1)] < 0: continue
            else: 
                theta = uniform(0, 2*np.pi) # A random orientation is chosen 
                coords.append([x_mod,y_mod,theta]) 
        return coords

    def initialize_particle_cloud(self): # This function initializes particles according to the valid coordinate data from get_particle
        
        coords = self.get_particle()
        
        self.particle_cloud = []
        for coord in coords:
            pos_x, pos_y, theta = coord
            or_x, or_y, or_z, or_w = quaternion_from_euler(0.0, 0.0, theta)
            
            pose = Pose()
            pose.position.x = pos_x
            pose.position.y = pos_y
            pose.position.z = 0

            pose.orientation = Quaternion()
            pose.orientation.x = or_x
            pose.orientation.y = or_y
            pose.orientation.z = or_z
            pose.orientation.w = or_w

            particle = Particle(pose, 1.0) # Everyt particle is initialized with weight 1
            self.particle_cloud.append(particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self): # This function makes the weight of all the particles sum to 1 

        weight_sum = 0
        for p in self.particle_cloud: #Sum the weights
            weight_sum += p.w

        for i, p in enumerate(self.particle_cloud): #Divide each particle by the weight sum. Create a new Particle to avoid reference errors
            new_p = Particle(p.pose, p.w / weight_sum)
            self.particle_cloud[i] = new_p

    def publish_particle_cloud(self):

        print("hello")
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self): #This function resamples particles according to the normalized weights, choosing higher probability particles for the next iteration of the particle cloud

        weight_list = []
        for p in self.particle_cloud: #Create a weight list 
            weight_list.append(p.w)

        new_particle_cloud = draw_random_sample(self.particle_cloud, weight_list, self.num_particles)
        self.particle_cloud = new_particle_cloud


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(1.0))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            self.curr_x = self.odom_pose.pose.position.x
            self.old_x = self.odom_pose_last_motion_update.pose.position.x
            self.curr_y = self.odom_pose.pose.position.y
            self.old_y = self.odom_pose_last_motion_update.pose.position.y
            self.curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            self.old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(self.curr_x - self.old_x) > self.lin_mvmt_threshold or 
                np.abs(self.curr_y - self.old_y) > self.lin_mvmt_threshold or
                np.abs(self.curr_yaw - self.old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self): #This function updates the robots estimated position 
        
        x_sum = 0 
        y_sum = 0
        theta_sum = 0

        for p in self.particle_cloud: # Sum position and orientation values
            x_sum = x_sum + p.pose.position.x
            y_sum = y_sum + p.pose.position.y
            quaternion = (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,p.pose.orientation.w)
            theta_sum = theta_sum + euler_from_quaternion(quaternion)[2] #, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)

        x_aver = x_sum / len(self.particle_cloud) # Take the averages of position and orientation values
        y_aver = y_sum / len(self.particle_cloud)
        theta_aver = theta_sum / len(self.particle_cloud)

        p = Pose() # Create a new pose an assign the robot_estimate to, which holds the position and orientation averages
        p.position.x = x_aver
        p.position.y = y_aver
        p.position.z = 0
        q = quaternion_from_euler(0.0, 0.0, theta_aver)
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        self.robot_estimate = p 

    
    def update_particle_weights_with_measurement_model(self, data): #This function implements the likilehood field range finder algorithm from class, updating the particle weights

        cardinal_directions_idxs = [0, 45 , 90, 135, 180, 225, 270, 315]
        new_particle_cloud = []
        for i,p in enumerate(self.particle_cloud): #for every particle in the cloud update the weight
            q = 1
            for idx in cardinal_directions_idxs: # for every direction, calculate part of the cumulative q value 
                ztk = data.ranges[idx]
                if ztk >= 3.5: # if their is no object scanned by the LIDAR, set the ztk to its max range of 3.5
                    q = q * 1e-30
                    continue

                # boilerplate vars
                particle_x, particle_y = p.pose.position.x, p.pose.position.y
                quat_in = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
                theta_z = euler_from_quaternion(quat_in)[2]
                theta_z += idx * np.pi / 180 # add curr cardinal direction (in rads)

                # algorithm
                x_ztk = particle_x + (ztk * np.cos(theta_z))
                y_ztk = particle_y + (ztk * np.sin(theta_z))
                dist = self.likelihood_field.get_closest_obstacle_distance(x_ztk, y_ztk)
                gauss = compute_prob_zero_centered_gaussian(dist, sd=.5) # The sd value was determined by playing with different values and optimizing 
                
                if math.isnan(gauss): # particles outside the box recieve a zero probability and will not be selected in resampling 
                    q = 0
                    break
                else:  
                    q *= gauss            
            self.particle_cloud[i].w = q

    def update_particles_with_motion_model(self): 
        """ This code uses the provided odometry values to update the particle cloud.
        delta_x, delta_y, delta_q represent the change in x-position, y-position, and
        yaw-position (in quaternion form)
        """
        
        delta_x, delta_y = self.curr_x - self.old_x, self.curr_y - self.old_y #Values for the change in position measured by the odometer
        delta_yaw = self.curr_yaw - self.old_yaw
        

        new_particle_cloud = []
        for i,p in enumerate(self.particle_cloud): #Update the position and yaw for every particle

            new_yaw = get_yaw_from_pose(p.pose) + delta_yaw # The new_yaw, which is used to transform the odometry updates, is a combination of the particle yaw and the robots odometry value

            new_pose = Pose()

            new_pose.position.x = p.pose.position.x
            new_pose.position.x += delta_x * math.cos(new_yaw) # Multiply the odometry change in x acccording to the specific particles orientation by taking the cos of the new yaw
            new_pose.position.x += normal(0, 0.1) # noise

            new_pose.position.y = p.pose.position.y
            new_pose.position.y += delta_y * math.sin(new_yaw) # Multiply the odometry change in y acccording to the specific particles orientation by taking the cos of the new yaw 
            new_pose.position.y += normal(0, 0.1) # noise

            
            new_yaw += normal(0, 0.1) # noise
            new_q = quaternion_from_euler(0.0, 0.0, new_yaw)

            new_pose.orientation.x = new_q[0]
            new_pose.orientation.y = new_q[1]
            new_pose.orientation.z = new_q[2]
            new_pose.orientation.w = new_q[3]
            new_p = Particle(new_pose, p.w)
            
            self.particle_cloud[i] = new_p

if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()






