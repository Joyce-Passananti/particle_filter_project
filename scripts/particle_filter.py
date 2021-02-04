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

from likelihood_field import LikelihoodField

import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random


def compute_prob_zero_centered_gaussian(dist, sd):
    """ A helper funtion that takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



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

        # inialize our map, likelihood field and occupancy field
        self.map = OccupancyGrid()
        self.occupancy_field = None
        self.likelihood_field = LikelihoodField()


        # the number of particles used in the particle filter
        self.num_particles = 100

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

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data

        # self.occupancy_field = OccupancyField(data)

    

    def initialize_particle_cloud(self):
        
        # gets the upper and lower bounds of x and y such that 
        #   the resultant bounding box contains all of the obstacles in the map
        ((x_lower, x_upper), (y_lower, y_upper)) = self.likelihood_field.get_obstacle_bounding_box()
        
        # we'll initialize our cloud of particles of form [x, y, theta] of size: num_particles
        i=0
        while i < self.num_particles:
            # randomly choose a position on the map for our particle
            p = Pose()
            p.position.x = np.random.uniform(x_lower, x_upper)
            p.position.y = np.random.uniform(y_lower, y_upper)

            # get Occupancy grid index of the particle
            x = (p.position.x - self.map.info.origin.position.x) // self.map.info.resolution            
            y = (p.position.y - self.map.info.origin.position.y) // self.map.info.resolution
            index = x + y*self.map.info.width

            # check that the index is not occupied by an obstacle
            if (self.map.data[int(index)] != -1):
                q = quaternion_from_euler(0, 0, np.random.uniform(0, 2 * np.pi))
                p.orientation.z = q[2]
                p.orientation.w = q[3]

                # initialize the new particle, where all will have the same weight (1.0)
                new_particle = Particle(p, 1.0)

                # append the particle to the particle cloud
                self.particle_cloud.append(new_particle)
                
                # increase our count of created particles
                i+= 1

        self.normalize_particles()
        
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # Get the sum of all particle weights
        total_weight = 0
        for p in self.particle_cloud:
            total_weight += p.w
        
        # Normalize each particle's weight accordingly
        for p in self.particle_cloud:
            p.w /= total_weight



    def publish_particle_cloud(self):

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



    def resample_particles(self):

         # TODO
         print("todo: resample")



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
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

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
         # TODO
         print("todo: estimate robot pose")


    
    def update_particle_weights_with_measurement_model(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        angles = [0, 90, 180, 270]
        
        for p in self.particle_cloud:
            q = 1
            for a in angles:

                # grab the observation at time t and laser range finder index k
                # set the distance to the max (3.5m) if the value is greater than the max value
                z_t_k = min(data.ranges[a], 3.5)

                # get the orientation of the robot from the quaternion (index 2 of the Euler angle)
                theta = euler_from_quaternion([
                    p.pose.orientation.x, 
                    p.pose.orientation.y, 
                    p.pose.orientation.z, 
                    p.pose.orientation.w])[2]

                # translate and rotate the laser scan reading from the robot to the particle's 
                # location and orientation
                x_z_t_k = p.pose.position.x + z_t_k * math.cos(theta + (a * math.pi / 180.0))
                y_z_t_k = p.pose.position.y + z_t_k * math.sin(theta + (a * math.pi / 180.0))

                # find the distance to the closest obstacle
                closest_obstacle_dist = self.likelihood_field.get_closest_obstacle_distance(x_z_t_k, y_z_t_k)

                # compute the probability based on a zero-centered gaussian with sd = 0.1
                prob = compute_prob_zero_centered_gaussian(closest_obstacle_dist, 0.1)

                # multiply all sensor readings together
                q = q * prob
                
                # set the weight of the particle
                p.w = q

                # print everything out so we can see what we get and debug (REMOVE BEFORE SUBMISSION)
                # print(p)
                # print("Scan[", a, "]: ", z_t_k)
                # print("\t", a, ": [", x_z_t_k, ", ", y_z_t_k, "]")
                # print("\tobs dist: ", closest_obstacle_dist)
                # print("\tprob: ", prob, "\n")
        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # get the current and previous positions of the robot
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        
        # calculate the distance moved and angle rotated
        x_moved = curr_x - old_x
        y_moved = curr_y - old_y
        yaw_rotated = curr_yaw - old_yaw
        
        for i, p in enumerate(self.particle_cloud):
            # get the x,y position and orientation of the particle from the quaternion (index 2 of the Euler angle)
            p_x = p.pose.position.x
            p_y = p.pose.position.y
            p_yaw = euler_from_quaternion([
                p.pose.orientation.x, 
                p.pose.orientation.y, 
                p.pose.orientation.z, 
                p.pose.orientation.w])[2]

            # Calculate the particle's new coordinates and orientation based on its angle difference from the robot's orientation
            rotation = p_yaw - old_yaw
            new_x = p_x + math.sin((math.pi / 180.0)*rotation) * y_moved + math.cos((math.pi / 180.0)*rotation) * x_moved
            new_y = p_y + math.sin((math.pi / 180.0)*rotation) * x_moved + math.cos((math.pi / 180.0)*rotation) * y_moved
            new_yaw = p_yaw + yaw_rotated
            new_q = quaternion_from_euler(0, 0 , new_yaw)

            #Create a new Pose with the updated particle position and orientation
            new_pose = Pose()
            new_pose.position.x = new_x
            new_pose.position.y = new_y
            new_pose.orientation.z = new_q[2]
            new_pose.orientation.w = new_q[3]

            self.particle_cloud[i] = Particle(new_pose, p.w)
        
        


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









