#!/home/alice/anaconda3/envs/py38/bin/python

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random

from likelihood_field import LikelihoodField

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

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

# transform vec in global coordinate to local coordinate
def transform_point(vec, yaw):
    return(np.cos(yaw)*vec[0] + np.sin(yaw)*vec[1], -np.sin(yaw)*vec[0]+np.cos(yaw)*vec[0])

def inverse_transform_point(vec, yaw):
    return(np.cos(yaw)*vec[0] -np.sin(yaw)*vec[1], np.sin(yaw)*vec[0]+np.cos(yaw)*vec[1])

def normalize_angle(yaw):
    while yaw < 0:
        yaw += 2*np.pi
    while yaw >= 2*np.pi:
        yaw -= 2*np.pi
    return yaw

def make_pose(x, y, yaw):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0.0
    p.orientation = Quaternion()
    q = quaternion_from_euler(0.0, 0.0, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

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
        self.likelihood_field:LikelihoodField = LikelihoodField()    

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.num_particles = 200
        self.particle_weights = [1.0/self.num_particles] * self.num_particles
        self.particle_cloud = []
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # noise moodel
        self.pos_sigma = 0.1
        self.angle_sigma = 0.01
        self.motion_noise = np.diag([self.pos_sigma, self.pos_sigma, self.angle_sigma])
        
        # Setup publishers and subscribers
        self.particles_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # intialize the particle cloud
        self.initialize_particle_cloud()
        self.initialized = True


    def get_map(self, data):
        self.map = data

    def initialize_particle_cloud(self):
        # we'll initialize these 4 particles of form [x, y, theta]
    
        self.particle_cloud = []
        self.particle_weights = []
        (x_range, y_range) = self.likelihood_field.get_obstacle_bounding_box()
        rands = np.random.rand(self.num_particles, 3)
        rands *= np.array([x_range[1]-x_range[0], y_range[1]-y_range[0], 2.0*np.pi])
        rands += np.array([x_range[0], y_range[0], 0.0])
       #
       # 
        for i in range(self.num_particles):
            p = make_pose(rands[i][0], rands[i][1], rands[i][2])
            new_particle = Particle(p, 1.0)
            self.particle_cloud.append(new_particle)
            self.particle_weights.append(new_particle.w)

        self.normalize_particles()
        self.publish_particle_cloud()

    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        sum_weights = np.sum(self.particle_weights)
        print("weights sum to : ", sum_weights)
        for (i, pt) in enumerate(self.particle_cloud):
            pt.w /= sum_weights
            self.particle_weights[i] = pt.w


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
        assert(len(self.particle_cloud) == self.num_particles)
        assert(np.abs(np.sum(self.particle_weights) - 1.0) < 1.0e-7)
        self.particle_cloud = draw_random_sample(self.particle_cloud, self.particle_weights, self.num_particles)
        
        # noise model: gaussian
        for i, pt in enumerate(self.particle_cloud):
            gaussian_noise = np.random.multivariate_normal([0.0, 0.0, 0.0], self.motion_noise, self.num_particles)
            yaw = get_yaw_from_pose(pt.pose)
            pt.pose = make_pose(pt.pose.position.x + gaussian_noise[i, 0], pt.pose.position.y + gaussian_noise[i, 1], yaw + gaussian_noise[i, 2])
            
        assert(len(self.particle_cloud) == self.num_particles)

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
        avg_x, avg_y, avg_yaw = 0.0, 0.0, 0.0
        for pt in self.particle_cloud:
            avg_x += pt.pose.position.x
            avg_y += pt.pose.position.y
            avg_yaw += get_yaw_from_pose(pt.pose)
        avg_yaw = normalize_angle(avg_yaw/self.num_particles)
        avg_x /= self.num_particles
        avg_y /= self.num_particles
        self.robot_estimate = make_pose(avg_x, avg_y, avg_yaw)

    
    def update_particle_weights_with_measurement_model(self, data):
        angles = [i*len(data.ranges)//4 for i in range(4)] # 0, pi/2, pi, 3*pi/2
        (x_range, y_range) = self.likelihood_field.get_obstacle_bounding_box()
        assert(len(self.particle_cloud) == self.num_particles)        
        for i, pt in enumerate(self.particle_cloud):
            # first prune if out of bounds
            if pt.pose.position.x > x_range[1] or pt.pose.position.x < x_range[0] or pt.pose.position.y > y_range[1] or pt.pose.position.y < y_range[0]:
                pt.w = 0.0
                self.particle_weights[i] = 0.0
                continue
             
            q = 1.0
            for scan_idx in angles:
                _, _, pt_angle = euler_from_quaternion([pt.pose.orientation.x, pt.pose.orientation.y, pt.pose.orientation.z, pt.pose.orientation.w])
                z_scan = data.ranges[scan_idx]
                if z_scan == math.inf: 
                    #continue
                    z_scan = data.range_max
                
                # need not scale angles here
                scan_angle = data.angle_min + scan_idx * data.angle_increment
                x_expected = pt.pose.position.x + z_scan * np.cos(pt_angle + scan_angle)
                y_expected = pt.pose.position.y + z_scan * np.sin(pt_angle + scan_angle)
                
                # if out of map...
                if x_expected > x_range[1] or x_expected < x_range[0] or y_expected > y_range[1] or y_expected < y_range[0]: 
                    x_expected = pt.pose.position.x
                    y_expected = pt.pose.position.y
                
                dist = self.likelihood_field.get_closest_obstacle_distance(x_expected, y_expected)
                if dist == np.nan or dist == float('nan'):
                    # a sensible replacement for dist when (x,y) is out of map.
                    dist = z_scan if z_scan != math.inf else data.range_max
                
                p = compute_prob_zero_centered_gaussian(dist, 0.1)
                q *= p
                
            pt.w = q
            self.particle_weights[i] = q


    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        dx = self.odom_pose.pose.position.x - self.odom_pose_last_motion_update.pose.position.x
        dy = self.odom_pose.pose.position.y - self.odom_pose_last_motion_update.pose.position.y
        th = np.arctan2(dy, dx)
        d = np.sqrt(dx**2 + dy**2)
        
        th1 = th - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        th2 = get_yaw_from_pose(self.odom_pose.pose) - th
        
        q0 = np.array([0.0, 0.0, 0.0, 0.0])
        # there is no 'yaw addition' here.
        for pt in self.particle_cloud:
            q0[0] = pt.pose.orientation.x
            q0[1] = pt.pose.orientation.y
            q0[2] = pt.pose.orientation.z
            q0[3] = pt.pose.orientation.w
            q0 = quaternion_multiply(quaternion_from_euler(0.0, 0.0, th1), q0)
            _, _, yaw = euler_from_quaternion(q0)
            
            (world_dx, world_dy) = inverse_transform_point([d, 0.0], yaw)
            pt.pose.position.x += world_dx
            pt.pose.position.y += world_dy
            
            q0 = quaternion_multiply(quaternion_from_euler(0.0, 0.0, th2), q0)
            pt.pose.orientation = Quaternion()
            pt.pose.orientation.x = q0[0]
            pt.pose.orientation.y = q0[1]
            pt.pose.orientation.z = q0[2]
            pt.pose.orientation.w = q0[3]
            

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_particle_cloud()
            r.sleep()

if __name__=="__main__":
    pf = ParticleFilter()
    pf.run()
    #rospy.spin()









