#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import Pose2D, Twist, Quaternion
from nav_msgs.msg import Odometry
import math
from aura import OuvrirPartie, Vec
import PyKDL

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def coordsToVec(coords):
    x,y = coords
    return Vec(y*1000,-x*1000)

def vecToCoords(v,scale=.001):
    return -v.y*scale, v.x*scale

class Simulation2d(object):

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node(self.__class__.__name__)
        nodename = rospy.get_name()      
        rospy.loginfo("%s started" % nodename)

        #rospy.Subscriber('odomgazebo', Odometry, self.odomCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        
        self.map = OuvrirPartie(nom='robomovies_vide.par')
        self.map.Affiche = False
        self.map.InitPreBoucle()
        
        self.map.Const_IPS = False
        self.robot = iter(self.map.Selection).next()
        
        self.boucleIter = self.map.boucleIter(exo_controle=True)
        self.map.Courir = False
        self.map.ImgParSec = self.rate # TODO set the rate based on elapsed time since last call for more accuracy
            

    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.ticks_since_state = self.timeout_ticks
    
        ###### main loop ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown():# and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
                
            idle.sleep()
        
    def spinOnce(self):
        #############################################################
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
        
        self.ticks_since_target += 1
        self.ticks_since_state += 1
        
        
    #############################################################
    def odomCallback(self,msg):
    #######################################
        ######################
        #rospy.loginfo("-D- odomCallback: %s" % str(msg))
        self.ticks_since_state = 0
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        vit = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        
        self.robot.pos = coordsToVec(pos)
        self.robot._v_ = coordsToVec(vit)
        #print 'vit', vit
        qt = Quaternion ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w );
        self.orient = quat_to_angle(qt)
        self.robot.orient_ = Vec(math.sin(self.orient),-math.cos(self.orient))
        self.w_vit = msg.twist.twist.angular.z
        #rospy.loginfo("-D- odomCallback: orient %.2f (%.2f, %.2f)" %( self.orient, pos[0], pos[1]))
        #print 'robot.orient_', self.robot.orient_, self.robot.pos

         
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    node = Simulation2d()
    node.spin()
