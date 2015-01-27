#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import Pose2D, Twist, Quaternion
from nav_msgs.msg import Odometry
import math
from rosebotnav.racine import Partie
from rosebotnav import OuvrirPartie, Vec
import PyKDL
from threading import Lock

mutex = Lock()

class mutexScope:
    
    def __enter__(self):
        mutex.acquire()
        
    def __exit__(self, type, value, traceback):
        mutex.release()

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def coordsToVec(coords):
    x,y = coords
    return Vec(x*1000, y*1000)

def vecToCoords(v,scale=.001):
    return v*scale

class Simulation2d(object):

    #############################################################
    def __init__(self, partie='robomovies_vide_blanc.par', Affiche=False):
    #############################################################
        rospy.init_node(self.__class__.__name__)
        nodename = rospy.get_name()      
        rospy.loginfo("%s started" % nodename)

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        #rospy.Subscriber('odomgazebo', Odometry, self.odomCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        
        self.map = Partie()#OuvrirPartie(nom=partie)
        
        self.map.Affiche = Affiche
        self.map.InitPreBoucle()
        self.map.AfficheMouvement = 3
        self.map.Const_IPS = False
        self.robot = iter(self.map.Selection).next()

        self.boucleIter = self.map.boucleIter(exo_controle=True)
        
        self.map.Courir = False
        self.map.ImgParSec = self.rate # TODO set the rate based on elapsed time since last call for more accuracy
        
        self.goal = None
        self.goal_orient = 0
        self.robot.AssBut(Vec(1000,1500),itineraire=True)
        

    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        self.ticks_since_target = self.ticks_since_state = self.timeout_ticks
        
        ###### main loop ######
    
        while not rospy.is_shutdown():# and self.ticks_since_target < self.timeout_ticks:
            
            self.spinOnce()
            r.sleep()
                
        
    def spinOnce(self):
        #############################################################
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
        
        self.ticks_since_target += 1
        self.ticks_since_state += 1
        
    
    def publishVelTwist(self, v, w):
        twist = Twist()
        
        twist.linear.x = v
        twist.angular.z = w

        self.pub_cmd_vel.publish(twist)

    #############################################################
    def odomCallback(self,msg):
    #######################################
        ######################
        #rospy.loginfo("-D- odomCallback: %s" % str(msg))
        
        with mutexScope():
        
            self.ticks_since_state = 0
    
            pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            vdirecte = msg.twist.twist.linear.x* 1000.
            vn = msg.twist.twist.linear.y * 1000.
            
            vit = self.robot.orient_ * vdirecte + vn * self.robot.orient_.nml
            
            self.map.Selection = set([self.robot])
            self.robot.pos = coordsToVec(pos)
            self.robot._eff_ = vit - self.robot._v_ 
            self.robot._v_ = vit 
            qt = Quaternion ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w );
            self.orient = quat_to_angle(qt)
            self.robot.orient_ = Vec(math.cos(self.orient), math.sin(self.orient))
            
            self.w = msg.twist.twist.angular.z
            
            if not self.map.Affiche:
                if hasattr(self, 'v_idea'):
                    print 'odom idea', self.v_idea,       'w %.3f'% self.w_idea
                    print 'odom real', Vec(vdirecte, vn), 'w %.3f'% self.w
                    
                
    
            #rospy.loginfo("-D- odomCallback: orient %.2f (%.2f, %.2f)" %( self.orient, pos[0], pos[1]))
            #print 'robot.orient_', self.robot.orient_, self.robot.pos
        
    #############################################################
    def goalCallback(self,msg):
    #############################################################
        with mutexScope():
            self.ticks_since_target = 0
            
            self.goal = [msg.x, msg.y]
            self.goal_orient = msg.theta
    
            self.robot.AssBut(coordsToVec(self.goal), itineraire=True)
            rospy.loginfo("-D- goalCallback: %s" % str(msg))
        
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    node = Simulation2d()
    node.spin()
