#!/usr/bin/env python
"""
   goal_to_vel - converts a twist message to motor commands.  Needed for navigation stack
"""
import twist_to_motors
import rospy
import roslib
from geometry_msgs.msg import Pose2D, Twist, Quaternion
from nav_msgs.msg import Odometry
import math
#from transform_utils import quat_to_angle
import PyKDL
from aura import OuvrirPartie, Vec

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def norme(vec):
    return math.sqrt(vec[0]*vec[0] + vec[1]*vec[1])
def diff_vec(u,v):
    return u[0]-v[0], u[1]-v[1]
def prod_scal(u,v):
    return u[0]*v[0] + u[1]*v[1]
def prod_vect(u,v):
    return u[0]*v[1] - u[1]*v[0]

def coordsToVec(coords):
    x,y = coords
    return Vec(y*1000,-x*1000)

def vecToCoords(v,scale=.001):
    return -v.y*scale, v.x*scale

class GoalToVel:

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("goal_to_vel")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)


        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('goal', Pose2D, self.goalCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.goal = None
        #print 'init'
        self.map = OuvrirPartie(nom='robomovies_vide.par')
        #self.map.Affiche = False
        self.map.Const_IPS = True
        self.robot = iter(self.map.Selection).next()
        self.map.InitPreBoucle()
        self.boucleIter = self.map.boucleIter(exo_controle=True)
        self.map.Courir = True
        self.map.ImgParSec = self.rate # TODO set the rate based on elapsed time since last call for more accuracy
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.ticks_since_state = self.timeout_ticks
    
        ###### main loop  ######
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
        
        twist = Twist()
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        twist.linear.z = 0
        twist.linear.x = 0
        twist.linear.y = 0
        
        
        pos = vecToCoords(self.robot.pos)
        orient_vec = vecToCoords(self.robot.orient_,scale=1.)
        prev_v = vecToCoords(self.robot._v_)
        
        self.boucleIter.next()
        next_v = vecToCoords(self.robot._v_)

        if self.goal:

            if False:
    
                twist.linear.x = next_v.nor
                twist.angular.z = math.atan2(prev_v ^ next_v, prev_v * next_v) * self.rate # angular velocity
            
            else:
                
                vit_angle = 1.5
                v_max = .5
            
                diff_goal = diff_vec(self.goal, pos)
                dist_goal = norme(diff_goal)
                
                if dist_goal > 1e-2:
                    
                    goal_orient = diff_goal[0]/dist_goal, diff_goal[1]/dist_goal                
                    diff_orient = diff_vec(goal_orient, orient_vec)
                    
                    if norme(diff_orient) <= 5e-1:
                        
                        twist.linear.x = v_max 
                        
                    if norme(diff_orient) <= 1e-1:
                        vit_angle   = 0.1 #min( vit_angle, diff_orient)
                    
                    if norme(diff_orient) >= 1e-2:
    
                        # realign needed
                        twist.angular.z = vit_angle * ( 1 if prod_vect(orient_vec,diff_goal) <= 0 else -1)
                        
                elif abs(self.goal_orient - self.orient) <= 1e-3: 
                    twist.angular.z = vit_angle * ( 1 if (self.goal_orient - self.orient > 0) else -1)
                
                else:
                    self.goal = None
                    rospy.loginfo("-D- goal reached")
            
            self.pub_cmd_vel.publish(twist)
            

    #############################################################
    def goalCallback(self,msg):
    #############################################################
        self.ticks_since_target = 0
        self.goal = [msg.x, msg.y]
        self.goal_orient = msg.theta
        #rospy.loginfo("-D- orient: %.2f w_vit %.2f" % (self.orient, self.w_vit))
        #rospy.loginfo("-D- pos   : %.2f,%.2f " % (self.pos[0], self.pos[1]))
        #self.Path = self.mesh_map.pathFind_py(self.pos_est, self.goal))
        self.robot.AssBut(coordsToVec(self.goal))
        rospy.loginfo("-D- goalCallback: %s" % str(msg))


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
        
        qt = Quaternion ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w );
        self.orient = quat_to_angle(qt)
        self.robot.orient_ = Vec(math.sin(self.orient),-math.cos(self.orient))
        self.w_vit = msg.twist.twist.angular.z
        #rospy.loginfo("-D- odomCallback: orient %.2f (%.2f, %.2f)" %( self.orient, pos[0], pos[1]))

         
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    node = GoalToVel()
    node.spin()
