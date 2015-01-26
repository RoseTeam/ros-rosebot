#!/usr/bin/env python
"""
   goal_to_vel - converts a twist message to motor commands.  Needed for navigation stack
"""
import rospy
import roslib
from geometry_msgs.msg import Pose2D, Twist, Quaternion
from nav_msgs.msg import Odometry
import math
from simulation2d import Simulation2d, coordsToVec, vecToCoords, mutexScope

def norme(vec):
    return math.sqrt(vec[0]*vec[0] + vec[1]*vec[1])

def diff_vec(u,v):
    return u[0]-v[0], u[1]-v[1]

def prod_scal(u,v):
    return u[0]*v[0] + u[1]*v[1]

def prod_vect(u,v):
    return u[0]*v[1] - u[1]*v[0]


class GoalToVel(Simulation2d):

    #############################################################
    def __init__(self):
    #############################################################
        Simulation2d.__init__(self)

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('goal', Pose2D, self.goalCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)
        #rospy.Subscriber('odomduino', Odometry, self.odomCallback)
        
            
    def publishCmdVel(self):
        
        twist = Twist()
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        twist.linear.z = 0
        twist.linear.x = 0
        twist.linear.y = 0
        
        if False:
            twist.angular.z = 1.5
              
        elif True:
            with mutexScope():
                deltaT = 1./self.rate
                pos = self.robot.pos
                #prev_v = copy.copy(self.robot._v_)
                #prev_orient = self.robot.orient_ * 1 # copie
                self.robot.PreBouge(deltaT)
                #next_orient = self.robot.orient_
                twist.linear.x = self.robot._v_ *  self.robot.orient_ * 0.001
                #self.robot.Avance(deltaT,deltaT,0)
                #self.robot.pos = pos
                
                twist.angular.z = self.robot.w
            
            #print 'Goal', self.goal, self.robot.but_
            #print self.robot.orient_, self.robot._v_
            #self.robot.orient_ = coordsToVec(prev_orient)/1000.
            #self.robot._v_ = prev_v
            if not self.map.Affiche:
                self.v_wish = self.robot._v_* 1
                self.w_wish = self.robot.w

            
        elif self.goal:
            
            orient_vec = vecToCoords(self.robot.orient_,scale=1.)
            
            vit_angle = 1.5
            v_max = .5

            pos = vecToCoords(self.robot.pos)
        
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
                    twist.angular.z = vit_angle * ( 1 if prod_vect(orient_vec,diff_goal) > 0 else -1)
                    
            elif abs(self.goal_orient - self.orient) <= 1e-3: 
                twist.angular.z = vit_angle * ( 1 if (self.goal_orient - self.orient < 0) else -1)
            
            else:
                self.goal = None
                rospy.loginfo("-D- goal reached")

        print " v %.3f w %.3f"%( twist.linear.x, twist.angular.z)
        print
                
        self.pub_cmd_vel.publish(twist)
        

    def odomCallback(self, msg):
        Simulation2d.odomCallback(self, msg)
        self.publishCmdVel()
        
         
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    node = GoalToVel()
    node.spin()
