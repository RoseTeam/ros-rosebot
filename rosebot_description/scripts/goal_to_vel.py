#!/usr/bin/env python
"""
   goal_to_vel - converts a twist message to motor commands.  Needed for navigation stack
"""
import rospy
import roslib
from geometry_msgs.msg import Pose2D, Twist, Quaternion
from nav_msgs.msg import Odometry
import math
from simulation2d import Simulation2d, coordsToVec, vecToCoords

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

        self.goal = None
        self.goal_orient = 0
        
        #self.goal = (1,2)
        #self.robot.AssBut(coordsToVec(self.goal))
        
                
    def spinOnce(self):
        #############################################################
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
        
        Simulation2d.spinOnce(self)
        
        twist = Twist()
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        twist.linear.z = 0
        twist.linear.x = 0
        twist.linear.y = 0
        
        if self.goal:

            if True:
                import copy

                deltaT = 1./self.rate

                prev_v = copy.copy(self.robot._v_)
                prev_orient = vecToCoords(self.robot.orient_,scale=1)
                self.robot.PreBouge(deltaT)
                next_orient = vecToCoords(self.robot.orient_,scale=1)
                
                twist.linear.x = norme(vecToCoords(self.robot._v_))
                
                twist.angular.z = -math.atan2(prod_vect(prev_orient,next_orient), prod_scal(prev_orient,next_orient)) / deltaT # angular velocity
                
                #print 'Goal', self.goal, self.robot.but_
                print 'orient', prev_orient, '->',  next_orient, math.atan2(prod_vect(prev_orient,next_orient), prod_scal(prev_orient,next_orient))
                rospy.loginfo("-D- x w %s %s", twist.linear.x, twist.angular.z)
                #print self.robot.orient_, self.robot._v_
                print
                self.robot.orient_ = coordsToVec(prev_orient)/1000.
                self.robot._v_ = prev_v
                
            else:
                
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
#############################################################
if __name__ == '__main__':
    """ main """
    node = GoalToVel()
    node.spin()
