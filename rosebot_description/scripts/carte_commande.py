#!/usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import Pose2D
import math
from simulation2d import Simulation2d, vecToCoords


class CarteCommande(Simulation2d):

    #############################################################
    def __init__(self):
    #############################################################

        Simulation2d.__init__(self)
        self.map.Affiche = True
        self.map.InitPreBoucle()
        
        self.pub_goal = rospy.Publisher('goal', Pose2D, queue_size=10)
        
        RobotClass = type(self.robot)
        origMethod = RobotClass.AssBut
        def newMethod(agent,pos,**kwargs):
            origMethod(agent,pos,**kwargs)
            self.pubGoal(vecToCoords(pos))
            
        RobotClass.AssBut = newMethod
    
    def spinOnce(self):
        
        Simulation2d.spinOnce(self)
        
        self.boucleIter.next()

    #######################################################
    def pubGoal(self,pos):
    #######################################################
        
        rospy.loginfo("publishing goal (%0.3f,%0.3f)" %(pos[0],pos[1]))
        pose2d = Pose2D()
        pose2d.x = pos[0]
        pose2d.y = pos[1]
        pose2d.theta = 0
        
        self.pub_goal.publish( pose2d )

         
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    node = CarteCommande()
    node.spin()
