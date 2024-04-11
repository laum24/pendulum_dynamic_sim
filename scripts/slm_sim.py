#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

tau_value = 0.0

#Declare Variables to be used
#SLM Parameters
k = 0.01; #friction
m = 0.75; #pendulum mass
l = 0.36; #pendulum length
a = l/2
J = 4/3*(m*(a*a))
g = 9.8;  #gravity
tau = 0.0; #entrance of the system
x1 = 0.0;  #ouput joint1
x2 = 0.0;  #output joint2

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message


#Define the callback functions
def callback_tau(msg):
     #rospy.loginfo
     global tau_value
     tau_value = msg.data

def joints():
     pubX1 = rospy.Publisher('/joint_states', Float32, queue_size=10)
     pubX2 = rospy.Publisher('/joint_states', Float32, queue_size=10)
     rate = rospy.Rate(10)
     while not rospy.is_shutdown():
          x1 = x1 + x2
          x1_rad = math.radians(x1)
          x2_dot = (1/(J+m*(a*a)) * (- (m*g*a) * math.cos(x1_rad) - k * x2 + tau_value ))
          x2 = x2 + x2_dot 
          pubX1.publish(x1)
          pubX2.publish(x2)
          rate.sleep()

  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    # Setup the Subscribers
    #rospy.Subscriber('tau_topic', Float32, callback_tau)


    #Setup de publishers
    pub = rospy.Publisher('/joint_states',Float32,queue_size=10)

    print("The SLM sim is Running")
    try:
        joints()
        #Run the node (YOUR CODE HERE)
        
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE

            #Wait and repeat
        loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node