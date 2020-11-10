import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp1 = 1
kp2 = 0.01
kp3 = 1

ki1 = 1
ki2 = 0.0001
ki3 = 0.01

kd1 = 1
kd2 = 0.01
kd3 = 1

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

# CALLBACKS --------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    erro1=0
    I1=0
    I2=0
    I3=0
    erro2=0
    erro3=0
    state = 'state1'
    msg = Twist()
    '''
    if state == 'initial':
        setpoint1 = 2.683991025         #,1.887759912) talvez seta um pouco pra tras de jeito
        
        position = odom.pose.pose.position
        dist = setpoint1 - position.x #math.sqrt((setpoint[0] - position.x)**2 + (setpoint[1] - position.y) **2)
        error1 = dist
        
        P1 = kp1*error1
        I1 = ki1*error1 + I1 #ki1*error1
        D1 = kd1*(error1 - erro1)
        control1 = P1+I1+D1
        erro1 = error1
        
        
        
        if control1>1:
            control1 = 1
        elif control1 < -1:
            control1 = -1
        print(state)
        msg.linear.x = control1
        state = 'state1'
        
    '''    
    if state == 'state1':
        yaw = getAngle(odom)
        msg.angular.z = 0.5
        scan_len = len(scan.ranges)
        print ("scan len")
        print (scan_len)
        if scan_len > 0:
            msg.angular.z = 0.5
            point = min(scan.ranges[scan_len-10 : scan_len+10])
            msg.angular.z = 0
            print ("point")
            print (point)
            print (scan.ranges[scan_len-10])
            print (scan.ranges[scan_len+10])
            #interpolando
            setpoint2 = (200*((point - scan.ranges[scan_len-10])/(scan.ranges[scan_len+10] - scan.ranges[0]))) - 100
            print (setpoint2)
            error2 = (setpoint2 - yaw)
    
            if abs(error2) > 180:
                if setpoint2 < 0:
                    error2 += 360 
                else:
                    error2 -= 360
            P2 = kp2*error2
            I2 = (ki2*error2) + I2 #ki1*error1
            D2 = kd2*(error2 - erro2)
            control2 = P2+I2+D2
            erro2 = error2 
                    
        
        else:
            control2 = 0
        msg.angular.z = control2
        
        state = 'state2'
                    
    if state == 'state2':
        setpoint3 = 0.5
    
        scan_len = len(scan.ranges)
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])

            error3 = -(setpoint3 - read)
        
            P3 = kp3*error3
            I3 = (ki3*error3) + I3 #ki1*error1
            D3 = kd3*(error3 - erro3)
            control3 = P3+I3+D3
            erro3 = error3
                
            if control3 > 1:
                control3 = 1
            elif control3 < -1:
                control3 = -1
        else:
            control3 = 0        
        
        print (state)
        msg.linear.x = control3
        state = 'initial'
  
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()