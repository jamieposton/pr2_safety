#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from pr2_msgs.msg import PowerBoardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s." , data.data)
    if data.data:
            print "Disabling Motors"
            cmd = "disable"
            power_control = rospy.ServiceProxy('power_board/control', PowerBoardCommand)
            for breaker in range(3):
                power_cmd = PowerBoardCommandRequest()
                power_cmd.breaker_number = breaker
                power_cmd.command = cmd
                power_cmd.serial_number = 0
                power_control(power_cmd)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("buttonBool", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()