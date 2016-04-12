#!/usr/bin/env python
# license removed for brevity
import roslib

roslib.load_manifest('pr2_safety')
import rospy

from std_msgs.msg import Bool
from pr2_msgs.msg import PowerBoardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest
import socket

if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("134.197.40.237", 2048))
    print "Connecting"
    sock.listen(1)
    (cSock, cAddr) = sock.accept()
    print "Connection accepted"
    msg = ""
    while msg != "END":
        msg = cSock.recv(1)
        #print msg
        if msg != "F":
            print "Disabling Motors"
            cmd = "disable"
            power_control = rospy.ServiceProxy('power_board/control', PowerBoardCommand)
            for breaker in range(3):
                power_cmd = PowerBoardCommandRequest()
                power_cmd.breaker_number = breaker
                power_cmd.command = cmd
                power_cmd.serial_number = 0
                power_control(power_cmd)
            msg = "END"
    
    cSock.close()
    sock.close()