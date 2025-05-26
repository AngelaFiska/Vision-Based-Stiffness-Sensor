#!/usr/bin/env python
import rospy
import struct, SocketServer
import roslib; roslib.load_manifest('sf_nibridge')
from std_msgs.msg import Float64MultiArray
from socket import *
filename="forces.txt"

this_HOST, this_PORT = '192.168.1.7', 11000 # fill this

class MutantServer(SocketServer.UDPServer):
    def __init__(self, server_address, RequestHandlerClass):
        SocketServer.UDPServer.__init__(self, server_address, RequestHandlerClass)

	# publisher for data received through UDP
        self.force_pub = rospy.Publisher('forces_Nano17', Float64MultiArray)
        print 'server online'

class CustomUDPHandler(SocketServer.DatagramRequestHandler):
    def handle(self):
	raw_data = self.request[0]
        msg = Float64MultiArray()
 
	# decode your data below
	dat = raw_data.decode("utf-8")        
	#print dat.split('x')
	data=dat.split('x')
        msg.data=[ float(data[0]),float(data[1]),float(data[2]),float(data[3]),float(data[4]),float(data[5]),float(data[6]),float(data[7]),float(data[8])]
	file.write(str(data[0])+ "," + str(data[1])+ "," + str(data[2])+ "," + str(data[3])+ ","+ str(data[4])+ "," + str(data[5])+ "," + str(data[6])+ ","+ str(data[7])+ ","+str(data[8])+ "\n")
	server.force_pub.publish(msg)
	#print(float(data[0]), float(data[1]),float(data[2]),float(data[3]),float(data[4]),float(data[5]),float(data[6]),float(data[7]),float(data[8]))
if __name__ == '__main__':
    try:
        rospy.init_node('NIBridge')
	file=open(filename,'w')
        print "starting udp server..."
        server = MutantServer((this_HOST, this_PORT), CustomUDPHandler)
        server.serve_forever()
    except rospy.ROSInterruptException:
        file.close
        pass
