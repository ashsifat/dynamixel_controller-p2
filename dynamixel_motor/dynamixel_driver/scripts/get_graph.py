#!/usr/bin/env python
__author__ = 'Ash'
__copyright__ = '...'

#__license__ = 'BSD'
__maintainer__ = 'Ashrarul Sifat'
__email__ = 'ashrar7@vt.edu'


import sys
#from optparse import OptionParser
import math
import sys
import errno
import time
import roslib
roslib.load_manifest('dynamixel_driver')
from dynamixel_driver import dynamixel_io
import rospy
#import dynamixel_io
from dynamixel_driver.dynamixel_const import *

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

def twos_complement(val, nbits):
    	"""Compute the 2's complement of int value val"""
    	if val < 0:
             val = (1 << nbits) + val
    	else:
             if (val & (1 << (nbits - 1))) != 0:
            	# If sign bit is set.
            	# compute negative value.
            	val = val - (1 << nbits)
    	return val

if __name__ == '__main__':
    port_name='/dev/ttyUSB0'
    port_namespace='ttyUSB0'
    baud_rate=3000000
    min_motor_id=1
    max_motor_id=25
    update_rate=5
    diagnostics_rate=1
    error_level_temp=75
    warn_level_temp=70
    readback_echo=False
    torque_on=True
    motor_id = 1
    r=0.126
    m=842/1000
    I=m*r*r
    c=r*m*9.81
    try:
        dxl_io = dynamixel_io.DynamixelIO(port_name, baud_rate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Turning torque %s for motor %d' % (torque_on, motor_id)
        if dxl_io.ping(motor_id):
#                torque_response= self.dxl_io.set_torque_enabled(2,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
		torque_response= dxl_io.set_torque_enabled(1,1)
		print "torque enable1 response", torque_response
		time.sleep(1)
                print 'done'
        else:
            print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id

	#write_address= [0x74, 0x00]
#	write_data = [0xAA, 0x02, 0x00, 0x00 ]
#	write_response = self.dxl_io.write(1,write_address, write_data)
#	print "write done, response:", write_response

	# write current goal to motor 1
	write_address= [0x66, 0x00]
	val = 15
	nbits = 16
	write_data_whole= twos_complement(val, nbits)
	write_data_L=write_data_whole & 0xFF
	write_data_H = (write_data_whole>>8) & 0xFF
	write_data = [write_data_L, write_data_H ]
	write_response = dxl_io.write(1,write_address, write_data)
	print "write done, response:", write_response
	last_time=time.time()
	write_=[]
	read_cur1=[]
	read_pos1=[]
	read_vel2=[]
	write_time=[]
	read_time1=[]
	read_time2=[]
	read_time3=[]
	torque_1=[]
	sync_write_address= [0x66, 0x00]
	last_read_vel=0
	last_read_time=last_time
	
	
	while not rospy.is_shutdown() :
#		current_time =time.time()
#		val=20*math.sin(3.1416*current_time)
#		val1=int(round(val))
#		val2=-val1;
#		write_.append(val1)
#		write_data_whole1= twos_complement(val1, nbits)
#		write_data_L1=write_data_whole1 & 0xFF
#		write_data_H1 = (write_data_whole1>>8) & 0xFF
#		#for motor2 value
#		write_data_whole2= twos_complement(val2, nbits)
#		write_data_L2=write_data_whole2 & 0xFF
#		write_data_H2 = (write_data_whole2>>8) & 0xFF
#		write_data = [write_data_L, write_data_H ]
		#write_response = dxl_io.write(1,write_address, write_data)
#		sync_write_data = [0x02, 0x00, 0x01, write_data_L1, write_data_H1, 0x02, write_data_L2, write_data_H2]
#		self.dxl_io.sync_write(sync_write_address, sync_write_data)
#		print "sync_write done, current control"
#		write_time.append(time.time())
#		#print "write done, response:", write_response
		read_response=dxl_io.read(1,DXL_PRESENT_CURRENT,2)
		read_val1= read_response[9] |  (read_response[10] << 8)
		read_val=twos_complement(read_val1, nbits)
		read_cur1.append(read_val)
		read_time1.append(read_response[13])
		read_response=dxl_io.read(1,DXL_PRESENT_POSITION,4)
		read_val3= read_response[9] |  (read_response[10] << 8)
		read_val_pos=twos_complement(read_val3, nbits)
		read_pos1.append(read_val)
		read_time3.append(read_response[15])
		read_response=dxl_io.read(1,DXL_PRESENT_VELOCITY,4)
		read_val2= read_response[9] |  (read_response[10] << 8)
		read_velocity=twos_complement(read_val2, nbits)
		read_vel2.append(read_velocity)
		read_time2.append(read_response[15])
		theta=2*3.1416*read_val_pos/4096
		tau1=I*(read_velocity-last_read_vel)/(read_response[15]-last_read_time)-c*math.sin(theta)
		last_read_vel=read_velocity
		last_read_time=read_response[15]
		torque_1.append(tau1)
#		read_response=self.dxl_io.read(2,DXL_PRESENT_CURRENT,2)
#		read_val2= read_response[9] |  (read_response[10] << 8)
#		read_val=self.twos_complement(read_val2, nbits)
#		read_2.append(read_val)
#		read_time2.append(read_response[13])
		#print "read done, response:", read_response
		
		if  (read_response[15]-last_time)>2:
			val = 0
			write_data_whole= twos_complement(val, nbits)
			write_data_L=write_data_whole & 0xFF
			write_data_H = (write_data_whole>>8) & 0xFF
			write_data = [write_data_L, write_data_H ]
			write_response = dxl_io.write(1,write_address, write_data)
			print "write done, response:", write_response
##			write_data_whole= self.twos_complement(val, nbits)
##			write_data_L=write_data_whole & 0xFF
##			write_data_H = (write_data_whole>>8) & 0xFF
#			sync_write_data = [0x02, 0x00, 0x01, val, val, 0x02, val, val]
#			self.dxl_io.sync_write(sync_write_address, sync_write_data)
#			print "sync write done", 
#			print "data write", write_
#			print "data write_time", write_time
			print "data read motor 1", read_cur1 
			print "data read_time1", read_time1
			print "data read_time2", read_time2
			print "data read_time3", read_time3
			print "torque", torque_1
			print "velocity", read_vel2
			print "read position1", read_pos1
#			print "data read motor 2",  read_2
#			print "data read_time2", read_time2
			break

	
	# set goal position on multiple motors
#	sync_write_address= [0x74, 0x00]
#	sync_write_data = [0x04, 0x00, 0x01, 0xAA, 0x00, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00 ]
#	self.dxl_io.sync_write(sync_write_address, sync_write_data)
#	print "sync_write done, position control"

	#set goal current on multiple motors
#	sync_write_address= [0x66, 0x00]
#	sync_write_data = [0x02, 0x00, 0x01, write_data_L, write_data_H, 0x02, write_data_L, write_data_H]
#	self.dxl_io.sync_write(sync_write_address, sync_write_data)
#	print "sync_write done, current control"
	#sys.exit(1)

