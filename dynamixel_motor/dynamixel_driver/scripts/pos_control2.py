#!/usr/bin/env python
__author__ = 'Ash'
__copyright__ = '...'

#__license__ = 'BSD'
__maintainer__ = 'Ashrarul Sifat'
__email__ = 'ashrar7@vt.edu'

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
#import roslib; roslib.load_manifest('numpy_tutorials')
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
import numpy as np
from std_msgs.msg import Byte
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


def pos_control2():
    pub = rospy.Publisher('current', Byte, queue_size=1)
    rospy.init_node('pos_control2', anonymous=True)
    rate = rospy.Rate(500) # 10hz
    angles=numpy.loadtxt(athena_pseudo_walk_joint_angles,dtype=<type 'float'>,comments='#', delimiter=None, converters=None)
    print 'Turning left torque %s for motor %d' % (torque_on, motor_id)
    if dxl_io_0.ping(motor_id):
#		torque_response= dxl_io.set_torque_enabled(2,0)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io.set_torque_enabled(1,0)
#		print "torque enable1 response", torque_response
#		time.sleep(1) 

#		opset1 = dxl_io.set_operating_mode(1,0)
#		print "set op 1 response", opset1
#		time.sleep(1)
#		opset2 = dxl_io.set_operating_mode(2,0)
#		print "set op 2 response", opset2
#		time.sleep(1)
                torque_response= dxl_io_0.set_torque_enabled(5,1)
		print "torque enable2 response", torque_response
		time.sleep(1) 
		torque_response= dxl_io_0.set_torque_enabled(6,1)
		print "torque enable1 response", torque_response
		time.sleep(1)
                print 'done'
    else:
    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id
     
#    torque_response= dxl_io_0.set_torque_enabled(5,1)
#    print "torque enable2 response", torque_response
#    time.sleep(1) 
#    torque_response= dxl_io_0.set_torque_enabled(6,1)
#    print "torque enable1 response", torque_response
#    time.sleep(1)
#    print 'done'
      
#    print 'Turning right torque %s for motor %d' % (torque_on, motor_id_3)
#    if dxl_io_3.ping(motor_id_3):
#                torque_response= dxl_io_1.set_torque_enabled(7,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io_1.set_torque_enabled(8,1)
#		print "torque enable1 response", torque_response
#		time.sleep(1)
#                print 'done'
#    else:
#    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id_3
    	        
    print 'Turning right torque %s for motor %d' % (torque_on, motor_id_3)
    if dxl_io_3.ping(motor_id_3):
                torque_response= dxl_io_3.set_torque_enabled(7,1)
		print "torque enable2 response", torque_response
		time.sleep(1) 
		torque_response= dxl_io_3.set_torque_enabled(8,1)
		print "torque enable1 response", torque_response
		time.sleep(1)
                print 'done'
    else:
    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id_3

#    torque_response= dxl_io_3.set_torque_enabled(7,1)
#    print "torque enable2 response", torque_response
#    time.sleep(1) 
#    torque_response= dxl_io_3.set_torque_enabled(8,1)
#    print "torque enable1 response", torque_response
#    time.sleep(1)
#    print 'done'

#    write_address= [0x74, 0x00]
#    write_data = [0xAA, 0x02, 0x00, 0x00 ]
#    write_response = dxl_io.write(1,write_address, write_data)
#    print "write done, response:", write_response
    #sync_write_address= [0x66, 0x00]  		#current
    sync_write_address= [0x74, 0x00]		#position
	# write current goal to motor 1
    #write_address= [0x66, 0x00]
    val = 2048
    nbits = 32
#	write_data_whole= twos_complement(val, nbits)
#	write_data_L=write_data_whole & 0xFF
#	write_data_H = (write_data_whole>>8) & 0xFF
#	write_data = [write_data_L, write_data_H ]
#	write_response = dxl_io.write(1,write_address, write_data)
#	print "write done, response:", write_response
#    val1=val
#    val2=-val1;
#	#write_.append(val1)
#    write_data_whole1= twos_complement(val1, nbits)
#    write_data_L1=write_data_whole1 & 0xFF
#    write_data_H1 = (write_data_whole1>>8) & 0xFF
	#for motor2 value
#    write_data_whole2= twos_complement(val2, nbits)
#    write_data_L2=write_data_whole2 & 0xFF
#    write_data_H2 = (write_data_whole2>>8) & 0xFF
#    sync_write_data = [0x02, 0x00, 0x01, write_data_L1, write_data_H1, 0x02, write_data_L2, write_data_H2]
#    dxl_io.sync_write(sync_write_address, sync_write_data)
#    print "sync_write done, current control"
	# set goal position on multiple motors
    val1=int(round(val))
    val2=val1;
		#write_.append(val1)
    write_data_whole1= twos_complement(val1, nbits)
#		write_data_L1 = write_data_whole1 & 0xFF
#		write_data_H1 = (write_data_whole1>>8) & 0xFF
    write_data_11 = write_data_whole1 & 0xFF
    write_data_21 = (write_data_whole1>>8) & 0xFF
    write_data_31 = (write_data_whole1>>16) & 0xFF
    write_data_41 = (write_data_whole1>>32) & 0xFF
		#for motor2 value
    write_data_whole2 = twos_complement(val2, nbits)
#		write_data_L2=write_data_whole2 & 0xFF
#		write_data_H2 = (write_data_whole2>>8) & 0xFF
    write_data_12 = write_data_whole2 & 0xFF
    write_data_22 = (write_data_whole2>>8) & 0xFF
    write_data_32 = (write_data_whole2>>16) & 0xFF
    write_data_42 = (write_data_whole2>>24) & 0xFF
    sync_write_address= [0x74, 0x00]
    #sync_write_data = [0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 ]
    sync_write_data = [0x04, 0x00, 0x05, write_data_11, write_data_21, write_data_31, write_data_41, 0x06, write_data_12,write_data_22, write_data_32, write_data_42 ]
    dxl_io_0.sync_write(sync_write_address, sync_write_data)
    sync_write_data = [0x04, 0x00, 0x07, write_data_11, write_data_21, write_data_31, write_data_41, 0x08, write_data_12,write_data_22, write_data_32, write_data_42 ]
    dxl_io_3.sync_write(sync_write_address, sync_write_data)
    print "sync_write done, position control"
    time.sleep(5)
    last_time=time.time()
#    write_=[]
#    read_cur1=[]
#    read_cur2=[]
#    read_pos1=[]
#    read_vel2=[]
#    write_time=[]
#    read_timecur1=[]
#    read_timecur2=[]
#    read_time2=[]
#    read_time3=[]
#    torque_1=[]
    last_read_vel=0
    i=0
    last_read_time=last_time
	#rr = rospy.Rate(500) # 10hz
#	while not rospy.is_shutdown() :
#	#read position
#		read_response=dxl_io.read(1,DXL_PRESENT_POSITION,4)
#		read_val_pos= read_response[9] | (read_response[10] << 8)| (read_response[11] << 16)| (read_response[12] << 24)
#		print read_val_pos

    while not rospy.is_shutdown() :
		current_time =time.time()
		val=200*math.sin(3.1416*current_time)		
		# byte write
#		val=int(val)
#		write_data_whole= twos_complement(val, nbits)
#		write_data_L=write_data_whole & 0xFF
#		write_data_H = (write_data_whole>>8) & 0xFF
#		write_data = [write_data_L, write_data_H ]
#		write_response = dxl_io.write(1,write_address, write_data)
#		print "write done, response:", write_response
#		write_data_whole= twos_complement(-val, nbits)
#		write_data_L=write_data_whole & 0xFF
#		write_data_H = (write_data_whole>>8) & 0xFF
#		write_data = [write_data_L, write_data_H ]
#		write_response = dxl_io.write(2,write_address, write_data)
#		print "write done, response:", write_response
		val1=2048+int(round(val))
		val2=2048-int(round(val))
		#val2=-val1;
		#write_.append(val1)
		write_data_whole1= twos_complement(val1, nbits)
#		write_data_L1 = write_data_whole1 & 0xFF
#		write_data_H1 = (write_data_whole1>>8) & 0xFF
		write_data_11 = write_data_whole1 & 0xFF
		write_data_21 = (write_data_whole1>>8) & 0xFF
		write_data_31 = (write_data_whole1>>16) & 0xFF
		write_data_41 = (write_data_whole1>>24) & 0xFF
		#for motor2 value
		write_data_whole2 = twos_complement(val2, nbits)
#		write_data_L2=write_data_whole2 & 0xFF
#		write_data_H2 = (write_data_whole2>>8) & 0xFF
		write_data_12 = write_data_whole2 & 0xFF
		write_data_22 = (write_data_whole2>>8) & 0xFF
		write_data_32 = (write_data_whole2>>16) & 0xFF
		write_data_42 = (write_data_whole2>>24) & 0xFF
		#write_data = [write_data_L, write_data_H ]
		#write_response = dxl_io.write(1,write_address, write_data)
#		sync_write_data = [0x02, 0x00, 0x01, write_data_L1, write_data_H1, 0x02, write_data_L2, write_data_H2]
#		dxl_io.sync_write(sync_write_address, sync_write_data)
#		print "sync_write done, current control"
		# set goal position on multiple motors
		sync_write_data = [0x04, 0x00, 0x05, write_data_11, write_data_21, write_data_31, write_data_41, 0x06, write_data_12,write_data_22, write_data_32, write_data_42 ]
		dxl_io_0.sync_write(sync_write_address, sync_write_data)
		sync_write_data = [0x04, 0x00, 0x07, write_data_11, write_data_21, write_data_31, write_data_41, 0x08, write_data_12,write_data_22, write_data_32, write_data_42 ]
		dxl_io_3.sync_write(sync_write_address, sync_write_data)
		print "sync_write done, position control"
		i=i+1
#		write_time.append(time.time())
#		#print "write done, response:", write_response
#########################################################################
#		read_response=dxl_io.read(1,DXL_PRESENT_CURRENT,2)
#		read_val1= read_response[9] | (read_response[10] << 8)
#		read_val=twos_complement(read_val1, nbits)
#		print "reading motor 1 current ", read_val
#		read_cur1.append(read_val)
#		read_timecur1.append(read_response[13])
#############################################################################
#		sync_read_address= [0x84, 0x00]
#		sync_read_data = [0x04, 0x00, 0x01, 0x02 ]
#		sync_read_response=dxl_io.sync_read(sync_read_address, sync_read_data)
#		print "sync_read response" , sync_read_response
##################################################################################
#		read_response=dxl_io.read(2,DXL_PRESENT_CURRENT,2)
#		read_val1= read_response[9] | (read_response[10] << 8)
#		read_val=twos_complement(read_val1, nbits)
#		print read_val
#		read_cur2.append(read_val)
#		read_timecur2.append(read_response[13])
###############	#read position
		read_response=dxl_io_0.read(5,DXL_PRESENT_POSITION,4)
		print read_response
#		read_val_pos= read_response[9] | (read_response[10] << 8)| (read_response[11] << 16)| (read_response[12] << 24)		
#		read_val_pos=twos_complement(read_val_pos, 32)
#		print "position read",read_val_pos
#		read_pos1.append(read_val_pos)
#		read_time3.append(read_response[15])
#		#read vel
#		read_response=dxl_io.read(1,DXL_PRESENT_VELOCITY,4)
#		read_val2= read_response[9] |  (read_response[10] << 8)
#		read_velocity=twos_complement(read_val2, nbits)
#		print "present velocity", read_velocity
#		read_vel2.append(read_velocity)
#		read_time2.append(read_response[15])
#		theta=2*3.1416*read_val_pos/4096
#		tau1=I*(read_velocity-last_read_vel)*0.023981/(read_response[15]-last_read_time)#-c*math.sin(theta)
#		print math.sin(theta)		
#		print (read_velocity-last_read_vel)
#		#print (read_response[15]-last_read_time)
#		last_read_vel=read_velocity
#		last_read_time=read_response[15]
#		torque_1.append(tau1)
#		read_response=self.dxl_io.read(2,DXL_PRESENT_CURRENT,2)
#		read_val2= read_response[9] |  (read_response[10] << 8)
#		read_val=self.twos_complement(read_val2, nbits)
#		read_2.append(read_val)
#		read_time2.append(read_response[13])
		#print "read done, response:", read_response
		pub.publish(val)
		
		if  (current_time-last_time)>5:
			val = 2048
#			write_data_whole= twos_complement(val, nbits)
#			write_data_L=write_data_whole & 0xFF
#			write_data_H = (write_data_whole>>8) & 0xFF
#			write_data = [write_data_L, write_data_H ]
#			write_response = dxl_io.write(1,write_address, write_data)
#			print "write done, response:", write_response
#			write_data_whole= twos_complement(val, nbits)
#			write_data_L=write_data_whole & 0xFF
#			write_data_H = (write_data_whole>>8) & 0xFF
#			sync_write_data = [0x02, 0x00, 0x01, val, val, 0x02, val, val]
#			dxl_io.sync_write(sync_write_address, sync_write_data)
#			print "sync write done", i
			val1=int(round(val))#write_.append(val1)
    			write_data_whole1= twos_complement(val1, nbits)
#		write_data_L1 = write_data_whole1 & 0xFF
#		write_data_H1 = (write_data_whole1>>8) & 0xFF
    			write_data_1 = write_data_whole1 & 0xFF
    			write_data_2 = (write_data_whole1>>8) & 0xFF
    			write_data_3 = (write_data_whole1>>16) & 0xFF
    			write_data_4 = (write_data_whole1>>32) & 0xFF
			sync_write_data = [0x04, 0x00, 0x05, write_data_1,write_data_2,write_data_3,write_data_4, 0x06, write_data_1, write_data_2, write_data_3, write_data_4 ]
			dxl_io_0.sync_write(sync_write_address, sync_write_data)
			sync_write_data = [0x04, 0x00, 0x07, write_data_1,write_data_2,write_data_3,write_data_4, 0x08, write_data_1, write_data_2, write_data_3, write_data_4 ]
			dxl_io_3.sync_write(sync_write_address, sync_write_data)
			print "sync_write done, position control", i
#			torque_response= dxl_io.set_torque_enabled(2,0)
#			print "torque enable2 response", torque_response
			# byte write
#			write_data_whole= twos_complement(val, nbits)
#			write_data_L=write_data_whole & 0xFF
#			write_data_H = (write_data_whole>>8) & 0xFF
#			write_data = [write_data_L, write_data_H ]
#			write_response = dxl_io.write(1,write_address, write_data)
#			print "write done, response:", write_response
#			write_data_whole= twos_complement(val, nbits)
#			write_data_L=write_data_whole & 0xFF
#			write_data_H = (write_data_whole>>8) & 0xFF
#			write_data = [write_data_L, write_data_H ]
#			write_response = dxl_io.write(2,write_address, write_data)
#			print "write done, response:", write_response
			#time.sleep(1) 
#			torque_response= dxl_io.set_torque_enabled(1,0)
#			print "torque enable1 response", torque_response
##			print "data write", write_
##			print "data write_time", write_time
#			print "data read motor 1curr", read_cur1
#			print "data read motor 2curr", read_cur2 
#			print "data read_timecurr", read_timecur1,read_timecur2
#			print "data read_timevel", read_time2
#			print "data read_timepos", read_time3
#			print "torque", torque_1
#			print "velocity", read_vel2
#			print "read position1", read_pos1
#			print "data read motor 2",  read_2
#			print "data read_time2", read_time2
			break

    		rate.sleep()

if __name__ == '__main__':
    port_name0='/dev/ttyUSB0'
    port_namespace0='ttyUSB0'
    port_name1='/dev/ttyUSB1'
    port_namespace1='ttyUSB1'
    port_name2='/dev/ttyUSB2'
    port_namespace2='ttyUSB2'
    port_name3='/dev/ttyUSB3'
    port_namespace3='ttyUSB3'
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
    motor_id_3 = 7
    r=0.126
    m=842
    I=m*r*r
    c=r*m*9.81
    #i=0
#    sync_write_address= [0x66, 0x00]
    try:
        dxl_io_0 = dynamixel_io.DynamixelIO(port_name0, baud_rate)
        #dxl_io_1 = dynamixel_io.DynamixelIO(port_name1, baud_rate)
        #dxl_io_2 = dynamixel_io.DynamixelIO(port_name2, baud_rate)
        dxl_io_3 = dynamixel_io.DynamixelIO(port_name3, baud_rate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
	pos_control2()
