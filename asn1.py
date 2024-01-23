#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed


# -------- OUR CONSTANTS --------
LEFT_SHOULDER, RIGHT_SHOULDER = 3, 2
LEFT_PAW, RIGHT_PAW = 5, 4  # MOTOR 4 IS LABELLED AS 14 ON ROBOT
BACK_LEFT_LEG, BACK_RIGHT_LEG = 7, 6
HEAD = 1
L_PAW_CENTER, R_PAW_CENTER = 512 + 250, 512 - 250
PORT = 1

# TO MOVE LEFT PAW UP, VALUES MORE THAN 512
# TO MOVE RIGHT PAW UP, VALUES LESS THAN 512

# TO 

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, Se:
        print "Service call failed: %s"%e

# PART 1: TODO 
def walk():
	# CORRECT WALKING CODE
	setMotorTargetPositionCommand(LEFT_SHOULDER, 512 + 100)
	setMotorTargetPositionCommand(RIGHT_SHOULDER, 512 - 50)
	setMotorTargetPositionCommand(LEFT_PAW, L_PAW_CENTER + 50)
	setMotorTargetPositionCommand(RIGHT_PAW, R_PAW_CENTER + 50)
	setMotorTargetPositionCommand(BACK_LEFT_LEG, 512 + 50)
	setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512 + 50)

	
	setMotorTargetPositionCommand(LEFT_SHOULDER, 512 + 50)
	setMotorTargetPositionCommand(RIGHT_SHOULDER, 512 - 100)
	setMotorTargetPositionCommand(LEFT_PAW, L_PAW_CENTER - 50)
	setMotorTargetPositionCommand(RIGHT_PAW, R_PAW_CENTER - 50)
	setMotorTargetPositionCommand(BACK_LEFT_LEG, 512 - 50)
	setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512 - 50)
	# END OF CORRECT WALKING CODE


# PART 2: TODO 
def turn(degrees):
	# +90 is right, -90 is left, 180 is turning around

	# first put motors in default position with helper function
	# maybe put a back leg flat to the ground so that the robot leans to one side
	# once robot is leaning to a side, move the opposite side's shoulder/paw forward?
	
	# figure this out for turning to one side; reverse values to turn other way;
	# perform twice to turn 180?
	setMotorTargetPositionCommand(LEFT_PAW, 512)	
	setMotorTargetPositionCommand(RIGHT_PAW, 512)
	if degrees == 90 or degrees == 180:
		# start going 180 by default
		for i in range(12):
			setMotorTargetPositionCommand(LEFT_PAW, 512 - 150)
			setMotorTargetPositionCommand(RIGHT_PAW, 512)
			setMotorTargetPositionCommand(BACK_LEFT_LEG, 512 + 20)
			setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512 - 150)

			setMotorTargetPositionCommand(LEFT_PAW, 512)
			setMotorTargetPositionCommand(RIGHT_PAW, 512 + 150)
			setMotorTargetPositionCommand(BACK_LEFT_LEG, 512 + 150)
			setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512 - 20)
			# stop if we actually wanted to stop at 90 degrees
			if degrees == 90 and i == 5:
				break
	elif degrees == -90:
		for i in range(12):

			setMotorTargetPositionCommand(RIGHT_PAW, 512)
			setMotorTargetPositionCommand(LEFT_PAW, 512 - 150)
			setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512 - 150)
			setMotorTargetPositionCommand(BACK_LEFT_LEG, 512 + 20)
			
			setMotorTargetPositionCommand(RIGHT_PAW, 512 + 150)
			setMotorTargetPositionCommand(LEFT_PAW, 512)
			setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512 - 20)
			setMotorTargetPositionCommand(BACK_LEFT_LEG, 512 + 150)
		

# PART 3: TODO 
# obstacle detection

# PART 4: TODO
# wall following

# PART 5
def turn_when_blocked(blocked):
	# TODO: "Blocked should be signified by the detection of an obstacle at 15cm or less"

	# a sensor is blocked if value is 1, not blocked if value is 0
	# maybe treat order as [left, front, right] ?
	
	# turn around if all sensors blocked
	if blocked[0] == blocked[1] == blocked[2] == 1:
		turn(180)
	# turn left if front and right sensors blocked
	elif blocked[1] == blocked[2] == 1:
		turn(-90)
	# turn right if front and left sensors blocked
	elif blocked[1] == blocked[0] == 1:
		turn(90)


def stand_up():
	setMotorTargetPositionCommand(HEAD, 512)

	setMotorTargetPositionCommand(BACK_LEFT_LEG, 512)
	setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512)

	setMotorTargetPositionCommand(LEFT_SHOULDER, 512 + 100)
	setMotorTargetPositionCommand(RIGHT_SHOULDER, 512 - 100)

	setMotorTargetPositionCommand(LEFT_PAW, L_PAW_CENTER)
	setMotorTargetPositionCommand(RIGHT_PAW, R_PAW_CENTER)

def move_head():
	setMotorTargetPositionCommand(HEAD, 512 + 256)
	while getIsMotorMovingCommand(HEAD) == 1:
		print("head is still moving ...right?")

	

	setMotorTargetPositionCommand(HEAD, 512 - 256)
	while getIsMotorMovingCommand(HEAD) == 1:
		print("head is still moving ...left?")

def detect():
	blocked = [0, 0, 0]
	setMotorTargetPositionCommand(HEAD, 512)

	setMotorTargetPositionCommand(BACK_LEFT_LEG, 512)
	setMotorTargetPositionCommand(BACK_RIGHT_LEG, 512)

	setMotorTargetPositionCommand(LEFT_SHOULDER, 512 + 100)
	setMotorTargetPositionCommand(RIGHT_SHOULDER, 512 - 100)

	setMotorTargetPositionCommand(LEFT_PAW, 512 + 350)
	setMotorTargetPositionCommand(RIGHT_PAW, 512 - 350)

	# check left side
	setMotorTargetPositionCommand(HEAD, 512 - 300)
	while getIsMotorMovingCommand(HEAD) == 1:
		print("head is still moving ...left?")
	sensor_reading = getSensorValue(PORT)
	rospy.loginfo("Sensor value at port %d: %f", PORT, sensor_reading)
	if sensor_reading > 35:
		blocked[0] = 1
	
	# check infront
	setMotorTargetPositionCommand(HEAD, 512)
	while getIsMotorMovingCommand(HEAD) == 1:
		print("head is still moving to center")
	sensor_reading = getSensorValue(PORT)
	rospy.loginfo("Sensor value at port %d: %f", PORT, sensor_reading)
	if sensor_reading > 35:
		blocked[1] = 1

	# check right side
	setMotorTargetPositionCommand(HEAD, 512 + 300)
	while getIsMotorMovingCommand(HEAD) == 1:
		print("head is still moving ...right?")
	sensor_reading = getSensorValue(PORT)
	rospy.loginfo("Sensor value at port %d: %f", PORT, sensor_reading)
	if sensor_reading > 35:
		blocked[2]=1
	

	print(blocked)
	turn_when_blocked(blocked)

	
	

def all_limbs_moving():
	return 1 == getIsMotorMovingCommand(LEFT_PAW) == getIsMotorMovingCommand(RIGHT_PAW) == getIsMotorMovingCommand(LEFT_SHOULDER) == getIsMotorMovingCommand(RIGHT_SHOULDER) == getIsMotorMovingCommand(BACK_LEFT_LEG) == getIsMotorMovingCommand(BACK_RIGHT_LEG) 


# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    stand_up()
    while all_limbs_moving():
	print("still trying to stand")
	pass

    detect()

    #turn(90) # turn right
    #turn(180) # turn around
    #turn(-90) # turn left
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 1
        sensor_reading = getSensorValue(port)
        rospy.loginfo("Sensor value at port %d: %f", port, sensor_reading)
	"""
	15cm is 35-40 reading
	
	"""

	#if not all_limbs_moving():
	#walk()
	#move_head()
	
        # sleep to enforce loop rate
        r.sleep()






