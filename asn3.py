#!/usr/bin/env python
import roslib
import rospy
import time
#import csv
from fw_wrapper.srv import *
#from csv import writer

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

LEFT_SHOULDER, RIGHT_SHOULDER = 3, 2
BACK_LEFT_LEG, BACK_RIGHT_LEG = 18, 6
HEAD = 1
DMS_PORT_1, DMS_PORT_2 = 1, 6

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
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#  wrapper function to call service to sync motor wheels
def syncMotorWheelSpeeds(number_ids, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', 0, 0, number_ids, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e





# OUR ROBOT FUNCTIONS

def turn_degrees(degrees):
    # MOTOR WHEEL VALUES
    # COUNTER-CLOCKWISE: 0 to 1023
    # CLOCKWISE: 1024 to 2047

    if degrees == 90:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 2.5:  # for i in range(8):
            # testing_gyro()
            syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG),
                                 (400, 400, 400, 400))  # TODO maybe 500

        # STOP
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (0, 0, 1024, 1024))

    elif degrees == -90:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 2.5:  # for i in range(8):
            syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG),
                                 (1024 + 400, 1024 + 400, 1024 + 400, 1024 + 400))

        while 1 == getIsMotorMovingCommand(RIGHT_SHOULDER) == getIsMotorMovingCommand(BACK_RIGHT_LEG):
            print("right motors still moving")
            pass

        # STOP
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (1024, 1024, 0, 0))

    elif degrees == 180:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 5.:  # for i in range(8):
            # testing_gyro()
            syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG),
                                 (400, 400, 400, 400))

        # STOP
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (0, 0, 1024, 1024))


# from prev lab
def move_head(direction):
    if direction == "LEFT":
        setMotorTargetPositionCommand(HEAD, 512 - 300)  # min 212
    elif direction == "RIGHT":
        setMotorTargetPositionCommand(HEAD, 512 + 300)  # max 812
    elif direction == "FRONT":
        setMotorTargetPositionCommand(HEAD, 512)
    else:
        print("invalid direction")

# from prev lab
def object_detected(port):
    end = time.time() + 2
    while time.time() < end:
        sum = 0
        for i in range(10):
            reading = getSensorValue(port)
            print(reading)
            sum += reading
            
        avg_reading = sum/10
        rospy.loginfo("Sensor value at port %d: %f", port, avg_reading)
        return 1

    return 0
def calibrate_DMS():
    setMotorTargetPositionCommand(HEAD, 512)
    c = 0
    err1 = 0
    err2 = 0
    while (c < 200):
        reading1 = getSensorValue(DMS_PORT_1)
        reading2 = getSensorValue(DMS_PORT_2)
        err1 += reading1
        err2 += reading2
        c += 1

    err1 = err1 / 200
    err2 = err2 / 200
    diff = err1 - err2
    return diff

def add_new_row(filename, new_row):
    with open (filename, 'ab') as file:
        writer = csv.writer(file)
        writer.writerow(new_row)

def collect_training_data(diff):
    # with open('training.csv', 'a') as csvfile:
    fields = ['readings', "angle"]
    res = {}
    # turn robot left/right
    # turn(90)  # right for now

    # once done turning in given dir
    # OG READINGS: get the sensor readings from robot after it completes a turn
    port1_reading = getSensorValue(DMS_PORT_1)
    port2_reading = getSensorValue(DMS_PORT_2)
    # import to csv at end
    
    # then, turn head in same dir
    # setMotorTargetSpeed(HEAD, 40)
    # move_head("RIGHT")

    # while head is turning, 
    # when sensor readings are in same threshold s.t. they're considered to be equivalent
    # stop moving head

    # while getIsMotorMovingCommand(HEAD):
    #     ir1 = getSensorValue(IR_PORT_1)
    #     ir2 = getSensorValue(IR_PORT_2)
    if(port1_reading - port2_reading + diff < 0):
        thresh = -1000000
        i = 0
        while(thresh < 10):
            setMotorTargetPositionCommand(HEAD, 512 - i)
            ir1 = getSensorValue(DMS_PORT_1)
            ir2 = getSensorValue(DMS_PORT_2)
            thresh = ir1 - ir2 + diff
            #print(thresh)
            i -= 2
        angle = (float(i)/300)*90
        print(angle)
    else:
        thresh = 1000000
        i = 0
        while(thresh > -10):
            setMotorTargetPositionCommand(HEAD, 512 - i)
            ir1 = getSensorValue(DMS_PORT_1)
            ir2 = getSensorValue(DMS_PORT_2)
            thresh = ir1 - ir2 + diff
            # print(thresh)
            i += 2
        angle = (float(i)/300)*90
        print(angle)
    

    setMotorTargetSpeed(HEAD, 0)
    data = [str(port1_reading), str(port2_reading), str(angle)]
    return(data)
    
  
    #data = "hello"

    # ANGLE: subtract final head pos from origin head pos

    # (flip these depending on l/r direction)
    # diff = getMotorPositionCommand(HEAD) - 512
    # angle = 'todo'

    # add_new_row('training.csv', data)
    # import angle to csv
    # writer = csv.writer(csvfile)
    # writer.writerows(data)


    # write og sensor readings & angle to turn by to csv
    # res['readings'] = (port1_reading, port2_reading)
    # res['angle'] = angle
    # writer_object = writer(csvfile)
    # writer_object.writerow([port1_reading, port2_reading, angle])
    # csvfile.close()
    # writer = DictWriter(csvfile, fields)
    # writer.writeheader()
    # writer.writerow({'readings': (port1_reading, port2_reading)})
    # writer.writerow({'angle': angle})
    
    
    # dictwriter_object.writerow({'readings' : (port1_reading, port2_reading), 'angle' : angle})

    # writer = csv.DictWriter(csvfile, fieldnames=fields)
    
    # writer.writerows(res)






# Main function
if __name__ == "__main__":
    # with open("test.txt", "w+") as filea:
    #     filea.write("hello")
    #     filea.flush()

    # print("ELLO")
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")
    
    setMotorTargetSpeed(HEAD, 1023)
    move_head("FRONT")
    while getIsMotorMovingCommand(HEAD):
        pass

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # setMotorTargetSpeed(HEAD, 50)
        #move_head("FORWARD")
        # while getIsMotorMovingCommand(HEAD):
        #     pass



        # diff = calibrate_DMS()
        # print(diff)        
        res = collect_training_data(-57)
        print(res)
        rospy.sleep(4)

        # turn_degrees(90)
        # port1_reading = object_detected(IR_PORT_1)
        # port2_reading = object_detected(IR_PORT_2)


        # rospy.loginfo("Sensor value at port %d: %f", 5, port1_reading)
        # rospy.loginfo("Sensor value at port %d: %f", 5, port2_reading)
        # break
        
        # sleep to enforce loop rate
        r.sleep()
