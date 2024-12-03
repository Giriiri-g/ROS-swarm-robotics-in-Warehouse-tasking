#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading

def spin_thread():
    rospy.spin()

Command_Queue = [{"Node":"R1", "ID":'21', "command":"X1"}, {"Node":"R2", "ID":'22', "command":"X5"}]
Stage1_Queue = []
Stage2_Queue = []
Stage3_Queue = []
Stage4_Queue = []

def ack_response_r1_callback(ACKR):
    global Stage2_Queue, Stage3_Queue
    for ind, command in enumerate(Stage2_Queue.copy()):
        if command["ID"] == ACKR.data.split('_')[1]:
            rospy.loginfo(f"Recieved Ack Response from Node R1: Command ID={ACKR.data}")
            Stage3_Queue.append(Stage2_Queue.pop(ind))
            return

def ack_response_r2_callback(ACKR):
    global Stage2_Queue, Stage3_Queue
    for ind, command in enumerate(Stage2_Queue.copy()):
        if command["ID"] == ACKR.data.split('_')[1]:
            rospy.loginfo(f"Recieved Ack Response from Node R2: Command ID={ACKR.data}")
            Stage3_Queue.append(Stage2_Queue.pop(ind))
            return

def robot1_position_callback(POSR):
    global Stage4_Queue
    for ind, command in enumerate(Stage4_Queue.copy()):
        if command["ID"] == POSR.data.split('_')[1]:
            rospy.loginfo(f"Recieved Response from Node R1: Command ID={POSR.data}")
            if POSR.data.split('_')[2] == 'failure':
                rospy.logerr(f"Fatal Error: R1 Aborted Command={POSR.data.split('_')[0]}, Command ID={POSR.data.split('_')[1]}")
            else:
                rospy.loginfo(f"Succesful: R1 Executed Command={POSR.data.split('_')[0]}, Command ID={POSR.data.split('_')[1]}")
            Stage4_Queue.pop(ind)
            return
    
def robot2_position_callback(POSR):
    global Stage4_Queue
    for ind, command in enumerate(Stage4_Queue.copy()):
        if command["ID"] == POSR.data.split('_')[1]:
            rospy.loginfo(f"Recieved Response from Node R2: Command ID={POSR.data}")
            if POSR.data.split('_')[2] == 'failure':
                rospy.logerr(f"Fatal Error: R2 Aborted Command={POSR.data.split('_')[0]}, Command ID={POSR.data.split('_')[1]}")
            else:
                rospy.loginfo(f"Succesful: R2 Executed Command={POSR.data.split('_')[0]}, Command ID={POSR.data.split('_')[1]}")
            Stage4_Queue.pop(ind)
            return


def core_node():
    global Command_Queue, Stage1_Queue, Stage2_Queue, Stage3_Queue, Stage4_Queue

    rospy.init_node('Command_and_control_server', anonymous=True)

    # Publishers
    ack_pub_r1 = rospy.Publisher('/ack_to_r1', String, queue_size=10)
    ack_pub_r2 = rospy.Publisher('/ack_to_r2', String, queue_size=10)
    position_pub_r1 = rospy.Publisher('/position_to_r1', String, queue_size=10)
    position_pub_r2 = rospy.Publisher('/position_to_r2', String, queue_size=10)

    # Subscribers
    rospy.Subscriber('/ack_response_r1', String, ack_response_r1_callback)
    rospy.Subscriber('/ack_response_r2', String, ack_response_r2_callback)
    rospy.Subscriber('/robot1_position', String, robot1_position_callback)
    rospy.Subscriber('/robot2_position', String, robot2_position_callback)

    spin_thread = threading.Thread(target=spin_thread)
    spin_thread.start()
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if Command_Queue != []:
            Stage1_Queue.extend(Command_Queue)
            Command_Queue = []
        index = []
        for ind, command in enumerate(Stage1_Queue):
            if command["Node"] == "R1":
                rospy.loginfo(f"Publishing ACK ID: {command['ID']} to Node R1.")
                ack_pub_r1.publish(f"ACK_{command['ID']}")
                index.append(ind)
            else:
                rospy.loginfo(f"Publishing ACK ID: {command['ID']} to Node R2.")
                ack_pub_r2.publish(f"ACK_{command['ID']}")
                index.append(ind)
        for ind in index[::-1]:
            Stage2_Queue.append(Stage1_Queue.pop(ind))
        index=[]
        for ind, command in enumerate(Stage3_Queue):
            if command["Node"] == "R1":
                rospy.loginfo(f"Publishing Command {command['ID']} to Node R1: {command['command']}")
                position_pub_r1.publish(f"{command['command']}_{command['ID']}")
                index.append(ind)
            else:
                rospy.loginfo(f"Publishing Command {command['ID']} to Node R2: {command['command']}")
                position_pub_r2.publish(f"{command['command']}_{command['ID']}")
                index.append(ind)
        for ind in index[::-1]:
            Stage4_Queue.append(Stage3_Queue.pop(ind))

        rate.sleep()

if __name__ == '__main__':
    try:
        core_node()
    except rospy.ROSInterruptException:
        pass