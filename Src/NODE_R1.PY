#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# Movement dictionary for R1
translate = {
    "X1": {"I1": "S"},
    "I1": {"X1": "W", "I2": "S", "R1": "D"},
    "I2": {"I1": "W", "I3": "S", "I4": "D"},
    "I3": {"I2": "W", "R2": "D", "X2": "S"},
    "I4": {"I2": "W", "I5": "S", "R3": "D"},
    "I5": {"I4": "W", "I6": "D", "I7": "A", "I8": "S"},
    "I6": {"I5": "W", "R5": "D", "X3": "S"},
    "I7": {"I5": "D", "R4": "W", "X4": "S"},
    "I8": {"I5": "W", "I9": "S", "R6": "D"},
    "I9": {"I8": "W", "I10": "S", "I11": "D"},
    "I10": {"I9": "W", "R7": "A", "X5": "S"},
    "I11": {"I9": "W", "R8": "A", "X6": "S"},
    "R1": {"I1": "A"},
    "R2": {"I3": "A"},
    "R3": {"I4": "A"},
    "R4": {"I7": "S"},
    "R5": {"I6": "A"},
    "R6": {"I8": "A"},
    "R7": {"I10": "W"},
    "R8": {"I11": "W"},
    "X2": {"I3": "W"},
    "X3": {"I6": "W"},
    "X4": {"I7": "W"},
    "X5": {"I10": "W"},
    "X6": {"I11": "W"}
}

curr_node = "X1"
Processing = False
Stage1Queue = []
Stage2Queue = []
Stage3Queue = []
Stage4Queue = ''


def on_ack_received(msg):
    global Stage1Queue
    rospy.loginfo(f"New ACK Handshake Initiated: command ID={msg.data.split('_')[1]}.")
    Stage1Queue.append(msg.data)

def on_position_received(msg):
    global curr_node, Processing, Stage2Queue, Stage3Queue
    # msg.data = X1_23 => command_command-ID
    command, id = msg.data.split('_')
    if id in Stage2Queue:
        rospy.loginfo(f"Command Received from Command and Control Server\nConnection Already Established: Command={command}, ID={id}")
        Stage2Queue.remove(id)
        Stage3Queue.append(translate[curr_node][command] + '_' + id)
        rospy.loginfo(f"Command Translated to Machine Command: Command={command} ID={id} Translation={translate[curr_node][command]}")

def move(command, id):
    """
    Send the master command to Robot Node 1
    set processing to True
    """
    global Processing
    Processing = True
    rospy.loginfo(f"Machine Command Sent to Robot 1: Command={command} ID={id}")
    pass

import random
# Robot1 Node
def robot1_node():
    global Stage1Queue, Stage2Queue, Stage3Queue, Stage4Queue, Processing 
    rospy.init_node('Robot1_Controller', anonymous=True)

    ack_response_pub = rospy.Publisher('/ack_response_r1', String, queue_size=10)
    position_pub = rospy.Publisher('/robot1_position', String, queue_size=10)
    rospy.Subscriber('/ack_to_r1', String, on_ack_received)
    rospy.Subscriber('/position_to_r1', String, on_position_received)

    rate = rospy.Rate(1)  # 1 Hz
    index = []
    while not rospy.is_shutdown():
        for ind, command in enumerate(Stage1Queue):
            ack_response_pub.publish(command)
            rospy.loginfo(f"ACK Accepted. Returning ACK: command ID={command.split('_')[1]}")
            index.append(ind)
        for ind in index[::-1]:
            Stage2Queue.append(Stage1Queue.pop(ind).split('_')[1])

        if not Processing and Stage3Queue:
            Stage4Queue = Stage3Queue.pop(0)
            command, id = Stage4Queue.split('_')
            move(command, id)
            Processing = False  # Get back the report from Robot, if no report -> then send failure
            status = random.choice(["Success", "Failure"])  # Implement the return report from Robot 1 here and set the status
            position_pub.publish(Stage4Queue + '_' + status)
        rate.sleep()

if __name__ == '__main__':
    try:
        robot1_node()
    except rospy.ROSInterruptException:
        pass
