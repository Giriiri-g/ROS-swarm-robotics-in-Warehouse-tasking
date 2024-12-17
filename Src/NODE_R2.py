#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import socket

# Movement dictionary for R2
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

curr_node = "X5"
Processing = False
Stage1Queue = []
Stage2Queue = []
# Stage3Queue = []
Stage3Queue = ['W_1', 'A_2', 'S_3', 'D_4']
Stage4Queue = ''
MachineCommand_Queue = ''

def send_to_esp32(ip, port, message):
    try:
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the ESP32 server
        client_socket.connect((ip, port))
        print(f"Connected to ESP32 at {ip}:{port}")

        # Send the message
        print(f"Sending: {message}")
        client_socket.sendall((message + "\n").encode())  # Add newline for ESP32

        # Receive response
        response = client_socket.recv(1024).decode()
        print(f"Received: {response}")

        # Close the connection
        client_socket.close()
    except Exception as e:
        print(f"Error: {e}")

def on_ack_received(msg):
    global Stage1Queue
    data = msg.data
    if data not in Stage1Queue:
        rospy.loginfo(f"New ACK Handshake Initiated to NODE_R2: command ID={data.split('_')[1]}.")
        Stage1Queue.append(msg.data)

def on_position_received(msg):
    global curr_node, Stage2Queue, Stage3Queue
    # msg.data = X5_23 => command_command-ID
    command, id = msg.data.split('_')
    if id in Stage2Queue:
        rospy.loginfo(f"Command Recieved from Command and Control Server\nConnection Already Established: Command={command}, ID={id}")
        Stage2Queue.remove(id)
        Stage3Queue.append(translate[curr_node][command]+'_'+id)
        rospy.loginfo(f"R2 Command Translated to Machine Command: Command={command} ID={id} Translation={translate[curr_node][command]}")

def move(command, id):
    """
    Send the master command to Robot Node 2
    set processing to True
    
    """
    global Processing, MachineCommand_Queue
    Processing = True
    send_to_esp32("192.168.137.155",80,command)
    rospy.loginfo(f"Machine Command Sent to Robot 2: Command={command} ID={id}")
    MachineCommand_Queue += command
    pass

import random
# Robot2 Node
def robot2_node():
    global Stage1Queue, Stage2Queue, Stage3Queue, Stage4Queue, Processing
    rospy.init_node('Robot2_Controller', anonymous=True)

    ack_response_pub = rospy.Publisher('/ack_response_r2', String, queue_size=10)
    position_pub = rospy.Publisher('/robot2_position', String, queue_size=10)
    rospy.Subscriber('/ack_to_r2', String, on_ack_received)
    rospy.Subscriber('/position_to_r2', String, on_position_received)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        index=[]
        for ind, command in enumerate(Stage1Queue):
            ack_response_pub.publish(command)
            rospy.loginfo(f"ACK Accepted. Returning ACK: command ID={command.split('_')[1]}")
            index.append(ind)
        for ind in index[::-1]:
            Stage2Queue.append(Stage1Queue.pop(ind).split('_')[1])


        if not Processing and Stage3Queue != []:
            rospy.loginfo(f"R2 Queue: {Stage3Queue}")
            Stage4Queue = Stage3Queue.pop(0)
            command, id = Stage4Queue.split('_')
            move(command, id)
            Processing = False # Get back the report from Robot, if no report -> then send failure
            status = random.choice(["Success", "Failure"]) # implement the Return report from Robot 2 here and set the status
            position_pub.publish(Stage4Queue+'_'+status)
        rate.sleep()

if __name__ == '__main__':
    try:
        robot2_node()
        print("NODE_R2 Commands:", MachineCommand_Queue)
    except rospy.ROSInterruptException:
        pass
