# Akhil Karri 
# program for Robot2 Client

# Imports
import sys
import time
import random
import socket
import pickle
from robot_controller import robot

drive_path = '172.29.208.123' #Bunsen

# Create a client socket ****************************
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
server_address = ('localhost', 8080)
client_socket.connect(server_address)

# Send data to the server
message = 'Connected to server!'
client_socket.sendall(message.encode())

def main():
    """! Main program entry"""

    # Create new robot object
    crx10 = robot(drive_path)

    # Set robot speed
    crx10.set_speed(2000)

    loops = 1
    while(loops <= 2):

        crx10.set_joints_to_mount_position()
        crx10.start_robot()
        
        print("==============================")
        print(f"Current loops: {loops}/2")
        print("==============================")

        int_1 = 1
        int_0 = 0
        crx10.write_robot_connection_bit(int_1)

        if loops == 1:
            # # Home position (set all positions to 1)
            crx10.set_joints_to_home_position()
            crx10.start_robot()

            time.sleep(0.3)
   
            # crx10.write_cartesian_position(1050.66, -270.615, 650, -30, -80, -150)
            # crx10.start_robot()     

            crx10.write_cartesian_position(1000.66, -150.615, 400, -80, -60, -90)
            crx10.start_robot()

            crx10.write_cartesian_position(840.66, -350.615, 300, -80, 0, -140)
            crx10.start_robot()

            crx10.write_cartesian_position(550.66, -400.615, 300, -90, 0, 180)
            crx10.start_robot()

            # Ready Position near the conveyor
            crx10.write_cartesian_position(680.66, -350.615, 270, -90, 60, 180)
            crx10.start_robot()


        # Pos Away from Conveyor
        crx10.write_cartesian_position(655.66, -230.615, 210, -90, 60, 180)
        crx10.start_robot()
        
        crx10.schunk_gripper('open')

        # write bit is not working currently dont know why
        crx10.write_robot_connection_bit(int_1)
        print("Bit set to 1")
        
        # Recieve Pos Coords from the server
        print("Waiting to recieve Data")
        data = client_socket.recv(4096) 
        con_pos = pickle.loads(data)
        print(con_pos)
        print("Data Recieved")

        con_pos[0] -= 30  # Adjust X coordinate
        con_pos[1] = -con_pos[1] - 152 - 20  # Adjust Y coordinate
        con_pos[2] -= 56  # Adjust Z coordinate
        con_pos[3] = -90
        con_pos[4] = 60
        con_pos[5] = -180

        print("Adjusted Coordinates")

        # [693.594, 635.615, 290, -90, -30, 0]  -= [13, -154, 56]
        # go to the robot pos
        # crx10.write_cartesian_position(680.66, -789.615, 234, -90, 60, -180)
        crx10.write_cartesian_position(*con_pos)
        crx10.start_robot()

        print("At the Cube")

        crx10.schunk_gripper('close')

        # Telling the server the we grabbed the cube
        print("Setting bit to 0")
        bit = 0
        client_socket.send(str(bit).encode())
        crx10.write_robot_connection_bit(int_0)

        # Move away from robo 1
        crx10.write_cartesian_position(655.66, -230.615, 210, -90, 60, 180)
        crx10.start_robot()

        # Above the designated loc
        crx10.write_cartesian_position(715.54, 265.67, -30.7, 180, 0, 120)
        crx10.start_robot()

        # designated loc
        crx10.write_cartesian_position(712.5, 254.68, -184.7, -177.9, 0, 120)
        crx10.start_robot()

        time.sleep(1.5)

        # Above the designated loc
        crx10.write_cartesian_position(715.54, 265.67, -30.7, 180, 0, 120)
        crx10.start_robot()

        # Move away from robo 1
        crx10.write_cartesian_position(655.66, -230.615, 210, -90, 60, 180)
        crx10.start_robot()

        # Above the Conveyor and Randomize the Loc Coords
        con_pos = [680.66, -789.615, 234, -90, 60, -180]
        con_pos[0] += random.uniform(-50, 50)  # Randomize X coordinate
        con_pos[1] += random.uniform(-50, 50)  # Randomize Y coordinate
        con_pos[2] += random.uniform(-50, 50)  # Randomize Z coordinate
        crx10.write_cartesian_position(*con_pos)
        crx10.start_robot()

        # Send the Loc Coords 
        print(con_pos)
        data = pickle.dumps(con_pos)
        client_socket.send(data)
        print("Coord Data Sent")

        bit = 1
        client_socket.send(str(bit).encode())
        crx10.write_robot_connection_bit(int_1)

        # Waiting for the Server to grab the cube
        data = client_socket.recv(1024)
        # bit = int(data)

        crx10.schunk_gripper('open')

        # Away from Conveyor
        crx10.write_cartesian_position(655.66, -230.615, 210, -90, 60, 180)
        crx10.start_robot()

        if (loops > 1):
            # Mount position (set all positions to 1)
            crx10.set_joints_to_mount_position()
            crx10.start_robot()

        # Print Final position list
        print("*************************")
        print(" Final Joint Positions")
        print("*************************")
        crx10.read_current_joint_position()

        # increment loops
        loops += 1

    # End program
    print("==============================")
    print("END OF PROGRAM")
    print("==============================")
    client_socket.close()
    

if __name__=="__main__":
    main()