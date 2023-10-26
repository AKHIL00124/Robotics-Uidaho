# Akhil Karri 
# program for Robot1  Server

# Imports
import sys
import time
import random
import socket
import pickle
from robot_controller import robot

drive_path = '172.29.208.124' #Beaker

# Create a server socket   ********************************************
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
server_address = ('localhost', 8080)
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)
print('Server listening on', server_address)

# Accept a new connection
conn, addr = server_socket.accept()
print('New connection from', addr)

# Receive data from the client
data = conn.recv(1024).decode()
print('Received from client:', data)


def main():
    """! Main program entry"""

    int_1 = 1
    # int_0 = 0

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

        if loops == 1:
            # Home position (set all positions to 1)
            crx10.set_joints_to_home_position()
            crx10.start_robot()

            time.sleep(0.5)

            crx10.write_joint_offset(2, -10)
            crx10.start_robot()

            # print("After Offset")
            crx10.write_cartesian_position(900.513, -90.448, 690.071, 30.383, -80, 150.133)
            crx10.start_robot()


        print ("First Before P1")
        crx10.write_cartesian_position(850.513, 100.448, 450.071, 90.383, -60, 100.133)
        crx10.start_robot()


        print ("Second before P1")
        crx10.write_cartesian_position(740.513, 160.448, 250.071, 130.383, -40, 60.133)
        crx10.start_robot()

         # P1
        print("P1")
        crx10.write_cartesian_position(692.513, 260.448, 110.071, 178.383, 0, 27.133)
        crx10.start_robot()

        crx10.schunk_gripper('open')

        print("P2")
        crx10.write_cartesian_position(692.513, 260.448, -206.505, 178.383, 0, 27.133)
        crx10.start_robot()

        crx10.schunk_gripper('close')

        crx10.write_cartesian_position(693.594, 262.105, -120, 180, 0, 30)
        crx10.start_robot()

        # Randomize the Coords
        # [680.66, -789.615, 234, -90, 60, -180]   -= [-13, +-154, -56]
        con_pos = [693.594, 655.615, 290, -90, -30, 0]  # Coords above the Conveyor
        con_pos[0] += random.uniform(-50, 50)  
        con_pos[1] += random.uniform(-50, 50)  
        con_pos[2] += random.uniform(-50, 50) 
        # Pass the coords to take the pos
        crx10.write_cartesian_position(*con_pos)
        crx10.start_robot()

        print("Waiting for bit to be set")
        while crx10.read_robot_connection_bit() == 1:
            time.sleep(1)

        # Send the Pos Coords to Client Robot
        print("Bit is 1, Sending Data ")
        print(con_pos)
        data = pickle.dumps(con_pos)
        conn.send(data)
        print("Coord Data Sent")


        # Recieve confirmation from Client that it grabbed the cube
        while crx10.read_robot_connection_bit() == 1:
            time.sleep(1)

        data = conn.recv(1024)
        bit = int(data)

        if bit == 0:
            time.sleep(0.3)

        # Open the Gripper because client is holding it
        crx10.schunk_gripper('open')

        # Move Away from the Conveyor
        crx10.write_cartesian_position(693.594, 410.105, 290, -90, -30, 0)
        crx10.start_robot()


        # Waiting for Client Pos to go and grab the cube
        print("Waiting to recieve Coord Data")
        # Recive the pos from Server
        data = conn.recv(4096) 
        con_pos = pickle.loads(data)
        print(con_pos)
        print("Coord Data Recieved")

        con_pos[0] += 13 + 17 # Adjust X coordinate
        con_pos[1] = +con_pos[1] + 154 + 20  # Adjust Y coordinate
        con_pos[2] += 56  # Adjust Z coordinate
        con_pos[3] = -90
        con_pos[4] = -30
        con_pos[5] = 0
        crx10.write_cartesian_position(*con_pos)
        crx10.start_robot()

        crx10.schunk_gripper('close')

        # Send confirmation the cube is grabbed 
        bit = 1
        conn.send(str(bit).encode())
        crx10.write_robot_connection_bit(int_1)


        # Away from the Conveyor
        crx10.write_cartesian_position(693.594, 410.105, 290, -90, -30, 0)
        crx10.start_robot()

        # Above the Designated Pos
        crx10.write_cartesian_position(693.594, 262.105, -120, 180, 0, 30)
        crx10.start_robot()

        # Designated Pos
        crx10.write_cartesian_position(693.594, 262.105, -197.386, 180, 0, 30)
        crx10.start_robot()

        # # Above the Designated Pos
        # crx10.write_cartesian_position(693.594, 262.105, -120, 180, 0, 30)
        # crx10.start_robot()

        # # Away from the Conveyor
        # crx10.write_cartesian_position(693.594, 410.105, 290, -90, -30, 0)
        # crx10.start_robot()
        

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
    conn.close()

if __name__=="__main__":
    main()