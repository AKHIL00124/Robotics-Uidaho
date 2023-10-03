# Akhil Karri 
# program for Beaker

# Imports
import sys
import time
import random
from robot_controller import robot

drive_path = '172.29.209.124' #Beaker

def main():
    """! Main program entry"""

    # Create new robot object
    crx10 = robot(drive_path)

    # Set robot speed
    crx10.set_speed(400)

    loops = 1
    while(loops <= 2):
        
        # Set robot speed
        #crx10.set_speed(random.randrange(200, 300))
        print("==============================")
        print(f"Current loops: {loops}/2")
        print("==============================")

        # Home position (set all positions to 1)
        crx10.set_joints_to_home_position()
        crx10.start_robot()


        # Write new position in joint 2 to -30
        # crx10.write_joint_position(2, -30)
        # # Execute move action
        # crx10.start_robot()
        # time.sleep(1)

        #crx10.read_current_cartesian_position()

        # write new position in join 3-6 to 30
#        crx10.write_joint_position(3, -20)
#        crx10.write_joint_position(4, 30)
#        crx10.write_joint_position(5, -30)
#        crx10.write_joint_position(6, -30)
        # Execute move action
#        crx10.start_robot()
#        time.sleep(1)


        # P1
        crx10.send_coords(692.513, 260.448, 110.071, 178.383, 0, 27.133)
        crx10.start_robot()

        # P1
        # crx10.send_coords(692.513, 260.448, 110.071, 178.383, -0.710, 27.133)
        # crx10.start_robot()
        #crx10.read_current_cartesian_position()

        # open the Gripper
        crx10.gripper('open')

        # P2
        crx10.send_coords(692.513, 260.448, -202.505, 178.383, 0, 27.133)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        # #P3
        # crx10.send_coords(1517.779541, 802.317017, -644.753723, -108.776756, 88.492714, -83.370804)
        # crx10.start_robot()
        # #crx10.read_current_cartesian_position()

        # close the gripper
        crx10.gripper('close')

        #P4
        crx10.send_coords(700.193, 673.248, 208.375, 178.382, 0, 27.133)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        #P5
        crx10.send_coords(850.193, 670.273, -193.578, 178.055, 0, 28.233)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        # open the gripper
        crx10.gripper('open')

        #move the conveyor belt
        crx10.conveyor("forward")
        conveyor_toggle = True
        crx10.start_robot()

        #P6
        crx10.send_coords(116.356, 679.201, -101.802, 178.048, 0, 28.233)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        conveyor_on = True
        # cloops = 1
        while(conveyor_on == True):
            print("Conveyor moving")
            # if conveyor_toggle == True:
            #     conveyor_toggle = False
            # else:
            #     crx10.conveyor("forward")
            # print(f"Conveyor Loops: {cloops}/3")
            # conveyor_toggle = False

            # try:
                # conveyor_on = True
            while(conveyor_on):
                # Check Sensors
                right = crx10.conveyor_proximity_sensor("right")
                left = crx10.conveyor_proximity_sensor("left")

                # Sensor check
                if right and not left:
                    crx10.conveyor("stop")
                    conveyor_on = False
                    # time.sleep(0.5)
                elif not right and left:
                    crx10.conveyor("stop")
                    conveyor_on = False
                    # time.sleep(0.5)

                # Brief sleep to check sensors
                # time.sleep(0.1)
            # finally:
            #     print("Stopping conveyor belt...")
            #     crx10.conveyor("stop")
            #     cloops = 4

        print("conveyor stopping")

        #P7
        crx10.send_coords(141.856, 693.541, -200.000, 178.058, 0, 28.233)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        #P8
        # crx10.send_coords(889.670837, 1256.223999, -608.619202, -.269361, 88.227859, 28.562368)
        # crx10.start_robot()
        #crx10.read_current_cartesian_position()

        #close the gripper
        crx10.gripper('close')

        # P9
        crx10.send_coords(141.856, 693.541, 80.000, 178.058, 0, 28.233)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        # P10
        crx10.send_coords(658.92, 209.532, -90.746, -177.323, -2.76, 120.642)
        crx10.start_robot()
        #crx10.read_current_cartesian_position()

        crx10.send_coords(658.92, 209.532, -198.746, -177.323, -2.76, 120.642)
        crx10.start_robot()

        #open the gripper
        crx10.gripper('open')


        crx10.send_coords(658.92, 209.532, -90.746, -177.323, -2.76, 120.642)
        crx10.start_robot()


        # P11
        # crx10.send_coords(133.410995, 1038.439941, -655.750977, -105.189362, 87.483856, 12.12787)
        # crx10.start_robot()
        #crx10.read_current_cartesian_position()



        # Home position (set all positions to 1)
        crx10.set_joints_to_home_position()
        # Execute move action
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

if __name__=="__main__":
    main()
