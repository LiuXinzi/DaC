#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dynamixel_sdk import * # Uses Dynamixel SDK library
import time
import signal
import sys





def dynamixel_start(DXL_MAXIMUM_POSITION_VALUE = 100):
    """
    @param: DXL_MAXIMUM_POSITION_VALUE -- default 1000
    """


    #********* DYNAMIXEL Model Config *********

    MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430

    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0       # Refer to the Minimum Position Limit of product eManual
    # DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

    # Factory default ID of all DYNAMIXEL is 1
    DXL_ID                      = 1

    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = '/dev/ttyUSB0'

    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 5   # Dynamixel moving status threshold

    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

    
    return MY_DXL, packetHandler, portHandler, DXL_ID, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, dxl_goal_position, DXL_MOVING_STATUS_THRESHOLD, ADDR_TORQUE_ENABLE, TORQUE_DISABLE



def elephant_start():
    from pymycobot import ElephantRobot

    "连接机器人服务器"
    elephant_client = ElephantRobot("10.25.161.59", 5001)


    elephant_client.start_client()

    return elephant_client

    # while True:
    #     leader.get_digital_in(0)

def toggle_01(input_param):
    """
    imput_param is limitted to 0 or 1, int;
    this function toggle the input_param
    """
    if input_param == 0:
        return 1
    else:
        return 0


    


if __name__ == "__main__":

    index = 1

    MY_DXL, packetHandler, portHandler,  DXL_ID, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, dxl_goal_position, DXL_MOVING_STATUS_THRESHOLD, ADDR_TORQUE_ENABLE, TORQUE_DISABLE = dynamixel_start(DXL_MAXIMUM_POSITION_VALUE=550)
    
    ele_client = elephant_start()

    pin_value_last = ele_client.get_digital_in(0)
    

    def sig(signal, frame):
        """
        Catch "ctrl + c", and execute the following code to ensure the hardware would be closed safely.
        """

         # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Close port 
        portHandler.closePort()

        print()
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  Successfully Terminated  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        sys.exit(0)

    signal.signal(signal.SIGINT, sig)


    ##################################  Init  ##############################################
    # Write goal position
    if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Action Loop
    while 1:
        # Read present position
        if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        else:
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

        if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            # Change goal position (index)
            index = toggle_01(index)
            break


    ##########################################  Main Loop  ############################################

    # STUCK_THRESHOLD = 5

    while 1:
     
        time.sleep(0.1)
       
        
        # Check pin value
        pin_value = ele_client.get_digital_in(0)
        

        if not pin_value == pin_value_last:
            index = toggle_01(index)
            print("#############  Index Toggled  ############")

            # Write goal position
            if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
            else:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            # Action Loop
            # last_pos = 0
            
            while 1:
                # Read present position
                if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                else:
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))



                # Check if exit control loop
                
                # diff_pos = abs(dxl_present_position - last_pos)

                # if diff_pos < STUCK_THRESHOLD:
                #     pin_value_last = pin_value
                #     break

                if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    # save this pin_value
                    pin_value_last = pin_value
                    break

                # last_pos = dxl_present_position
                
                



        
        


   