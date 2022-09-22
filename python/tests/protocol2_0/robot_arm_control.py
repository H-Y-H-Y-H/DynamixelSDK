#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


#*******************************************************************************
#***********************     SyncRead and SyncWrite Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
import numpy as np
import random

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    LEN_GOAL_POSITION           = 4
    ADDR_PRESENT_POSITION       = 611
    LEN_PRESENT_POSITION        = 4
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = -150000    # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000     # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
# DXL1_ID                     = 8                # Dynamixel#1 ID : 1
# DXL2_ID                     = 1                # Dynamixel#1 ID : 2

joint_ids = [1,2,3,4,5,6,7]
# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB1'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 30               # Dynamixel moving status threshold

index = 0
# dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Enable Dynamixel Torque
for i in range(len(joint_ids)):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, joint_ids[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % joint_ids[i])


# Add parameter storage for Dynamixel present position value
for i in range(len(joint_ids)):
    dxl_addparam_result = groupSyncRead.addParam(joint_ids[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" %  joint_ids[i])
        quit()

dxl_goal_position = [[1000,1000,1000,1000],
                    [1500,1200,1000,1000],    # must be the same 
                    [1500,1200,1000,1000],    # must be inversed
                    [1500,1200,1000,1000],
                    [2047,2000,2100,2047],
                    [2047,1800,2300,2047],
                    [2047,2047,2047,2047],]


reset_cmds = [3100,1000,1000,1000,2047,2047,2047]


def random_bubbling():
    a0 = random.uniform(500,4000)
    a1 = random.uniform(1000,1500)
    a2 = a1
    a3 = random.uniform(1100,2000)
    a4 = random.uniform(1800,2300)
    a5 = random.uniform(1500,2300)
    a6 = 2047

    return [a0,a1,a2,a3,a4,a5,a6]

# def sliced_motor(cur_pos, target_pos):




def sin_move(t, sep = 100):
    a0 = 300 * np.sin(t * 2*np.pi/sep )+ 1000
    a1 = 300 * np.sin(t * 2*np.pi/sep )+ 1300
    a2 = a1
    a3 = 200 * np.sin(t * 2*np.pi/sep )+ 1200
    a4 = 200 * np.sin(t * 2*np.pi/sep )+ 2047
    a5 = 200 * np.sin(t * 2*np.pi/sep )+ 2047
    a6 = 2047

    return [a0,a1,a2,a3,a4,a5,a6]


# for index in range(len(dxl_goal_position[0])):
#     dxl_goal_position[2][index] = (2050 - dxl_goal_position[1][index]) + 2050




def act(action_list):

    # Allocate goal position value into byte array
    for i in range(len(joint_ids)):
        # if i == 2:
        #     continue
        motr_pos = int(action_list[i])
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(motr_pos)), DXL_HIBYTE(DXL_LOWORD(motr_pos)), DXL_LOBYTE(DXL_HIWORD(motr_pos)), DXL_HIBYTE(DXL_HIWORD(motr_pos))]

        # Add Dynamixel goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(joint_ids[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % joint_ids[i])
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()


def read_motor_pos():
    pos_list = []

    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for j in range(len(joint_ids)):

        # Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncRead.isAvailable(joint_ids[j], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % joint_ids[j])
            quit()

        # Get Dynamixel present position value
        dxl_present_position = groupSyncRead.getData(joint_ids[j], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        pos_list.append(dxl_present_position)

    return pos_list
        
# if getch() == chr(0x1b):
#     break

def step(cmds):
    act(cmds)
    time.sleep(0.03)

    finish_flag = [0]*len(joint_ids)
    # while 1:
    #     pos = read_motor_pos()
    #     for j in range(len(joint_ids)):
    #         if (abs(cmds[j] - pos[j]) < DXL_MOVING_STATUS_THRESHOLD):
    #             finish_flag[j] = 1
    #     print("finish_flag:", finish_flag)
    #     if (sum(finish_flag) == len(joint_ids)):
    #         break

    
if __name__ == "__main__":
    
    sep = 200
    epoch = 4
    
    num_steps = sep * epoch

    step(reset_cmds)
    time.sleep(2)
    for ep in range(epoch):

        cmds = random_bubbling()
        cmds = np.asarray(cmds)
        print(cmds)

        time.sleep(1)
        if( ep == (epoch-1)):
            cmds = reset_cmds
        for step_id in range(sep):
            # cmds = sin_move(t= i, sep= sep)
            cur_pos = np.asarray(read_motor_pos())
            mv_cmds = (cmds-cur_pos) * (0.5*np.cos(np.pi*2*step_id /(sep*1.5) - np.pi)+0.5) + cur_pos

            step(mv_cmds)
            print("READ: ",cur_pos)
            print("CMDS: ",mv_cmds)


# Clear syncread parameter storage
groupSyncRead.clearParam()

for i in range(len(joint_ids)):
    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, joint_ids[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
