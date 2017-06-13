#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"

//Control table address for Dynamixel MX
#define ADDR_MX_TORQUE_ENABLE           24
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version 
#define PROTOCOL_VERSION                2.0

// Default setting
#define DXL_ID                          1               // Dynamixel ID
#define BAUDRATE                        1000000         
#define DEVICENAME                      "/dev/ttyUSB0"  // check which port is beeing used

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MINIMUM_POSITION_VALUE      100
#define DXL_MAXIMUM_POSITION_VALUE      4000
#define DXL_MOVING_STATUS_THRESHOLD     10              // MX moving status threahold?

#define ESC_ASCII_VALUE                 0x1b

int getch(){
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(){
    // initilize PortHandler instance
    // set teh port path 
    // get method and members of PortHandlerLinx
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    //initialize PacketHandler instance
    //set the protocol version 
    //get hte mehod and members of protocolPacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION); 

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;    // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};
    
    uint8_t dxl_error = 0;          // Dynamixel erro
    uint16_t dxl_present_position = 0;

    //Open port
    if (portHandler->openPort()){
        printf("Succeeded to open the port!\n");
    }else{
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)){
        printf("Succeeded to change the baudrate!\n");
    }else{
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable Dynamixel torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS){
        packetHandler->printTxRxResult(dxl_comm_result); 
    }else if (dxl_error != 0){
        packetHandler->printRxPacketError(dxl_error); 
    }else{
        printf("Dynamixel %d has been successfully connected \n", DXL_ID); 
    }

    while(1){
        printf("press any key to continue!(or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE){
            break; 
        }

        // Write Dynamixel goal position
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            packetHandler->printTxRxResult(dxl_comm_result); 
        }else if(dxl_error != 0){
            packetHandler->printRxPacketError(dxl_error); 
        }

        do{
            //Read Dynamixel present position
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            
            if (dxl_comm_result != COMM_SUCCESS){
                packetHandler->printTxRxResult(dxl_comm_result); 
            }else if(dxl_error != 0){
                packetHandler->printRxPacketError(dxl_error); 
            }
            printf("[ID:%03d] GoalPos:%03d PresPos:%03d",DXL_ID, dxl_goal_position[index],dxl_present_position);

        }while(abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD);

        //Change goal position
        if (index == 0){
            index =1;
        } else{
            index = 0; 
        }
    }

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS){
        packetHandler->printTxRxResult(dxl_comm_result); 
    }else if(dxl_error != 0){
        packetHandler->printRxPacketError(dxl_error); 
    }

    // Close port
    portHandler->closePort();
    
    return 0;

}
