#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
//Control table address for Dynamixel MX
#define ADDR_MX_TORQUE_ENABLE           24
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_CW_ANGLE_LIMIT          6               // set these two addr to 0 can triger
#define ADDR_MX_CCW_ANGLE_LIMIT         8               // the wheel mode?
#define ADDR_MX_MOVING_SPEED               32              // address of the moving speed control bit
#define ADDR_MX_READ_SPEED		38
// Protocol version 
#define PROTOCOL_VERSION                1.0

// Default setting
#define DXL_ID                          1               // Dynamixel ID
#define BAUDRATE                        1000000         
#define DEVICENAME                      "/dev/ttyUSB0"  // check which port is beeing used

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MINIMUM_POSITION_VALUE      100
#define DXL_MAXIMUM_POSITION_VALUE      4000
#define DXL_MOVING_STATUS_THRESHOLD     10              // MX moving status threahold?
#define CW_CCW_ANGLE_ZERO               0

#define ESC_ASCII_VALUE                 0x1b

/* The keyboard input method */
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

/* ROS topic listener feedback function 
 * set the global variable speed */
uint16_t speed = 0;
uint16_t cur_speed = 0;
void speedCallback(const std_msgs::Float64::ConstPtr& msg){
    double abs_msg;
    int tmp_speed;
    abs_msg = std::abs(msg->data);
    tmp_speed = int(abs_msg * 1023 / 100);
    if (msg->data > 0){
	if (abs_msg > 100) {
	    speed = 1023;
	} else{
            speed = tmp_speed; 
	}
    }else{
	if (abs_msg > 100) {
	    speed = 2047;
	} else {
            speed = 1024 + tmp_speed; 
	}
    }
    ROS_INFO("the command received is %d: ", speed);
}

int main(int argc, char ** argv){
    //initizlize ROS node
    ros::init(argc, argv, "dyna_cmd_node");
    ros::NodeHandle node;
    ros::Subscriber cmd_sub = node.subscribe("/dynamixel_speed", 10, speedCallback);
    ros::Rate loop_rate(5);
    // initilize PortHandler instance
    // set teh port path 
    // get method and members of PortHandlerLinx
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    //initialize PacketHandler instance
    //set the protocol version 
    //get hte mehod and members of protocolPacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION); 

    int dxl_comm_result = COMM_TX_FAIL;    // Communication result
    
    uint8_t dxl_error = 0;          // Dynamixel erro

    //Open port
    if (portHandler->openPort()){
        printf("Succeeded to open the port!\n");
    }else{
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        //getch();
        //return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)){
        printf("Succeeded to change the baudrate!\n");
    }else{
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        //getch();
        //return 0;
    }

    // Set the wheel mode
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_ANGLE_LIMIT, CW_CCW_ANGLE_ZERO, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS){
        packetHandler->printTxRxResult(dxl_comm_result); 
	ROS_INFO("dxl_comm_result!= COMM_SUCCESS");
    }else if (dxl_error != 0){
        packetHandler->printRxPacketError(dxl_error); 
    }else{
        printf("Succeeded in setting CW_ANGLE_LIMIT"); 
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_ANGLE_LIMIT, CW_CCW_ANGLE_ZERO, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS){
        packetHandler->printTxRxResult(dxl_comm_result); 
    }else if (dxl_error != 0){
        packetHandler->printRxPacketError(dxl_error); 
    }else{
        printf("Succeeded in setting CCW_ANGLE_LIMIT"); 
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

    while (ros::ok()){ 
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, speed, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            packetHandler->printTxRxResult(dxl_comm_result); 
        }else if(dxl_error != 0){
            packetHandler->printRxPacketError(dxl_error); 
        }

	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_READ_SPEED, &cur_speed, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS){
            packetHandler->printTxRxResult(dxl_comm_result); 
        }else if(dxl_error != 0){
            packetHandler->printRxPacketError(dxl_error); 
        }
	ROS_INFO("cur_speed:%d",cur_speed);
        ros::spinOnce();
        loop_rate.sleep();
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
    ROS_INFO("closed normally"); 
    return 0;

}
