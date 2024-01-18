//This code allows the user to change the ID of the motor

#include "InterfaceCAN.h"
#include "PinNames.h"
#include "Serial.h"
#include "mbed.h"
#include "math_ops.h"
#include "leg_message.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdio.h>


spi_data_t spi_data; // data from spine to up
spi_command_t spi_command; // data from up to spine



DigitalOut led(PA_5); 




Serial       pc(PA_2, PA_3);
CAN          can1(PB_12, PB_13, 1000000);  // CAN Rx pin name, CAN Tx pin name
//CAN          can2(PB_8, PB_9, 1000000);  // CAN Rx pin name, CAN Tx pin name

CANMessage   rxMsg1, rxMsg2;

Ticker                  sendCAN;
int                     counter = 0;
volatile bool           msgAvailable = false;
Ticker loop;


int function_code = 0;


void changeCAN(int ID, int new_ID){ 
    unsigned char changeCAN_ID_msg[6] = {0xff,0x06,0x15,0x01,0x02,0x00}; //0x15 is the register address of the equipment, for example the 0x02 change the ID into 0x02
    changeCAN_ID_msg[4] = new_ID;
    can1.write(CANMessage(ID, changeCAN_ID_msg, 6));  //we are changing the ID 0x01 into another ID
    function_code = 6;
    }
void saveCAN(int OLD_ID){ 
    unsigned char saveCommand[6] = {0xff,0x06,0x14,0x01,0x01,0x00}; //0x14 is the register address to save commands. To save we need 0x01 in message[4]
    can1.write(CANMessage(OLD_ID, saveCommand, 6));  //we are saving the new bit
    }  
void read_motor_parameter(int ID){
    unsigned char read_motor_parameters[8] = {0xff,0x02,0x00,0x00,0x00,0x00,0x00,0x00}; //function code 2
    can1.write(CANMessage(ID, read_motor_parameters, 8));  
    function_code = 2;
    }    
void incrementalPositionCommand(int ID, int angle){ //to move of n degrees from the actual position
//this function generates problem while reading the position each cycle
    read_motor_parameter(ID);
    if(can1.read(rxMsg1)){
        int offset = 0;
        
        int current_torque1 = rxMsg1.data[3];
        int speed1 = ((rxMsg1.data[5]<<8) | rxMsg1.data[4]); //from -256 to 256 rpm
        int position1 = ((rxMsg1.data[7]<<8) | rxMsg1.data[6])/100;
        angle = (angle + position1 + offset) * 100;
        uint8_t angle_low = (uint8_t)((angle << 8)>>8);
        uint8_t angle_high = (uint8_t)(angle >> 8);

        unsigned char message[8] = {0xff,0x01,0x00,0x24,0x00,0x20,angle_low,angle_high};   
        can1.write(CANMessage(ID, message, 8)); //mando questo messaggio can;, id + messaggio + lunghezza messaggio
        function_code = 1;
        printf("Message sent CAN 1: %x %x %x %x %x %x %x %x. CAN ID: %x \n", message[0], message[1], message[2], message[3], message[4], message[5], message[6], message[7], ID);
    } 
}
void PositionCommand(int ID, int angle){ //to move of n degrees from the zero position. Use this only after a first position reading
    /*
message[0] = 0xFF; //host address
message[1] = 0x01; //function code
message[2] = 0x00; //retain
message[3] = 0x24; //acceleration
message[4] = 0x00; //Retain
message[5] = 0x20; //0x80; //target speed
message[6] = 0x00; //target location low e high bit
message[7] = 0x00; //target location low e high bit
*/ 
        angle = angle * 100;
        uint8_t angle_low = (uint8_t)((angle << 8)>>8);
        uint8_t angle_high = (uint8_t)(angle >> 8);

        unsigned char message[8] = {0xff,0x01,0x00,0x24,0x00,0x20,angle_low,angle_high};   
        can1.write(CANMessage(ID, message, 8)); //mando questo messaggio can;, id + messaggio + lunghezza messaggio
        function_code = 1;
        //printf("Message sent CAN %x : %x %x %x %x %x %x %x %x \n", ID, message[0], message[1], message[2], message[3], message[4], message[5], message[6], message[7]);
} 
void initialCheck(){ //It checks the initial position of the motors (3 per leg)
        for(int k = 1; k<4; k++)
        {
        read_motor_parameter(k); //first of all read the actual position and the other mootor parameters
        wait_us(1000);
        printf("i = %d \n\r", k);
                    if ((can1.read(rxMsg1)) && (function_code == 2)) { //to read all the motor parameters
                    int slave_address1 = rxMsg1.data[0];
                    int current_torque1 = rxMsg1.data[3];
                    int temperature1 = rxMsg1.data[4];
                    int position1 = ((rxMsg1.data[7]<<8) | rxMsg1.data[6])/100;
                    printf("Message received rxMsg1: ID %x, Data length: %d, Slave address: %d, Function code: %x, Current Torque: %d, Temperature: %d, Position: %d \n\r", 
                    rxMsg1.id, rxMsg1.len, slave_address1, rxMsg1.data[1], current_torque1, 
                    temperature1, position1);
                    wait_us(10);
                }
        }
}


void LimitedWorkspaceCommand(int ID, int PositionValue, int inf_threshold[3], int sup_threshold[3]){
    if ((PositionValue>inf_threshold[ID-1]) && (PositionValue<sup_threshold[ID-1])) {
                PositionCommand(ID,PositionValue);
                printf("Sent position= %d ", PositionValue);
        }
    else if (PositionValue<inf_threshold[ID-1]) {
            PositionCommand(ID,inf_threshold[ID-1]);
            printf("Sent position= %d , inf threshold ", inf_threshold[ID-1]);
        }
    else if (PositionValue>sup_threshold[ID-1]) {
            PositionCommand(ID,sup_threshold[ID-1]);
            printf("Sent position= %d max threshold ", sup_threshold[ID-1]);
        } 
}
int main() {
        pc.baud(115200);

    /* RUN THIS SEQUENCE TO CHANGE MOTOR CAN ID */
      //  changeCAN(0x01, 0x02); //to change the motor ID from 0x01 to 0x02
       // wait_us(1000);
        //saveCAN(0x01); //to save the new motor ID
    /* SEQUENCE END */

        
// ----- MOTOR MOVEMENTS -----

        
        PositionCommand(0x02,50);//id & angle

        function_code = 2;
        wait_us(10);
        
    while (true) {

        PositionCommand(0x02,100);

        switch(function_code){
            case(1):
                if ((can1.read(rxMsg1)) && (function_code == 1)) { //to read the position motor response
                    int slave_address1 = rxMsg1.data[0];
                    int current_torque1 = rxMsg1.data[3];
                    int speed1 = ((rxMsg1.data[5]<<8) | rxMsg1.data[4]); //from -256 to 256 rpm
                    int position1 = ((rxMsg1.data[7]<<8) | rxMsg1.data[6])/100;
                    printf("Message received rxMsg1: ID %x, Data length: %d, Slave address: %d, Function code: %x, Current Torque: %d, speed: %d, Position: %d \n\r", 
                    rxMsg1.id, rxMsg1.len, slave_address1, rxMsg1.data[1], current_torque1, speed1, position1);
                    wait_us(10);
                }

            case(2):
                if ((can1.read(rxMsg1)) && (function_code == 2)) { //to read all the motor parameters
                    int slave_address1 = rxMsg1.data[0];
                    int current_torque1 = rxMsg1.data[3];
                    int temperature1 = rxMsg1.data[4];
                    int position1 = ((rxMsg1.data[7]<<8) | rxMsg1.data[6])/100;
                    printf("Message received rxMsg1: ID %x, Data length: %d, Slave address: %d, Function code: %x, Current Torque: %d, Temperature: %d, Position: %d \n\r", 
                    rxMsg1.id, rxMsg1.len, slave_address1, rxMsg1.data[1], current_torque1, 
                    temperature1, position1);
                    wait_us(10);
                }

            case(6):
                if ((can1.read(rxMsg1)) && (function_code == 6)) { //to read the change ID response
                    int slave_address1 = rxMsg1.data[0];
                    int register_address1 = rxMsg1.data[2];
                    int DATA1 = ((rxMsg1.data[4]<<8) | rxMsg1.data[5])/100;
                    printf("Message received rxMsg1: ID %x, Data length: %d, Slave address: %d, Function code: %x, Register address: %d, DATA: %d \n\r", 
                    rxMsg1.id, rxMsg1.len, slave_address1, rxMsg1.data[1], register_address1, DATA1);
                    wait_us(10);
                }
            }
        }

} 


