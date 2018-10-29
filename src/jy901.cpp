/**
 * @file jy901.cpp
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-16
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include "jy901_constants.h"
#include "jy901.h"

#include <cstring>
#include <cassert>
#include <iostream>

void CJY901::CopeSerialData(const unsigned char ucData[], unsigned int ucLength)
{
	static unsigned char ucRxBuffer[2048];
	static int ucRxCnt = 0;
    static bool initialised = false;

    // std::cout << "ADDING: " << ucLength << std::endl;

    // STEP 1: Add all new data in the buffer
	assert (ucRxCnt+ucLength<250);
    std::memcpy(&ucRxBuffer[ucRxCnt], ucData, ucLength);
	ucRxCnt = ucRxCnt + ucLength;

    // for (int i = 0 ; i < ucLength; i++ )
    //     printf("%x ", (unsigned int)(ucData[i]));
    // printf("\n");
    
    // STEP 2: Attempt to do initialisation, if it is not
    if (!initialised)
    {
        for(int i=0; i< ucRxCnt-11; i++){
            // Found the starting signature, which is a time message
            if (ucRxBuffer[i]==0x55 && ucRxBuffer[i+1]==0x50) 
            {
                // Remove all the junk before the starting signature 
                std::memmove(ucRxBuffer,&ucRxBuffer[i],ucRxCnt-i);
                initialised = true;
                break;
            }
        }

        // Abort if no starting signature detected
        if (!initialised){
            // std::cerr << "CJY901 Warning: No message yet" << std::endl;
            ucRxCnt = 0; // clear all, since there is no starting signature in the whole sequence
            return;
        }

        std::cout << "CJY901 Initialsed with First Incoming Message!" << std::endl;
    }
    
    ///// ALL CODE BELOW ONLY WORK ON INITIALISED BUFFER

    // DEBUG
	// if (ucRxBuffer[0]!=0x55) 
	// {
	// 	ucRxCnt=0;
    //     std::cerr << "CJY901 Warning: ucRxBuffer does not start with start byte 0x55." << std::endl;
	// 	exit(-1);
	// }

    // STEP 3: Parse the data, the first frame is aligned to the zero index
	while(ucRxCnt>=11)
	{
        // Check Sum
        unsigned char sum = 0;
        for (unsigned char i=0; i<10; i++)
            sum += ucRxBuffer[i];

        if (sum != ucRxBuffer[10]) // Check sum failed
        {
            std::cerr << "CJY901 Warning: Check Sum Failed." << std::endl;
            std::cerr << "sum= " << (unsigned int)sum << " ucRxBuffer[10]= " << (unsigned int)ucRxBuffer[10] << std::endl;
            initialised = false;
        }
        else // Check sum ok
        {
            static unsigned char last_msg_code = 0;
            switch(ucRxBuffer[1])
            {
                case 0x50:  std::memcpy(&(data_buffer.stcTime),&ucRxBuffer[2],8);break;
                case 0x51:	std::memcpy(&(data_buffer.stcAcc),&ucRxBuffer[2],8);break;
                case 0x52:	std::memcpy(&(data_buffer.stcGyro),&ucRxBuffer[2],8);break;
                case 0x53:	std::memcpy(&(data_buffer.stcAngle),&ucRxBuffer[2],8);break;
                case 0x54:	std::memcpy(&(data_buffer.stcMag),&ucRxBuffer[2],8);break;
                case 0x55:	std::memcpy(&(data_buffer.stcDStatus),&ucRxBuffer[2],8);break;
                case 0x56:	std::memcpy(&(data_buffer.stcPress),&ucRxBuffer[2],8);break;
                case 0x57:	std::memcpy(&(data_buffer.stcLonLat),&ucRxBuffer[2],8);break;
                case 0x58:	std::memcpy(&(data_buffer.stcGPSV),&ucRxBuffer[2],8);break;
                case 0x59:	std::memcpy(&(data_buffer.stcQuater),&ucRxBuffer[2],8);break;
                case 0x5a:	std::memcpy(&(data_buffer.stcSN),&ucRxBuffer[2],8);break;
            }
            // printf("Received: %x\n", ucRxBuffer[1]);

            last_msg_code = std::max(last_msg_code,ucRxBuffer[1]);

            if (ucRxBuffer[1] == last_msg_code){
                publishData();
            }
        }
        
		ucRxCnt -= 11; // this is valid due to the while loop condition check
        std::memmove(ucRxBuffer,&ucRxBuffer[11],ucRxCnt);
	}
}


void CJY901::publishData()
{
    data_buffer.seq = data.seq + 1;
    data = data_buffer;
}