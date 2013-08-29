/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: ipa_canopen
 * \note
 *   ROS stack name: ipa_canopen
 * \note
 *   ROS package name: ipa_canopen_core
 *
 * \author
 *   Author: Thiago de Freitas
 * \author
 *   Supervised by: Thiago de Freitas, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: August 2013
 *
 * \brief
 *   Implementation of ipa_canopen driver.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include "schunkErrors.h"
#include "canopen.h"

namespace canopen
{
/***************************************************************/
//			define global variables and functions

std::map<std::string, HANDLE> h;
std::map<std::string, std::thread> listener_threads;
std::vector <std::string> open_devices;

bool openConnection(std::string devFile)
{
    h[devFile] = LINUX_CAN_Open(devFile.c_str(), O_RDWR);
    if (!h[devFile])
        return false;
    errno = CAN_Init(h[devFile], CAN_BAUD_500K, CAN_INIT_TYPE_ST);
    return true;
}

/***************************************************************/
//			define NMT variables
/***************************************************************/

TPCANMsg NMTmsg;

/***************************************************************/
//			define SYNC variables
/***************************************************************/

TPCANMsg syncMsg;


/***************************************************************/
//		define SDO protocol functions
/***************************************************************/

void requestDataBlock1(uint8_t CANid, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x60;
    msg.DATA[1] = 0x00;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h[devFile], &msg);
}

void requestDataBlock2(uint8_t CANid, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x70;
    msg.DATA[1] = 0x00;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h[devFile], &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x40;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h[devFile], &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x23;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h[devFile], &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x23;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h[devFile], &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x2F;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h[devFile], &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value, std::string devFile){
    TPCANMsg msg;
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x2B;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h[devFile], &msg);
}

void processSingleSDO(uint8_t CANid, TPCANRdMsg* message, std::string devFile)
{
    message->Msg.ID = 0x00;

    while (message->Msg.ID != (0x580+CANid))
    {
        LINUX_CAN_Read(h[devFile], message);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
}
