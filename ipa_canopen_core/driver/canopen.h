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
 *   Interfaces for the canopen library.
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

#ifndef CANOPEN_H
#define CANOPEN_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <cstdlib>
#include <thread>
#include <math.h>
#include <libpcan.h>
#include <utility>
#include <fcntl.h>    // for O_RDWR
#include <stdint.h>
#include <inttypes.h>
#include<algorithm>

namespace canopen
{

    /***************************************************************/
    //		    define classes and structs
    /***************************************************************/

    class Device
    {
    };

    class DeviceGroup{
    };

    struct SDOkey{
        uint16_t index;
        uint8_t subindex;

        inline SDOkey(TPCANRdMsg m):
            index((m.Msg.DATA[2] << 8) + m.Msg.DATA[1]),
            subindex(m.Msg.DATA[3]) {};

        inline SDOkey(uint16_t i, uint8_t s):
            index(i),
            subindex(s) {};
    };

    extern std::chrono::milliseconds syncInterval;
    extern std::map<std::string, HANDLE> h;
    extern std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers;
    extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;
    extern bool recover_active;

    bool openConnection(std::string devFile);

    inline bool operator<(const SDOkey &a, const SDOkey &b) {
            return a.index < b.index || (a.index == b.index && a.subindex < b.subindex);
    }

    /***********************************************************************/
    // SDO specific functions
    /***********************************************************************/
    void processSingleSDO(uint8_t CANid, TPCANRdMsg* message, std::string devFile);
    void requestDataBlock1(uint8_t CANid, std::string devFile);
    void requestDataBlock2(uint8_t CANid, std::string devFile);

    void sendSDO(uint8_t CANid, SDOkey sdo, std::string devFile);
    void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value,std::string devFile);
    void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value, std::string devFile);
    void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value, std::string devFile);
    void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value, std::string devFile);

    /***********************************************************************/
    // NMT specific functions
    /***********************************************************************/

    extern TPCANMsg NMTmsg;

    inline void sendNMT(uint8_t CANid, uint8_t command, std::string devFile)
    {
        //std::cout << "Sending NMT. CANid: " << (uint16_t)CANid << "\tcommand: " << (uint16_t)command << std::endl;
        NMTmsg.DATA[0] = command;
        NMTmsg.DATA[1] = CANid;
        CAN_Write(canopen::h[devFile], &NMTmsg);
    }

    /***************************************************************/
    //	define SYNC variables and functions
    /***************************************************************/

    extern TPCANMsg syncMsg;

    inline void sendSync(std::string devFile)
    {
        CAN_Write(h[devFile], &syncMsg);
    }

    /***************************************************************/
    //	Thread for device and port management
    /***************************************************************/
    extern std::map<std::string, std::thread> listener_threads;
    extern std::vector <std::string> open_devices;
}

#endif
