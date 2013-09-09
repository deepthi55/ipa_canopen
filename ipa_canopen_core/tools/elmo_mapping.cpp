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
 *   Author: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 * \author
 *   Supervised by: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: July 2013
 *
 * \brief
 *   Get errors from the canopen device
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

#include <utility>
#include <iostream>
#include <iomanip>
#include "canopen.h"
#include <sstream>

int main(int argc, char *argv[])
{

    if (argc != 3) {
        std::cout << "Arguments:" << std::endl
        << "(1) device file" << std::endl
        << "(2) CAN deviceID" << std::endl
        << "Example: ./elmo_test /dev/pcan32 12" << std::endl;
        return -1;
    }



    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

    std::string deviceFile = std::string(argv[1]);

    if (!canopen::openConnection(deviceFile)){
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    uint16_t CANid = std::stoi(std::string(argv[2]));

    canopen::devices[ CANid ] = canopen::Device(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::sendNMT(CANid, canopen::NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    TPCANMsg m;

// //////////////////// disable TPDO 4
//    m.ID = CANid + 0x600;
//    m.MSGTYPE = 0x00;
//    m.LEN = 8;
//    m.DATA[0] = 0x22;
//    m.DATA[1] = 0x03;
//    m.DATA[2] = 0x18;
//    m.DATA[3] = 0x01;
//    m.DATA[4] = 0x81;
//    m.DATA[5] = 0x04;
//    m.DATA[6] = 0x00;
//    m.DATA[7] = 0x80;
//    CAN_Write(canopen::h, &m);

//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

//    std::cin.get();
//    /////////////////////////

//    //////////////////// clear mapping
//       m.ID = CANid + 0x600;
//       m.MSGTYPE = 0x00;
//       m.LEN = 8;
//       m.DATA[0] = 0x2F;
//       m.DATA[1] = 0x03;
//       m.DATA[2] = 0x1A;
//       m.DATA[3] = 0x00;
//       m.DATA[4] = 0x00;
//       m.DATA[5] = 0x00;
//       m.DATA[6] = 0x00;
//       m.DATA[7] = 0x00;
//       CAN_Write(canopen::h, &m);

//       std::this_thread::sleep_for(std::chrono::milliseconds(10));
//       std::cin.get();
//       /////////////////////////

//       //////////////////// sub ind1=63
//          m.ID = CANid + 0x600;
//          m.MSGTYPE = 0x00;
//          m.LEN = 8;
//          m.DATA[0] = 0x2F;
//          m.DATA[1] = 0x03;
//          m.DATA[2] = 0x1A;
//          m.DATA[3] = 0x01;
//          m.DATA[4] = 0x20;
//          m.DATA[5] = 0x00;
//          m.DATA[6] = 0x63;
//          m.DATA[7] = 0x60;
//          CAN_Write(canopen::h, &m);

//          std::this_thread::sleep_for(std::chrono::milliseconds(10));
//          std::cin.get();
//          /////////////////////////


//          //////////////////// sub ind2=69
//             m.ID = CANid + 0x600;
//             m.MSGTYPE = 0x00;
//             m.LEN = 8;
//             m.DATA[0] = 0x2F;
//             m.DATA[1] = 0x03;
//             m.DATA[2] = 0x1A;
//             m.DATA[3] = 0x02;
//             m.DATA[4] = 0x20;
//             m.DATA[5] = 0x00;
//             m.DATA[6] = 0x69;
//             m.DATA[7] = 0x60;
//             CAN_Write(canopen::h, &m);

//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             std::cin.get();
//             /////////////////////////


//             //////////////////// Sync
//                m.ID = CANid + 0x600;
//                m.MSGTYPE = 0x00;
//                m.LEN = 8;
//                m.DATA[0] = 0x2F;
//                m.DATA[1] = 0x03;
//                m.DATA[2] = 0x18;
//                m.DATA[3] = 0x02;
//                m.DATA[4] = 0x01;
//                m.DATA[5] = 0x00;
//                m.DATA[6] = 0x00;
//                m.DATA[7] = 0x00;
//                CAN_Write(canopen::h, &m);

//                std::this_thread::sleep_for(std::chrono::milliseconds(10));
//                std::cin.get();
//                //////////////////////
//                ///
//                ///
//                /////////////////////// Mapping 2 objects
//                      m.ID = CANid + 0x600;
//                      m.MSGTYPE = 0x00;
//                      m.LEN = 8;
//                      m.DATA[0] = 0x2F;
//                      m.DATA[1] = 0x03;
//                      m.DATA[2] = 0x1A;
//                      m.DATA[3] = 0x00;
//                      m.DATA[4] = 0x02;
//                      m.DATA[5] = 0x00;
//                      m.DATA[6] = 0x00;
//                      m.DATA[7] = 0x00;
//                      CAN_Write(canopen::h, &m);

//                      std::this_thread::sleep_for(std::chrono::milliseconds(10));
//                      std::cin.get();
//                      /////////////////////////

//                      //////////////////// Enable tpdo4
//                         m.ID = CANid + 0x600;
//                         m.MSGTYPE = 0x00;
//                         m.LEN = 8;
//                         m.DATA[0] = 0x22;
//                         m.DATA[1] = 0x03;
//                         m.DATA[2] = 0x18;
//                         m.DATA[3] = 0x01;
//                         m.DATA[4] = 0x81;
//                         m.DATA[5] = 0x04;
//                         m.DATA[6] = 0x00;
//                         m.DATA[7] = 0x00;
//                         CAN_Write(canopen::h, &m);

//                         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//                         std::cin.get();
//                         /////////////////////////





//    // NMT Reset
//    0x00 0x01 0x00

//    // RPDO4 auf Node ID 127 löschen
//    0x67F   0x2F  0x03  0x1A  0x00  0x00  0x00  0x00  0x00

//    // RPDO4 auf Node ID 127 Mapp Objekt 0x6063 (PX) an Position 1 , 32 bit
//    0x67F   0x2F  0x03  0x1A  0x01  0x20  0x00  0x63  0x60

//    // RPDO4 konfiguration auf Node ID 127 Event asyncron
//    0x67F   0x2F  0x03  0x18  0x02  0xFF  0x00  0x00  0x00

//    // RPDO4 konfiguration auf Node ID 127 Regler sendet aktiv alle 100 msec
//    0x67F   0x2F  0x03  0x18  0x05  0x64  0x00  0x00  0x00

//    // RPDO4 auf Node ID 127 aktivieren mit 1 Objekt
//    0x67F   0x2F  0x03  0x1A  0x00  0x01  0x00  0x00  0x00


                         /********************************************************************************************
                          *Mapping RPDO1 to StatusWord*/


                            //////////////////// clear mapping
                               m.ID = CANid + 0x600;
                               m.MSGTYPE = 0x00;
                               m.LEN = 8;
                               m.DATA[0] = 0x23;
                               m.DATA[1] = 0x03;
                               m.DATA[2] = 0x1A;
                               m.DATA[3] = 0x00;
                               m.DATA[4] = 0x00;
                               m.DATA[5] = 0x00;
                               m.DATA[6] = 0x00;
                               m.DATA[7] = 0x00;
                               CAN_Write(canopen::h, &m);

                               std::this_thread::sleep_for(std::chrono::milliseconds(10));
                               std::cin.get();
                               /////////////////////////

                               //////////////////// sub ind1=63
                                  m.ID = CANid + 0x600;
                                  m.MSGTYPE = 0x00;
                                  m.LEN = 8;
                                  m.DATA[0] = 0x23;
                                  m.DATA[1] = 0x03;
                                  m.DATA[2] = 0x1A;
                                  m.DATA[3] = 0x01;
                                  m.DATA[4] = 0x20;
                                  m.DATA[5] = 0x00;
                                  m.DATA[6] = 0x63;
                                  m.DATA[7] = 0x60;
                                  CAN_Write(canopen::h, &m);

                                  std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                  std::cin.get();
                                  /////////////////////////





                                     //////////////////// ASync
                                        m.ID = CANid + 0x600;
                                        m.MSGTYPE = 0x00;
                                        m.LEN = 8;
                                        m.DATA[0] = 0x23;
                                        m.DATA[1] = 0x03;
                                        m.DATA[2] = 0x18;
                                        m.DATA[3] = 0x02;
                                        m.DATA[4] = 0xff;
                                        m.DATA[5] = 0x00;
                                        m.DATA[6] = 0x00;
                                        m.DATA[7] = 0x00;
                                        CAN_Write(canopen::h, &m);

                                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                        std::cin.get();

                                        /////////////////////// Mapping 2 objects
                                              m.ID = CANid + 0x600;
                                              m.MSGTYPE = 0x00;
                                              m.LEN = 8;
                                              m.DATA[0] = 0x23;
                                              m.DATA[1] = 0x03;
                                              m.DATA[2] = 0x1A;
                                              m.DATA[3] = 0x00;
                                              m.DATA[4] = 0x01;
                                              m.DATA[5] = 0x00;
                                              m.DATA[6] = 0x00;
                                              m.DATA[7] = 0x00;
                                              CAN_Write(canopen::h, &m);

                                              std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                              std::cin.get();
                                              /////////////////////////


//                                                 // 0x20 = number of bits
//                                             sendSDODownload ( 0x1A00, 0, 0, true );	// stop all emissions of TPDO1
//                                             sendSDODownload ( 0x1A00, 1, 0x60640020, true );	// map actual position to 4 bytes of TPDO1

//                                             // #ifdef MEASURE_VELOCITY
//                                             // sendSDODownload ( 0x1A00, 2, 0x60690020, true ); // map actual velocity to 4 bytes of TPDO1
//                                             //#else
//                                             sendSDODownload ( 0x1A00, 2, 0x60780010, true );	// map actual current (in promille of rated current) to 2 bytes of TPDO1
//                                             //#endif

//                                             sendSDODownload ( 0x1800, 2, 1, true );	// transmission type "synch"
//                                             sendSDODownload ( 0x1A00, 0, 2, true );	// activate mapped objects


//                                             void CanDriveHarmonica::sendSDODownload ( int iObjIndex, int iObjSubIndex, int iData, bool bDelay )
//                                             {
//                                             CanMsg CMsgTr;

//                                             const int ciInitDownloadReq = 0x20;
//                                             const int ciNrBytesNoData = 0x00;
//                                             const int ciExpedited = 0x02;
//                                             const int ciDataSizeInd = 0x01;

//                                             CMsgTr.m_iLen = 8;
//                                             CMsgTr.m_iID = m_ParamCANopen.iRxSDO;

//                                             unsigned char cMsg[8];

//                                             cMsg[0] = ciInitDownloadReq | ( ciNrBytesNoData << 2 ) | ciExpedited | ciDataSizeInd;
//                                             cMsg[1] = iObjIndex;
//                                             cMsg[2] = iObjIndex >> 8;
//                                             cMsg[3] = iObjSubIndex;
//                                             cMsg[4] = iData;
//                                             cMsg[5] = iData >> 8;
//                                             cMsg[6] = iData >> 16;
//                                             cMsg[7] = iData >> 24;

//                                             CMsgTr.set ( cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7] );
//                                             m_pCanCtrl->transmitMsg ( CMsgTr );

//                                             if ( bDelay )
//                                             {
//                                             Sleep ( 20 );
//                                             }
//                                             }






}
