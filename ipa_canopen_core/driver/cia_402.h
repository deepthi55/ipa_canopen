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
 *   Author: Eduard Herkel, Thiago de Freitas, Tobias Sing
 * \author
 *   Supervised by: Eduard Herkel, Thiago de Freitas, Tobias Sing, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen driver.
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

#ifndef CIA_402_H
#define CIA_402_H

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
#include <canopen.h>
#include <cia_402_constants.h>


/***************************************************************/
//		    Class for the CAN in automation standard
//          CIA 402 - Drives and Motion Control
/***************************************************************/
/***************************************************************/
//		    define classes and structs
/***************************************************************/

class cia_402
{
public:

    class Device : public canopen::Device
    {

    private:

        uint8_t CANid_;
        std::string NMTState_;
        std::string motorState_;
        std::string deviceFile_;
        std::string name_;
        std::string group_;

        std::vector<char> manufacturer_sw_version_;
        std::vector<char> manufacturer_hw_version_;
        std::vector<char> manufacturer_device_name_;

        std::vector<uint16_t> vendor_id_;
        std::vector<uint16_t> product_code_;
        uint16_t revision_number_;

        bool initialized_;
        bool driveReferenced_;
        bool ip_mode_active_;
        bool homingError_;
        double actualPos_;		// unit = rad
        double desiredPos_;		// unit = rad
        double actualVel_;		// unit = rad/sec
        double desiredVel_;		// unit = rad/sec
        std::chrono::milliseconds timeStamp_msec_;
        std::chrono::microseconds timeStamp_usec_;



        bool ready_switch_on_;
        bool switched_on_;
        bool op_enable_;
        bool fault_;
        bool volt_enable_;
        bool quick_stop_;
        bool switch_on_disabled_;
        bool warning_;

        bool mode_specific_;
        bool remote_;
        bool target_reached_;
        bool internal_limit_;
        bool op_specific_;
        bool op_specific1_;
        bool man_specific1_;
        bool man_specific2_;

        bool emcy_pressed_;
        bool emcy_released_;


    public:

        Device() {};

        Device(uint8_t CANid):
            CANid_(CANid),
            desiredVel_(0),
            actualVel_(0),
            desiredPos_(0),
            actualPos_(0),
            initialized_(false),
            NMTState_("START_UP"),
            motorState_("START_UP"),
            emcy_released_(true) {};

        Device(uint8_t CANid, std::string name, std::string group, std::string bus):
            CANid_(CANid),
            name_(name),
            group_(group),
            deviceFile_(bus),
            desiredVel_(0),
            actualVel_(0),
            desiredPos_(0),
            actualPos_(0),
            initialized_(false),
            emcy_released_(true) {};

        std::string getNMTState();
        std::string getdeviceStateMachine();
        std::vector<char> getManufacturerSWVersion();
        std::vector<char> getManufacturerHWVersion();
        std::vector<char> getManufacturerDevName();
        std::vector<uint16_t> getVendorID();
        std::vector<uint16_t> getProdCode();
        uint16_t getRevNumber();
        uint8_t getCANid();
        std::string getDeviceFile();
        std::string getGroup();
        std::string getName();
        bool getInitialized();
        bool getVoltageEnabled();
        bool getReadySwitchOn();
        bool getSwitchOn();
        bool getOpEnabled();
        bool getQuickStop();
        bool getSwitchOnDisabled();
        bool getWarning();
        bool getModeSpecific();
        bool getRemote();
        bool getTargetReached();
        bool getInternalLimits();
        bool getOpSpec0();
        bool getOpSpec1();
        bool getManSpec1();
        bool getmanSpec2();
        bool getHomingError();
        bool getFault();
        bool getIPMode();
        bool getDriveReferenced();
        double getActualPos();
        double getDesiredPos();
        double getActualVel();
        double getDesiredVel();
        inline std::chrono::milliseconds getTimeStamp_msec();
        inline std::chrono::microseconds getTimeStamp_usec();
        void setActualPos(double pos);
        void setDesiredPos(double pos);
        void setActualVel(double vel);
        void setDesiredVel(double vel);
        void deviceStateMachine(std::string nextState);
        void setManufacturerSWVersion(std::vector<char> ms_version);
        void setManufacturerHWVersion(std::vector<char> mh_version);
        void setManufacturerDevName(std::vector<char> dev_name);
        void setVendorID(std::vector<uint16_t> v_id);
        void setProdCode(std::vector<uint16_t> prod_code);
        void setRevNum(uint16_t rev_num);
        void setNMTState(std::string nextState);
        void setVoltageEnabled(bool voltage_enabled);
        void setReadySwitchON(bool r_switch_on);
        void setSwitchON(bool switch_on);
        void setOpEnable(bool op_enable);
        void setQuickStop(bool quick_stop);
        void setSwitchOnDisable(bool switch_disabled);
        void setWarning(bool warning);
        void setModeSpec(bool modespec);
        void setRemote(bool remote);
        void setManSpec1(bool manspec1);
        void setTargetReached(bool target_reached);
        void setInternalLimits(bool internal_limits);
        void setManSpec2(bool manspec2);
        void setOpSpec1(bool opspec1);
        void setOpSpec0(bool opspec0);
        void setFault(bool fault);
        void setIPMode(bool ip_mode);
        void setHoming(bool homing_error);
        void setInitialized(bool initialized);
        void updateDesiredPos(std::chrono::milliseconds syncInterval);
        void setTimeStamp_msec(std::chrono::milliseconds timeStamp);
        void setTimeStamp_usec(std::chrono::microseconds timeStamp);
        void setEMCYpressed(bool pressed);
        void setEMCYreleased(bool released);
        bool getEMCYpressed();
        bool getEMCYreleased();

    };

    class DeviceGroup : virtual canopen::DeviceGroup
    {


    public:
        DeviceGroup() {};
        typedef std::shared_ptr<Device> device_ptr;

        DeviceGroup(std::string name):
            group_name_(name) {};

        DeviceGroup(std::vector<uint8_t> CANids):
            CANids_(CANids) {};

        DeviceGroup(std::vector<uint8_t> CANids, std::vector<std::string> names):
            CANids_(CANids),
            names_(names) {};

        std::vector<uint8_t> getCANids();

        std::vector<std::string> getNames();

        std::string getGroupName();

        std::vector<double> getActualPos();

        std::vector<double> getDesiredPos();

        std::vector<double> getActualVel();

        std::vector<double> getDesiredVel();

        bool atFirstInit();
        bool rActive();

        std::map<uint8_t, device_ptr> getDevices();

        uint32_t getSyncInterval();

        std::string getBaudRate();

        std::string getDeviceFile();

        void setSyncInterval(uint32_t syncInterval);

        void setFirstInit(bool atInit);

        void setRecoverActive(bool rActive);

        void setBaudRate(std::string baudRate);

        void setDeviceFile(std::string deviceFile);

        void setGroupName(std::string groupName);

        void setVel(std::vector<double> velocities);

        void setCANids(std::vector<uint8_t> CANids);

        void setNames(std::vector<std::string> names);

        void setDevices(std::map<uint8_t, device_ptr> devices_map);

    private:

        std::vector<uint8_t> CANids_;
        std::vector<std::string> names_;
        std::string group_name_;
        uint32_t syncInterval_;
        std::string baudrate_;
        std::string deviceFile_;  //For the moment assume that all group is connected to the same port
        std::map<uint8_t, DeviceGroup::device_ptr> devices_map_;
        bool atFirstInit_;
        bool recoverActive_;

    };

    typedef std::shared_ptr<Device> device_ptr;
    typedef std::shared_ptr<DeviceGroup> device_group_ptr;

    std::map<std::string, device_group_ptr> deviceGroups;

    void updatedeviceGroups(std::string name, device_group_ptr dgroup);
    std::map <std::string, device_group_ptr> getdeviceGroups();

    /***************************************************************/
    //		define global variables and functions
    /***************************************************************/
    static int32_t rad2mdeg(double phi);
    static double mdeg2rad(int32_t alpha);
    //    inline int32_t rad2mdeg(double phi){
    //        return static_cast<int32_t>(round(phi/(2*M_PI)*360000.0));
    //    }

    //    inline double mdeg2rad(int32_t alpha){
    //        return static_cast<double>(static_cast<double>(alpha)/360000.0*2*M_PI);
    //    }

    static void statusword_incoming(uint8_t CANid, BYTE data[8], cia_402::device_group_ptr);
    void errorword_incoming(uint8_t CANid, BYTE data[1], cia_402::device_group_ptr);


    static std::map<canopen::SDOkey, std::function<void (uint8_t CANid, BYTE data[8], cia_402::device_group_ptr)> > incomingDataHandlers;
    static std::map<uint16_t, std::function<void (const TPCANRdMsg m, cia_402::device_group_ptr)> > incomingPDOHandlers;
    static std::map<uint16_t, std::function<void (const TPCANRdMsg m, cia_402::device_group_ptr)> > incomingEMCYHandlers;

    cia_402() { };

    /***************************************************************/
    //			define state machine functions
    /***************************************************************/

    void setNMTState(uint16_t CANid, std::string targetState);
    void setMotorState(uint16_t CANid, std::string targetState, device_ptr device, std::string deviceFile);

    /***************************************************************/
    //	define get errors functions
    /***************************************************************/

    void getErrors(uint16_t CANid, std::string deviceFile);
    std::vector<char> obtainManSWVersion(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile);
    std::vector<char> obtainManHWVersion(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile);
    std::vector<char> obtainManDevName(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile);
    std::vector<uint16_t> obtainVendorID(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile);
    uint16_t obtainRevNr(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile);
    std::vector<uint16_t> obtainProdCode(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile);
    void readErrorsRegister(uint16_t CANid, TPCANRdMsg *m, std::string deviceFile);
    void readManErrReg(uint16_t CANid, TPCANRdMsg *m, std::string deviceFile);


    /***************************************************************/
    //	define init and recover variables and functions
    /***************************************************************/

    void init(cia_402::device_group_ptr, std::chrono::milliseconds syncInterval);
    void pre_init(cia_402::device_group_ptr);
    void process_errors(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile, cia_402::device_group_ptr devGroup);
    void recover(cia_402::device_group_ptr, std::chrono::milliseconds syncInterval);

    static std::function< void (uint16_t CANid, double positionValue,cia_402::device_group_ptr) > sendPos;
    std::function< void (uint16_t CANid) > geterrors;



    /***************************************************************/
    //		define PDO protocol functions
    /***************************************************************/

    //void initDeviceManagerThread(std::function<void ()> const& deviceManager);
    static void deviceManager(cia_402::device_group_ptr);

    static void defaultPDOOutgoing(uint16_t CANid, double positionValue, cia_402::device_group_ptr);
    static void defaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m, cia_402::device_group_ptr);
    void defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m, cia_402::device_group_ptr);

    /***************************************************************/
    //		define functions for receiving data
    /***************************************************************/
    std::map<std::string, std::thread> manager_threads;
    static void defaultListener(cia_402::device_group_ptr devGroup);
};

#endif // CIA_402_H
