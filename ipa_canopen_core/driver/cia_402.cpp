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

#include "cia_402.h"
#include "schunkErrors.h"
#include "canopen.h"

namespace cia_402
{

        std::string Device::getNMTState(){
            return Device::NMTState_;
        }

        std::string Device::getMotorState(){
            return motorState_;
        }

        std::vector<char> Device::getManufacturerSWVersion(){
            return manufacturer_sw_version_;
        }

        std::vector<char> Device::getManufacturerHWVersion(){
            return manufacturer_hw_version_;
        }

        std::vector<char> Device::getManufacturerDevName(){
            return manufacturer_device_name_;
        }

        std::vector<uint16_t> Device::getVendorID(){
            return vendor_id_;
        }

        std::vector<uint16_t> Device::getProdCode(){
            return product_code_;
        }

        uint16_t Device::getRevNumber(){
            return revision_number_;
        }

        uint8_t Device::getCANid(){
            return CANid_;
        }
        std::string Device::getDeviceFile(){
            return deviceFile_;
        }
        std::string Device::getGroup(){
            return group_;
        }
        std::string Device::getName(){
            return name_;
        }
        bool Device::getInitialized(){
            return initialized_;
        }


        bool Device::getVoltageEnabled(){
            return volt_enable_;
        }

        bool Device::getReadySwitchOn(){
            return ready_switch_on_;
        }

        bool Device::getSwitchOn(){
            return switched_on_;
        }

        bool Device::getOpEnabled(){
            return op_enable_;
        }

        bool Device::getQuickStop(){
            return quick_stop_;
        }

        bool Device::getSwitchOnDisabled(){
            return switch_on_disabled_;
        }

        bool Device::getWarning(){
            return warning_;
        }

        bool Device::getModeSpecific(){
            return mode_specific_;
        }

        bool Device::getRemote(){
            return remote_;
        }
        bool Device::getTargetReached(){
            return target_reached_;
        }

        bool Device::getInternalLimits(){
            return internal_limit_;
        }


        bool Device::getOpSpec0(){
            return op_specific_;
        }

        bool Device::getOpSpec1(){
            return op_specific1_;
        }

        bool Device::getManSpec1(){
            return man_specific1_;
        }

        bool Device::getmanSpec2(){
            return man_specific2_;
        }

        bool Device::getHomingError(){
            return homingError_;
        }

        bool Device::getFault(){
            return fault_;
        }

        bool Device::getIPMode(){
            return ip_mode_active_;
        }

        bool Device::getDriveReferenced(){
            return driveReferenced_;
        }
        double Device::getActualPos(){
            return actualPos_;
        }
        double Device::getDesiredPos(){
            return desiredPos_;
        }

        double Device::getActualVel(){
            return actualVel_;
        }

        double Device::getDesiredVel(){
            return desiredVel_;
        }

        inline std::chrono::milliseconds Device::getTimeStamp_msec(){
            return timeStamp_msec_;
        }

        inline std::chrono::microseconds Device::getTimeStamp_usec(){
            return timeStamp_usec_;
        }

        void Device::setActualPos(double pos){
            actualPos_ = pos;
        }

        void Device::setDesiredPos(double pos){
            desiredPos_ = pos;
        }

        void Device::setActualVel(double vel){
            actualVel_ = vel;
        }

        void Device::setDesiredVel(double vel){
            desiredVel_ = vel;
        }

        void Device::setMotorState(std::string nextState){
            motorState_ = nextState;
        }

        void Device::setManufacturerSWVersion(std::vector<char> ms_version){
            manufacturer_sw_version_ = ms_version;
        }

        void Device::setManufacturerHWVersion(std::vector<char> mh_version){
            manufacturer_hw_version_ = mh_version;
        }

        void Device::setManufacturerDevName(std::vector<char> dev_name){
            manufacturer_device_name_ = dev_name;
        }

        void Device::setVendorID(std::vector<uint16_t> v_id){
            vendor_id_ = v_id;
        }

        void Device::setProdCode(std::vector<uint16_t> prod_code){
            product_code_ = prod_code;
        }


        void Device::setRevNum(uint16_t rev_num){
            revision_number_ = rev_num;
        }


        void Device::setNMTState(std::string nextState){
            NMTState_ = nextState;
        }


        void Device::setVoltageEnabled(bool voltage_enabled){
            volt_enable_ = voltage_enabled;
        }

        void Device::setReadySwitchON(bool r_switch_on){
            ready_switch_on_ = r_switch_on;
        }

        void Device::setSwitchON(bool switch_on){
            switched_on_ = switch_on;
        }

        void Device::setOpEnable(bool op_enable){
            op_enable_ = op_enable;
        }

        void Device::setQuickStop(bool quick_stop){
            quick_stop_ = quick_stop;
        }

        void Device::setSwitchOnDisable(bool switch_disabled){
            switch_on_disabled_ = switch_disabled;
        }

        void Device::setWarning(bool warning){
            warning_ = warning;
        }


        void Device::setModeSpec(bool modespec){
            mode_specific_ = modespec;
        }


        void Device::setRemote(bool remote){
            remote_ = remote;
        }

        void Device::setManSpec1(bool manspec1){
            man_specific1_ = manspec1;
        }

        void Device::setTargetReached(bool target_reached){
            target_reached_ = target_reached;
        }

        void Device::setInternalLimits(bool internal_limits){
            internal_limit_ = internal_limits;
        }


        void Device::setManSpec2(bool manspec2){
            man_specific2_ = manspec2;
        }

        void Device::setOpSpec1(bool opspec1){
            op_specific1_ = opspec1;
        }

        void Device::setOpSpec0(bool opspec0){
            op_specific_ = opspec0;
        }

        void Device::setFault(bool fault){
            fault_ = fault;
        }

        void Device::setIPMode(bool ip_mode){
            ip_mode_active_ = ip_mode;
        }

        void Device::setHoming(bool homing_error){
            homingError_ = homing_error;
        }

        void Device::setInitialized(bool initialized){
            initialized_ = initialized;
        }

        void Device::updateDesiredPos()
        {
            desiredPos_ += desiredVel_ * (canopen::syncInterval.count() / 1000.0);
        }

        void Device::setTimeStamp_msec(std::chrono::milliseconds timeStamp){
            timeStamp_msec_ = timeStamp;
        }

        void Device::setTimeStamp_usec(std::chrono::microseconds timeStamp){
            timeStamp_usec_ = timeStamp;
        }

        void Device::setEMCYpressed(bool pressed){
            emcy_pressed_ = pressed;
        }

        void Device::setEMCYreleased(bool released){
            emcy_released_ = released;
        }

        bool Device::getEMCYpressed(){
            return emcy_pressed_;
        }
        bool Device::getEMCYreleased(){
            return emcy_released_;
        }


        std::vector<uint8_t> DeviceGroup::getCANids(){
            return CANids_;
        }

        std::vector<std::string> DeviceGroup::getNames(){
            return names_;
        }

        std::vector<double> DeviceGroup::getActualPos() {
                std::vector<double> actualPos;
                for (uint8_t CANid : CANids_)
                actualPos.push_back(devices[CANid].getActualPos());
                return actualPos;
            }

            std::vector<double> DeviceGroup::getDesiredPos() {
                std::vector<double> desiredPos;
                for (auto CANid : CANids_)
                    desiredPos.push_back(devices[CANid].getDesiredPos());
                return desiredPos;
            }

        std::vector<double> DeviceGroup::getActualVel() {
            std::vector<double> actualVel;
            for (auto CANid : CANids_)
                actualVel.push_back(devices[CANid].getActualVel());
            return actualVel;
        }

        std::vector<double> DeviceGroup::getDesiredVel() {
            std::vector<double> desiredVel;
            for (auto CANid : CANids_)
                desiredVel.push_back(devices[CANid].getDesiredVel());
            return desiredVel;
        }

        void DeviceGroup::setVel(std::vector<double> velocities) {
            for (unsigned int i=0; i<velocities.size(); i++) {
                devices[CANids_[i]].setDesiredVel(velocities[i]);
            }
            }

    std::map<uint8_t, Device> devices;
    std::map<std::string, DeviceGroup> deviceGroups;

    std::map<canopen::SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers{ { STATUSWORD, statusword_incoming } };
    std::map<canopen::SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingErrorHandlers{ { ERRORWORD, errorword_incoming } };
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;

    /***************************************************************/
    //		define init and recover sequence
    /***************************************************************/

    bool atFirstInit = true;

    void pre_init()
    {

        canopen::NMTmsg.ID = 0;
        canopen::NMTmsg.MSGTYPE = 0x00;
        canopen::NMTmsg.LEN = 2;

        canopen::syncMsg.ID = 0x80;
        canopen::syncMsg.MSGTYPE = 0x00;

        canopen::syncMsg.LEN = 0x00;

        for (auto dg : (cia_402::devices))
        {
        /*********************************************/
       canopen::sendNMT(dg.second.getCANid(), cia_402::NMT_START_REMOTE_NODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        TPCANRdMsg m;

        cia_402::process_errors(dg.second.getCANid(), &m);

        }
    }

    void process_errors(uint16_t CANid, TPCANRdMsg* m)
    {
        cia_402::readErrorsRegister(CANid, m);

        /***************************************************************/
        //		Manufacturer specific errors register
        /***************************************************************/
        cia_402::readManErrReg(CANid, m);

        /**************************
         * Hardware and Software Information
        *************************/

        std::vector<uint16_t> vendor_id = cia_402::obtainVendorID(CANid, m);
        uint16_t rev_number = cia_402::obtainRevNr(CANid, m);
        std::vector<uint16_t> product_code = cia_402::obtainProdCode(CANid, m);
        std::vector<char> manufacturer_device_name = cia_402::obtainManDevName(CANid,m);
        std::vector<char> manufacturer_hw_version =  cia_402::obtainManHWVersion(CANid, m);
        std::vector<char> manufacturer_sw_version =  cia_402::obtainManSWVersion(CANid, m);


        devices[CANid].setManufacturerHWVersion(manufacturer_hw_version);
        devices[CANid].setManufacturerSWVersion(manufacturer_sw_version);
        devices[CANid].setManufacturerDevName(manufacturer_device_name);
        devices[CANid].setVendorID(vendor_id);
        devices[CANid].setProdCode(product_code);
        devices[CANid].setRevNum(rev_number);
    }

    void init(std::string deviceFile, std::chrono::milliseconds syncInterval){
        CAN_Close(canopen::h);

        canopen::NMTmsg.ID = 0;
        canopen::NMTmsg.MSGTYPE = 0x00;
        canopen::NMTmsg.LEN = 2;

        canopen::syncMsg.ID = 0x80;
        canopen::syncMsg.MSGTYPE = 0x00;

        canopen::syncMsg.LEN = 0x00;

        canopen::recover_active = false;

        if (!canopen::openConnection(deviceFile)){
            std::cout << "Cannot open CAN device; aborting." << std::endl;
            exit(EXIT_FAILURE);
        }
        else{
            std::cout << "Connection to CAN bus established" << std::endl;
        }

        if (atFirstInit){
            cia_402::initListenerThread(cia_402::defaultListener);
        }

        for (auto device : devices){
            std::cout << "Module with CAN-id " << (uint16_t)device.second.getCANid() << " connected" << std::endl;
            getErrors(device.second.getCANid());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

           std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (auto device : devices)
        {

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "Resetting CAN-device with CAN-ID " << (uint16_t)device.second.getCANid() << std::endl;
           canopen::sendNMT((uint16_t)device.second.getCANid(), cia_402::NMT_RESET_NODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
           canopen::sendNMT((uint16_t)device.second.getCANid(), cia_402::NMT_START_REMOTE_NODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));


            cia_402::setMotorState(device.second.getCANid(), cia_402::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(device.second.getCANid(), cia_402::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(device.second.getCANid(), cia_402::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(device.second.getCANid(), cia_402::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            canopen::sendSDO((uint16_t)device.second.getCANid(), cia_402::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canopen::sendSDO((uint16_t)device.second.getCANid(), cia_402::IP_TIME_INDEX, (uint8_t)cia_402::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canopen::sendSDO((uint16_t)device.second.getCANid(), cia_402::SYNC_TIMEOUT_FACTOR, (uint8_t)cia_402::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }

        for (auto device : devices){
            getErrors(device.second.getCANid());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (atFirstInit)
            atFirstInit = false;
    }


    void recover(std::string deviceFile, std::chrono::milliseconds syncInterval){
        CAN_Close(canopen::h);


        canopen::recover_active = true;

        canopen::NMTmsg.ID = 0;
        canopen::NMTmsg.MSGTYPE = 0x00;
        canopen::NMTmsg.LEN = 2;

        canopen::syncMsg.ID = 0x80;
        canopen::syncMsg.MSGTYPE = 0x00;

        canopen::syncMsg.LEN = 0x00;

        if (!canopen::openConnection(deviceFile)){
            std::cout << "Cannot open CAN device; aborting." << std::endl;
            exit(EXIT_FAILURE);
        }
        else{
            std::cout << "Connection to CAN bus established (recover)" << std::endl;
        }




        for (auto device : devices){
            std::cout << "Module with CAN-id " << (uint16_t)device.second.getCANid() << " connected (recover)" << std::endl;
           // pre_init();
        }

        for (auto device : devices){

            if(device.second.getMotorState() == MS_OPERATION_ENABLED)
            {
                std::cout << "Node" << device.second.getCANid() << "is operational" << std::endl;
            }
            else
            {

                canopen::sendSDO(device.second.getCANid(), cia_402::CONTROLWORD, cia_402:: CONTROLWORD_HALT);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                canopen::sendSDO(device.second.getCANid(), cia_402::CONTROLWORD, cia_402:: CONTROLWORD_DISABLE_INTERPOLATED);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                canopen::sendSDO(device.second.getCANid(), cia_402::CONTROLWORD, cia_402:: CONTROL_WORD_DISABLE_VOLTAGE);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                canopen::sendSDO(device.second.getCANid(), cia_402::CONTROLWORD, cia_402::CONTROLWORD_QUICKSTOP);
                canopen::sendSDO(device.second.getCANid(), cia_402::STATUSWORD);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                cia_402::setMotorState(device.second.getCANid(), cia_402::MS_SWITCHED_ON_DISABLED);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                cia_402::setMotorState(device.second.getCANid(), cia_402::MS_READY_TO_SWITCH_ON);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                cia_402::setMotorState(device.second.getCANid(), cia_402::MS_SWITCHED_ON);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                cia_402::setMotorState(device.second.getCANid(), cia_402::MS_OPERATION_ENABLED);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                canopen::sendSDO((uint16_t)device.second.getCANid(), cia_402::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                canopen::sendSDO((uint16_t)device.second.getCANid(), cia_402::IP_TIME_INDEX, (uint8_t)cia_402::IP_TIME_INDEX_MILLISECONDS);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                canopen::sendSDO((uint16_t)device.second.getCANid(), cia_402::SYNC_TIMEOUT_FACTOR, (uint8_t)cia_402::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));


            }


            devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
            devices[device.second.getCANid()].setDesiredVel(0);

            cia_402::sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
            cia_402::sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());

        }
        canopen::recover_active = false;
    }

    /***************************************************************/
    //		define state machine functions
    /***************************************************************/

    void setNMTState(uint16_t CANid, std::string targetState){

    }

    void setMotorState(uint16_t CANid, std::string targetState){
        while (devices[CANid].getMotorState() != targetState){
                    canopen::sendSDO(CANid, cia_402::STATUSWORD);
        if (devices[CANid].getMotorState() == MS_FAULT)
    {
        canopen::sendSDO(CANid, cia_402::STATUSWORD);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if(!devices[CANid].getFault())
        {
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_0);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_0);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_0);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_0);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_0);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_0);
        }
        else
        {
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_1);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_1);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_1);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_1);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_1);
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402:: CONTROLWORD_FAULT_RESET_1);
        }

        }

        if (devices[CANid].getMotorState() == MS_NOT_READY_TO_SWITCH_ON){
            canopen::sendSDO(CANid, cia_402::STATUSWORD);
        }

        if (devices[CANid].getMotorState() == MS_SWITCHED_ON_DISABLED){
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402::CONTROLWORD_SHUTDOWN);
        }
        if (devices[CANid].getMotorState() == MS_READY_TO_SWITCH_ON){
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402::CONTROLWORD_SWITCH_ON);
        }
        if (devices[CANid].getMotorState() == MS_SWITCHED_ON){
            canopen::sendSDO(CANid, cia_402::CONTROLWORD, cia_402::CONTROLWORD_ENABLE_OPERATION);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        }

    /***************************************************************/
    //		define PDO protocol functions
    /***************************************************************/

    void initDeviceManagerThread(std::function<void ()> const& deviceManager) {
        std::thread device_manager_thread(deviceManager);
        device_manager_thread.detach();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void deviceManager() {
        // todo: init, recover... (e.g. when to start/stop sending SYNCs)
        while (true) {
            auto tic = std::chrono::high_resolution_clock::now();
            if (!canopen::recover_active){
                for (auto device : devices) {
                    if (device.second.getInitialized()) {
                        devices[device.first].updateDesiredPos();
                        sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
                    }
                }
                canopen::sendSync();
                std::this_thread::sleep_for(canopen::syncInterval - (std::chrono::high_resolution_clock::now() - tic ));
            }

        }
    }

    std::function< void (uint16_t CANid, double positionValue) > sendPos;

    void defaultPDOOutgoing(uint16_t CANid, double positionValue) {
        static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
        TPCANMsg msg;
        msg.ID = 0x200 + CANid;
        msg.MSGTYPE = 0x00;
        msg.LEN = 8;
        msg.DATA[0] = myControlword & 0xFF;
        msg.DATA[1] = (myControlword >> 8) & 0xFF;
        msg.DATA[2] = 0;
        msg.DATA[3] = 0;
        int32_t mdegPos = rad2mdeg(positionValue);
        msg.DATA[4] = mdegPos & 0xFF;
        msg.DATA[5] = (mdegPos >> 8) & 0xFF;
        msg.DATA[6] = (mdegPos >> 16) & 0xFF;
        msg.DATA[7] = (mdegPos >> 24) & 0xFF;
        CAN_Write(canopen::h, &msg);
    }

    void defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m) {


        uint16_t mydata_low = m.Msg.DATA[0];
        uint16_t mydata_high = m.Msg.DATA[1];

       

        if (mydata_low == 0)
        {
            devices[CANid].setEMCYreleased(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            devices[CANid].setEMCYpressed(false);
        }
        
        else
        {
            devices[CANid].setEMCYpressed(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            devices[CANid].setEMCYreleased(false);
        }


        //std::cout << "EMCY" << (uint16_t)CANid << " is: " << (uint16_t)m.Msg.DATA[1] << std::endl;


    }

    void defaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m) {
        double newPos = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24) );

        if (devices[CANid].getTimeStamp_msec() != std::chrono::milliseconds(0) || devices[CANid].getTimeStamp_usec() != std::chrono::microseconds(0)) {
            auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].getTimeStamp_msec();
            auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].getTimeStamp_usec();
            double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
            double result = (newPos - devices[CANid].getActualPos()) / deltaTime_double;
            devices[CANid].setActualVel(result);
                if (! devices[CANid].getInitialized()) {
                devices[CANid].setDesiredPos(newPos);
                devices[CANid].setInitialized(true);
            }
            //std::cout << "actualPos: " << devices[CANid].getActualPos() << "  desiredPos: " << devices[CANid].getDesiredPos() << std::endl;
        }


        devices[CANid].setActualPos(newPos);
        devices[CANid].setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
        devices[CANid].setTimeStamp_usec(std::chrono::microseconds(m.wUsec));

        uint16_t mydata_low = m.Msg.DATA[0];
        uint16_t mydata_high = m.Msg.DATA[1];

        bool ready_switch_on = mydata_low & 0x01;
        bool switched_on = mydata_low & 0x02;
        bool op_enable = mydata_low & 0x04;
        bool fault = mydata_low & 0x08;
       // std::cout << "fault PDO" << fault << std::endl;
        bool volt_enable = mydata_low & 0x10;
        bool quick_stop = mydata_low & 0x20;
        bool switch_on_disabled = mydata_low & 0x40;
        bool warning = mydata_low & 0x80;

        bool mode_specific = mydata_high & 0x01;
        bool remote = mydata_high & 0x02;
        bool target_reached = mydata_high & 0x04;
        bool internal_limit = mydata_high & 0x08;
        bool op_specific = mydata_high & 0x10;
        bool op_specific1 = mydata_high & 0x20;
        bool man_specific1 = mydata_high & 0x40;
        bool man_specific2 = mydata_high & 0x80;

        bool ip_mode = op_specific & volt_enable;


        if(!ready_switch_on)
        {
            if(fault)
                {
                 devices[CANid].setMotorState(cia_402::MS_FAULT);
                }
            else if(switch_on_disabled)
                {
                 devices[CANid].setMotorState(cia_402::MS_SWITCHED_ON_DISABLED);
                }
            else
                 devices[CANid].setMotorState(cia_402::MS_NOT_READY_TO_SWITCH_ON);
        }

        else
         {
                 if(switched_on)
                 {
                        if(op_enable)
                         {

                            //if(volt_enable)
                           // {
                                devices[CANid].setMotorState(cia_402::MS_OPERATION_ENABLED);
                           // }

                        }
                        else
                            devices[CANid].setMotorState(cia_402::MS_SWITCHED_ON);
                 }
                 else if(!quick_stop)
                        devices[CANid].setMotorState(cia_402::MS_QUICK_STOP_ACTIVE);

                 else
                    devices[CANid].setMotorState(cia_402::MS_READY_TO_SWITCH_ON);

                }

        if(fault & op_enable & switched_on & ready_switch_on)
            devices[CANid].setMotorState(cia_402::MS_FAULT_REACTION_ACTIVE);


        devices[CANid].setFault(fault);
        devices[CANid].setIPMode(ip_mode);
        devices[CANid].setHoming(op_specific);
        devices[CANid].setOpSpec0(op_specific);
        devices[CANid].setOpSpec1(op_specific1);
        devices[CANid].setManSpec1(man_specific1);
        devices[CANid].setManSpec2(man_specific2);
        devices[CANid].setInternalLimits(internal_limit);
        devices[CANid].setTargetReached(target_reached);
        devices[CANid].setRemote(remote);
        devices[CANid].setModeSpec(mode_specific);
        devices[CANid].setWarning(warning);
        devices[CANid].setSwitchOnDisable(switch_on_disabled);
        devices[CANid].setQuickStop(quick_stop);
        devices[CANid].setOpEnable(op_enable);
        devices[CANid].setVoltageEnabled(volt_enable);
        devices[CANid].setReadySwitchON(ready_switch_on);
        devices[CANid].setSwitchON(switched_on);

       // std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;


    }

    /***************************************************************/
    //		define functions for receiving data
    /***************************************************************/

    void initListenerThread(std::function<void ()> const& listener){
        std::thread listener_thread(listener);
        listener_thread.detach();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //std::cout << "Listener thread initialized" << std::endl;
    }

    void defaultListener(){
        while(true){
            //std::cout << "Reading incoming data" << std::endl;
            TPCANRdMsg m;
            errno = LINUX_CAN_Read(canopen::h, &m);
            if (errno)
                perror("LINUX_CAN_Read() error");

            // incoming SYNC
            else if (m.Msg.ID == 0x080){
               // std::cout << std::hex << "SYNC received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
            }

            // incoming EMCY
            else if (m.Msg.ID >= 0x081 && m.Msg.ID <= 0x0FF){
                //std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
                if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
                    incomingEMCYHandlers[m.Msg.ID](m);
            }

            // incoming TIME
            else if (m.Msg.ID == 0x100){
               // std::cout << std::hex << "TIME received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
            }

            // incoming PD0
            else if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x4FF){
               //std::cout << std::hex << "PDO received:  " << (m.Msg.ID - 0x180) << "  " << m.Msg.DATA[0] << " " << m.Msg.DATA[1] << " " << m.Msg.DATA[2] << " " << m.Msg.DATA[3] << " " << m.Msg.DATA[4] << " " << m.Msg.DATA[5] << " " << m.Msg.DATA[6] << " " <<  m.Msg.DATA[7] << " " << std::endl;
               //std::cout << std::hex << "PDO received:  " << (uint16_t)(m.Msg.ID - 0x180) << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " <<  (uint16_t)m.Msg.DATA[7] << " " << std::endl;
                if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                    incomingPDOHandlers[m.Msg.ID](m);
            }

            // incoming SD0
            else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5FF){
                std::cout << std::hex << "SDO received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
                canopen::SDOkey sdoKey(m);
                if (incomingErrorHandlers.find(sdoKey) != incomingErrorHandlers.end())
                    incomingErrorHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
                if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end())
                    incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
            }

            // incoming NMT error control
            else if (m.Msg.ID >= 0x700 && m.Msg.ID <= 0x7FF){
                if (m.Msg.DATA[0] == 0x00){
                    std::cout << "Bootup received. Node-ID =  " << (uint16_t)(m.Msg.ID - 0x700) << std::endl;
                }
                else{
                    std::cout << "NMT error control received:  " << (uint16_t)(m.Msg.ID - 0x700) << "  " << (uint16_t)m.Msg.DATA[0] << std::endl;
                }
            }
            else{
                 std::cout << "Received unknown message" << std::endl;
            }
        }
    }

/******************************************************************************
 * Define get errors function
 *****************************************************************************/
    void getErrors(uint16_t CANid)
    {
   canopen::sendSDO(CANid, cia_402::ERRORWORD);
    }

    void errorword_incoming(uint8_t CANid, BYTE data[1])
    {
        uint16_t mydata_low = data[0];

    }

    void readManErrReg(uint16_t CANid, TPCANRdMsg *m)
    {

        canopen::sendSDO(CANid, cia_402::MANUFACTURER);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);

        uint16_t code = m->Msg.DATA[4];
        uint16_t classification = m->Msg.DATA[5];

        std::cout << "manufacturer_status_register=0x" << std::hex << int(classification) << int(code) <<
                     ": code=0x" << std::hex << int( code ) << " (" << errorsCode[int(code)] << "),"
               << ", classification=0x" << std::hex << int( classification ) << std::dec;
        if ( classification == 0x88 )
            std::cout << " (CMD_ERROR)";
        if ( classification == 0x89 )
            std::cout << " (CMD_WARNING)";
        if ( classification == 0x8a )
            std::cout << " (CMD_INFO)";
        std::cout << "\n";


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void readErrorsRegister(uint16_t CANid, TPCANRdMsg *m)
    {
        canopen::sendSDO(CANid, cia_402::STATUSWORD);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        canopen::processSingleSDO(CANid, m);

        canopen::sendSDO(CANid, cia_402::ERRORWORD);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        canopen::processSingleSDO(CANid, m);

        uint16_t error_register;
        error_register = m->Msg.DATA[4];

        std::cout << "error_register=0x" << std::hex << (int)error_register << ", categories:";

        if ( error_register & cia_402::EMC_k_1001_GENERIC )
            std::cout << " generic,";
        if ( error_register & cia_402::EMC_k_1001_CURRENT)
            std::cout << " current,";
        if ( error_register & cia_402::EMC_k_1001_VOLTAGE )
            std::cout << " voltage,";
        if ( error_register & cia_402::EMC_k_1001_TEMPERATURE )
            std::cout << " temperature,";
        if ( error_register & cia_402::EMC_k_1001_COMMUNICATION )
            std::cout << " communication,";
        if ( error_register & cia_402::EMC_k_1001_DEV_PROF_SPEC )
            std::cout << " device profile specific,";
        if ( error_register & cia_402::EMC_k_1001_RESERVED )
            std::cout << " reserved,";
        if ( error_register & cia_402::EMC_k_1001_MANUFACTURER)
            std::cout << " manufacturer specific";
        std::cout << "\n";
    }

    std::vector<uint16_t> obtainVendorID(uint16_t CANid, TPCANRdMsg *m)
    {
        canopen::sendSDO(CANid, cia_402::IDENTITYVENDORID);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::vector<uint16_t> vendor_id;

        canopen::processSingleSDO(CANid, m);

        uint16_t id4 = m->Msg.DATA[4];
        uint16_t id3 = m->Msg.DATA[5];
        uint16_t id2 = m->Msg.DATA[6];
        uint16_t id1 = m->Msg.DATA[7];

        vendor_id.push_back(id1);
        vendor_id.push_back(id2);
        vendor_id.push_back(id3);
        vendor_id.push_back(id4);

        return vendor_id;
    }

    std::vector<uint16_t> obtainProdCode(uint16_t CANid, TPCANRdMsg *m)
    {
        canopen::sendSDO(CANid, cia_402::IDENTITYPRODUCTCODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::vector<uint16_t> product_code;

        canopen::processSingleSDO(CANid, m);

        uint16_t id4 = m->Msg.DATA[4];
        uint16_t id3 = m->Msg.DATA[5];
        uint16_t id2 = m->Msg.DATA[6];
        uint16_t id1 = m->Msg.DATA[7];

        product_code.push_back(id1);
        product_code.push_back(id2);
        product_code.push_back(id3);
        product_code.push_back(id4);

        return product_code;

    }

    uint16_t obtainRevNr(uint16_t CANid, TPCANRdMsg* m)
    {
        canopen::sendSDO(CANid, cia_402::IDENTITYREVNUMBER);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        canopen::processSingleSDO(CANid, m);

        uint16_t rev_number = m->Msg.DATA[4];

        return rev_number;

    }

    std::vector<char> obtainManDevName(uint16_t CANid, TPCANRdMsg* m)
    {
        canopen::sendSDO(CANid, cia_402::MANUFACTURERDEVICENAME);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::vector<char> manufacturer_device_name;

        canopen::processSingleSDO(CANid, m);

        int size = m->Msg.DATA[4];

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_device_name.size() <= size)
                manufacturer_device_name.push_back(it);
        }


        canopen::requestDataBlock2(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_device_name.size() <= size)
                manufacturer_device_name.push_back(it);
        }

        return manufacturer_device_name;

    }




     std::vector<char> obtainManHWVersion(uint16_t CANid, TPCANRdMsg* m)
     {
         canopen::sendSDO(CANid, cia_402::MANUFACTURERHWVERSION);
         std::this_thread::sleep_for(std::chrono::milliseconds(10));

         std::vector<char> manufacturer_hw_version;

         canopen::processSingleSDO(CANid, m);

         int size = m->Msg.DATA[4];

         canopen::requestDataBlock1(CANid);
         std::this_thread::sleep_for(std::chrono::milliseconds(10));

         canopen::processSingleSDO(CANid, m);


         for (auto it : m->Msg.DATA)
         {
             if(manufacturer_hw_version.size() <= size)
                 manufacturer_hw_version.push_back(it);
         }


         canopen::requestDataBlock2(CANid);
         std::this_thread::sleep_for(std::chrono::milliseconds(10));

         canopen::processSingleSDO(CANid, m);


         for (auto it : m->Msg.DATA)
         {
             if(manufacturer_hw_version.size() <= size)
                 manufacturer_hw_version.push_back(it);
         }

         return manufacturer_hw_version;
     }

    std::vector<char> obtainManSWVersion(uint16_t CANid, TPCANRdMsg* m)
    {
        std::vector<char> manufacturer_sw_version;

        canopen::sendSDO(CANid, cia_402::MANUFACTURERSOFTWAREVERSION);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);

        int size = (uint8_t)m->Msg.DATA[4];

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }


        canopen::requestDataBlock2(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }

        canopen::requestDataBlock2(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }

        return manufacturer_sw_version;

    }



void statusword_incoming(uint8_t CANid, BYTE data[8])
{

        //std::cout << (uint16_t)data[4] << std::endl;
        uint16_t mydata_low = data[4];
        uint16_t mydata_high = data[5];

        bool ready_switch_on = mydata_low & 0x01;
        bool switched_on = mydata_low & 0x02;
        bool op_enable = mydata_low & 0x04;
        bool fault = mydata_low & 0x08;
        bool volt_enable = mydata_low & 0x10;
        bool quick_stop = mydata_low & 0x20;
        bool switch_on_disabled = mydata_low & 0x40;
        bool warning = mydata_low & 0x80;

        bool mode_specific = mydata_high & 0x01;
        bool remote = mydata_high & 0x02;
        bool target_reached = mydata_high & 0x04;
        bool internal_limit = mydata_high & 0x08;
        bool op_specific = mydata_high & 0x10;
        bool op_specific1 = mydata_high & 0x20;
        bool man_specific1 = mydata_high & 0x40;
        bool man_specific2 = mydata_high & 0x80;


        bool ip_mode = op_specific & volt_enable;


        if(!ready_switch_on)
        {
            if(fault)
                {
                 devices[CANid].setMotorState(cia_402::MS_FAULT);
                }
            else if(switch_on_disabled)
                {
                 devices[CANid].setMotorState(cia_402::MS_SWITCHED_ON_DISABLED);
                }
            else
                 devices[CANid].setMotorState(cia_402::MS_NOT_READY_TO_SWITCH_ON);
        }

        else
         {
                 if(switched_on)
                 {
                        if(op_enable)
                         {

                            //if(volt_enable)
                           // {
                                devices[CANid].setMotorState(cia_402::MS_OPERATION_ENABLED);
                           // }

                        }
                        else
                            devices[CANid].setMotorState(cia_402::MS_SWITCHED_ON);
                 }
                 else if(!quick_stop)
                        devices[CANid].setMotorState(cia_402::MS_QUICK_STOP_ACTIVE);

                 else
                    devices[CANid].setMotorState(cia_402::MS_READY_TO_SWITCH_ON);

                }

        if(fault & op_enable & switched_on & ready_switch_on)
            devices[CANid].setMotorState(cia_402::MS_FAULT_REACTION_ACTIVE);



        devices[CANid].setFault(fault);
        devices[CANid].setHoming(op_specific);
        devices[CANid].setOpSpec0(op_specific);
        devices[CANid].setOpSpec1(op_specific1);
        devices[CANid].setManSpec1(man_specific1);
        devices[CANid].setManSpec2(man_specific2);
        devices[CANid].setInternalLimits(internal_limit);
        devices[CANid].setTargetReached(target_reached);
        devices[CANid].setRemote(remote);
        devices[CANid].setModeSpec(mode_specific);
        devices[CANid].setWarning(warning);
        devices[CANid].setSwitchOnDisable(switch_on_disabled);
        devices[CANid].setQuickStop(quick_stop);
        devices[CANid].setOpEnable(op_enable);
        devices[CANid].setVoltageEnabled(volt_enable);
        devices[CANid].setReadySwitchON(ready_switch_on);
        devices[CANid].setSwitchON(switched_on);

        std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
    }

}
