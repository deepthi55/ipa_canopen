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


/***************************************************************/
//          IMPLEMENTATION:
//		    Namespace for the CAN in automation standard
//          CIA 402 - Drives and Motion Control
/***************************************************************/
std::map<std::string, cia_402::DeviceGroup> deviceGroups_402;

int32_t cia_402::rad2mdeg(double phi){
    return static_cast<int32_t>(round(phi/(2*M_PI)*360000.0));
}

double cia_402::mdeg2rad(int32_t alpha){
    return static_cast<double>(static_cast<double>(alpha)/360000.0*2*M_PI);
}

std::string cia_402::Device::getNMTState(){
    return Device::NMTState_;
}

std::string cia_402::Device::getdeviceStateMachine(){
    return motorState_;
}

std::vector<char> cia_402::Device::getManufacturerSWVersion(){
    return manufacturer_sw_version_;
}

std::vector<char> cia_402::Device::getManufacturerHWVersion(){
    return manufacturer_hw_version_;
}

std::vector<char> cia_402::Device::getManufacturerDevName(){
    return manufacturer_device_name_;
}

std::vector<uint16_t> cia_402::Device::getVendorID(){
    return vendor_id_;
}

std::vector<uint16_t> cia_402::Device::getProdCode(){
    return product_code_;
}

uint16_t cia_402::Device::getRevNumber(){
    return revision_number_;
}

uint8_t cia_402::Device::getCANid(){
    return CANid_;
}
std::string cia_402::Device::getDeviceFile(){
    return deviceFile_;
}
std::string cia_402::Device::getGroup(){
    return group_;
}
std::string cia_402::Device::getName(){
    return name_;
}
bool cia_402::Device::getInitialized(){
    return initialized_;
}


bool cia_402::Device::getVoltageEnabled(){
    return volt_enable_;
}

bool cia_402::Device::getReadySwitchOn(){
    return ready_switch_on_;
}

bool cia_402::Device::getSwitchOn(){
    return switched_on_;
}

bool cia_402::Device::getOpEnabled(){
    return op_enable_;
}

bool cia_402::Device::getQuickStop(){
    return quick_stop_;
}

bool cia_402::Device::getSwitchOnDisabled(){
    return switch_on_disabled_;
}

bool cia_402::Device::getWarning(){
    return warning_;
}

bool cia_402::Device::getModeSpecific(){
    return mode_specific_;
}

bool cia_402::Device::getRemote(){
    return remote_;
}
bool cia_402::Device::getTargetReached(){
    return target_reached_;
}

bool cia_402::Device::getInternalLimits(){
    return internal_limit_;
}


bool cia_402::Device::getOpSpec0(){
    return op_specific_;
}

bool cia_402::Device::getOpSpec1(){
    return op_specific1_;
}

bool cia_402::Device::getManSpec1(){
    return man_specific1_;
}

bool cia_402::Device::getmanSpec2(){
    return man_specific2_;
}

bool cia_402::Device::getHomingError(){
    return homingError_;
}

bool cia_402::Device::getFault(){
    return fault_;
}

bool cia_402::Device::getIPMode(){
    return ip_mode_active_;
}

bool cia_402::Device::getDriveReferenced(){
    return driveReferenced_;
}
double cia_402::Device::getActualPos(){
    return actualPos_;
}
double cia_402::Device::getDesiredPos(){
    return desiredPos_;
}

double cia_402::Device::getActualVel(){
    return actualVel_;
}

double cia_402::Device::getDesiredVel(){
    return desiredVel_;
}

inline std::chrono::milliseconds cia_402::Device::getTimeStamp_msec(){
    return timeStamp_msec_;
}

inline std::chrono::microseconds cia_402::Device::getTimeStamp_usec(){
    return timeStamp_usec_;
}

void cia_402::Device::setActualPos(double pos){
    actualPos_ = pos;
}

void cia_402::Device::setDesiredPos(double pos){
    desiredPos_ = pos;
}

void cia_402::Device::setActualVel(double vel){
    actualVel_ = vel;
}

void cia_402::Device::setDesiredVel(double vel){
    desiredVel_ = vel;
}

void cia_402::Device::deviceStateMachine(std::string nextState){
    motorState_ = nextState;
}

void cia_402::Device::setManufacturerSWVersion(std::vector<char> ms_version){
    manufacturer_sw_version_ = ms_version;
}

void cia_402::Device::setManufacturerHWVersion(std::vector<char> mh_version){
    manufacturer_hw_version_ = mh_version;
}

void cia_402::Device::setManufacturerDevName(std::vector<char> dev_name){
    manufacturer_device_name_ = dev_name;
}

void cia_402::Device::setVendorID(std::vector<uint16_t> v_id){
    vendor_id_ = v_id;
}

void cia_402::Device::setProdCode(std::vector<uint16_t> prod_code){
    product_code_ = prod_code;
}


void cia_402::Device::setRevNum(uint16_t rev_num){
    revision_number_ = rev_num;
}


void cia_402::Device::setNMTState(std::string nextState){
    NMTState_ = nextState;
}


void cia_402::Device::setVoltageEnabled(bool voltage_enabled){
    volt_enable_ = voltage_enabled;
}

void cia_402::Device::setReadySwitchON(bool r_switch_on){
    ready_switch_on_ = r_switch_on;
}

void cia_402::Device::setSwitchON(bool switch_on){
    switched_on_ = switch_on;
}

void cia_402::Device::setOpEnable(bool op_enable){
    op_enable_ = op_enable;
}

void cia_402::Device::setQuickStop(bool quick_stop){
    quick_stop_ = quick_stop;
}

void cia_402::Device::setSwitchOnDisable(bool switch_disabled){
    switch_on_disabled_ = switch_disabled;
}

void cia_402::Device::setWarning(bool warning){
    warning_ = warning;
}


void cia_402::Device::setModeSpec(bool modespec){
    mode_specific_ = modespec;
}


void cia_402::Device::setRemote(bool remote){
    remote_ = remote;
}

void cia_402::Device::setManSpec1(bool manspec1){
    man_specific1_ = manspec1;
}

void cia_402::Device::setTargetReached(bool target_reached){
    target_reached_ = target_reached;
}

void cia_402::Device::setInternalLimits(bool internal_limits){
    internal_limit_ = internal_limits;
}


void cia_402::Device::setManSpec2(bool manspec2){
    man_specific2_ = manspec2;
}

void cia_402::Device::setOpSpec1(bool opspec1){
    op_specific1_ = opspec1;
}

void cia_402::Device::setOpSpec0(bool opspec0){
    op_specific_ = opspec0;
}

void cia_402::Device::setFault(bool fault){
    fault_ = fault;
}

void cia_402::Device::setIPMode(bool ip_mode){
    ip_mode_active_ = ip_mode;
}

void cia_402::Device::setHoming(bool homing_error){
    homingError_ = homing_error;
}

void cia_402::Device::setInitialized(bool initialized){
    initialized_ = initialized;
}

void cia_402::Device::updateDesiredPos(std::chrono::milliseconds syncInterval)
{
    desiredPos_ += desiredVel_ * (syncInterval.count() / 1000.0);
}

void cia_402::Device::setTimeStamp_msec(std::chrono::milliseconds timeStamp){
    timeStamp_msec_ = timeStamp;
}

void cia_402::Device::setTimeStamp_usec(std::chrono::microseconds timeStamp){
    timeStamp_usec_ = timeStamp;
}

void cia_402::Device::setEMCYpressed(bool pressed){
    emcy_pressed_ = pressed;
}

void cia_402::Device::setEMCYreleased(bool released){
    emcy_released_ = released;
}

bool cia_402::Device::getEMCYpressed(){
    return emcy_pressed_;
}
bool cia_402::Device::getEMCYreleased(){
    return emcy_released_;
}


std::vector<uint8_t> cia_402::DeviceGroup::getCANids(){
    return CANids_;
}

std::vector<std::string>cia_402::DeviceGroup::getNames(){
    return names_;
}

uint32_t cia_402::DeviceGroup::getSyncInterval(){
    return syncInterval_;
}

std::string cia_402::DeviceGroup::getBaudRate(){
    return baudrate_;
}

std::string cia_402::DeviceGroup::getDeviceFile(){
    return deviceFile_;
}

bool cia_402::DeviceGroup::atFirstInit()
{
    return atFirstInit_;
}

bool cia_402::DeviceGroup::rActive()
{
    return recoverActive_;
}



std::string cia_402::DeviceGroup::getGroupName(){
    return group_name_;
}

std::map<uint8_t, cia_402::DeviceGroup::device_ptr> cia_402::DeviceGroup::getDevices()
{
    return devices_map_;
}

std::vector<double> cia_402::DeviceGroup::getActualPos()
{
    std::vector<double> actualPos;
    for (uint8_t CANid : CANids_)
        actualPos.push_back(DeviceGroup::getDevices()[CANid]->getActualPos());
    return actualPos;
}

std::vector<double> cia_402::DeviceGroup::getDesiredPos() {
    std::vector<double> desiredPos;
    for (auto CANid : CANids_)
        desiredPos.push_back(DeviceGroup::getDevices()[CANid]->getDesiredPos());
    return desiredPos;
}

std::vector<double> cia_402::DeviceGroup::getActualVel() {
    std::vector<double> actualVel;
    for (auto CANid : CANids_)
        actualVel.push_back(DeviceGroup::getDevices()[CANid]->getActualVel());
    return actualVel;
}

std::vector<double> cia_402::DeviceGroup::getDesiredVel() {
    std::vector<double> desiredVel;
    for (auto CANid : CANids_)
        desiredVel.push_back(DeviceGroup::getDevices()[CANid]->getDesiredVel());
    return desiredVel;
}

void cia_402::DeviceGroup::setSyncInterval(uint32_t syncInterval)
{
    DeviceGroup::syncInterval_ = syncInterval;
}

void cia_402::DeviceGroup::setBaudRate(std::string baudRate)
{
    DeviceGroup::baudrate_ = baudRate;
}

void cia_402::DeviceGroup::setFirstInit(bool atInit)
{
    DeviceGroup::atFirstInit_ = atInit;
}

void cia_402::DeviceGroup::setRecoverActive(bool rActive)
{
    DeviceGroup::recoverActive_ = rActive;
}

void cia_402::DeviceGroup::setDeviceFile(std::string deviceFile)
{
    DeviceGroup::deviceFile_ = deviceFile;
}

void cia_402::DeviceGroup::setGroupName(std::string groupName)
{
    DeviceGroup::group_name_ = groupName;
}


void cia_402::DeviceGroup::setVel(std::vector<double> velocities)
{
    for (unsigned int i=0; i<velocities.size(); i++) {
        DeviceGroup::getDevices()[CANids_[i]]->setDesiredVel(velocities[i]);
    }
}

void cia_402::DeviceGroup::setCANids(std::vector<uint8_t> CANids)
{
    CANids_ = CANids;
}

void cia_402::DeviceGroup::setNames(std::vector<std::string> names)
{
    names_= names;
}



void cia_402::DeviceGroup::setDevices(std::map<uint8_t, device_ptr> devices_map)
{
    DeviceGroup::devices_map_ = devices_map;
}

std::map<std::string, std::thread> manager_threads;

void statusword_incoming(uint8_t CANid, BYTE data[8], std::string chainName)
{

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
            deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_FAULT);
        }
        else if(switch_on_disabled)
        {
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_SWITCHED_ON_DISABLED);
        }
        else
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_NOT_READY_TO_SWITCH_ON);
    }

    else
    {
        if(switched_on)
        {
            if(op_enable)
            {

                //if(volt_enable)
                // {
               deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_OPERATION_ENABLED);
                // }

            }
            else
               deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_SWITCHED_ON);
        }
        else if(!quick_stop)
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_QUICK_STOP_ACTIVE);

        else
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_READY_TO_SWITCH_ON);

    }

    if(fault & op_enable & switched_on & ready_switch_on)
       deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_FAULT_REACTION_ACTIVE);



   deviceGroups_402[chainName].getDevices()[CANid]->setFault(fault);
   deviceGroups_402[chainName].getDevices()[CANid]->setHoming(op_specific);
   deviceGroups_402[chainName].getDevices()[CANid]->setOpSpec0(op_specific);
   deviceGroups_402[chainName].getDevices()[CANid]->setOpSpec1(op_specific1);
   deviceGroups_402[chainName].getDevices()[CANid]->setManSpec1(man_specific1);
   deviceGroups_402[chainName].getDevices()[CANid]->setManSpec2(man_specific2);
   deviceGroups_402[chainName].getDevices()[CANid]->setInternalLimits(internal_limit);
   deviceGroups_402[chainName].getDevices()[CANid]->setTargetReached(target_reached);
   deviceGroups_402[chainName].getDevices()[CANid]->setRemote(remote);
   deviceGroups_402[chainName].getDevices()[CANid]->setModeSpec(mode_specific);
   deviceGroups_402[chainName].getDevices()[CANid]->setWarning(warning);
   deviceGroups_402[chainName].getDevices()[CANid]->setSwitchOnDisable(switch_on_disabled);
   deviceGroups_402[chainName].getDevices()[CANid]->setQuickStop(quick_stop);
   deviceGroups_402[chainName].getDevices()[CANid]->setOpEnable(op_enable);
   deviceGroups_402[chainName].getDevices()[CANid]->setVoltageEnabled(volt_enable);
   deviceGroups_402[chainName].getDevices()[CANid]->setReadySwitchON(ready_switch_on);
   deviceGroups_402[chainName].getDevices()[CANid]->setSwitchON(switched_on);

    std::cout << "STATUS:Motor State of Device with CANid " << (uint16_t)CANid << " is: " <<deviceGroups_402[chainName].getDevices()[CANid]->getdeviceStateMachine() << std::endl;
}

void errorword_incoming(uint8_t CANid, BYTE data[1], std::string chainName)
{
    uint16_t mydata_low = data[0];

}


std::map<canopen::SDOkey, std::function<void (uint8_t CANid, BYTE data[8], std::string chainName)> > cia_402::incomingDataHandlers;

std::map<canopen::SDOkey, std::function<void (uint8_t CANid, BYTE data[8], std::string chainName)> > incomingErrorHandlers{ { ERRORWORD, errorword_incoming } };
std::map<uint16_t, std::function<void (const TPCANRdMsg m, std::string chainName)> > cia_402::incomingPDOHandlers;
std::map<uint16_t, std::function<void (const TPCANRdMsg m, std::string chainName)> > cia_402::incomingEMCYHandlers;


/******************************************************************************
 * Define get errors function
 *****************************************************************************/
void cia_402::getErrors(uint16_t CANid, std::string deviceFile)
{
    canopen::sendSDO(CANid, ERRORWORD, deviceFile);
}


void cia_402::readManErrReg(uint16_t CANid, TPCANRdMsg *m, std::string deviceFile)
{

    canopen::sendSDO(CANid, MANUFACTURER, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);

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

void cia_402::readErrorsRegister(uint16_t CANid, TPCANRdMsg *m, std::string deviceFile)
{
    canopen::sendSDO(CANid, STATUSWORD, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::processSingleSDO(CANid, m, deviceFile);

    canopen::sendSDO(CANid, ERRORWORD, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::processSingleSDO(CANid, m, deviceFile);

    uint16_t error_register;
    error_register = m->Msg.DATA[4];

    std::cout << "error_register=0x" << std::hex << (int)error_register << ", categories:";

    if ( error_register & EMC_k_1001_GENERIC )
        std::cout << " generic,";
    if ( error_register & EMC_k_1001_CURRENT)
        std::cout << " current,";
    if ( error_register & EMC_k_1001_VOLTAGE )
        std::cout << " voltage,";
    if ( error_register & EMC_k_1001_TEMPERATURE )
        std::cout << " temperature,";
    if ( error_register & EMC_k_1001_COMMUNICATION )
        std::cout << " communication,";
    if ( error_register & EMC_k_1001_DEV_PROF_SPEC )
        std::cout << " device profile specific,";
    if ( error_register & EMC_k_1001_RESERVED )
        std::cout << " reserved,";
    if ( error_register & EMC_k_1001_MANUFACTURER)
        std::cout << " manufacturer specific";
    std::cout << "\n";
}

std::vector<uint16_t> cia_402::obtainVendorID(uint16_t CANid, TPCANRdMsg *m, std::string deviceFile)
{
    canopen::sendSDO(CANid, IDENTITYVENDORID, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<uint16_t> vendor_id;

    canopen::processSingleSDO(CANid, m, deviceFile);

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

std::vector<uint16_t> cia_402::obtainProdCode(uint16_t CANid, TPCANRdMsg *m, std::string deviceFile)
{
    canopen::sendSDO(CANid, IDENTITYPRODUCTCODE, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<uint16_t> product_code;

    canopen::processSingleSDO(CANid, m, deviceFile);

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

uint16_t cia_402::obtainRevNr(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile)
{
    canopen::sendSDO(CANid, IDENTITYREVNUMBER, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    canopen::processSingleSDO(CANid, m, deviceFile);

    uint16_t rev_number = m->Msg.DATA[4];

    return rev_number;

}

std::vector<char> cia_402::obtainManDevName(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile)
{
    canopen::sendSDO(CANid, MANUFACTURERDEVICENAME, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<char> manufacturer_device_name;

    canopen::processSingleSDO(CANid, m, deviceFile);

    int size = m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_device_name.size() <= size)
            manufacturer_device_name.push_back(it);
    }


    canopen::requestDataBlock2(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_device_name.size() <= size)
            manufacturer_device_name.push_back(it);
    }

    return manufacturer_device_name;

}




std::vector<char> cia_402::obtainManHWVersion(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile)
{
    canopen::sendSDO(CANid, MANUFACTURERHWVERSION, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<char> manufacturer_hw_version;

    canopen::processSingleSDO(CANid, m, deviceFile);

    int size = m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_hw_version.size() <= size)
            manufacturer_hw_version.push_back(it);
    }


    canopen::requestDataBlock2(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_hw_version.size() <= size)
            manufacturer_hw_version.push_back(it);
    }

    return manufacturer_hw_version;
}

std::vector<char> cia_402::obtainManSWVersion(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile)
{
    std::vector<char> manufacturer_sw_version;

    canopen::sendSDO(CANid, MANUFACTURERSOFTWAREVERSION, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);

    int size = (uint8_t)m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }


    canopen::requestDataBlock2(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    canopen::requestDataBlock1(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    canopen::requestDataBlock2(CANid, deviceFile);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m, deviceFile);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    return manufacturer_sw_version;

}


void cia_402::process_errors(uint16_t CANid, TPCANRdMsg* m, std::string deviceFile, std::string chainName)
{
    cia_402::readErrorsRegister(CANid, m, deviceFile);

    /***************************************************************/
    //		Manufacturer specific errors register
    /***************************************************************/
    cia_402::readManErrReg(CANid, m, deviceFile);

    /**************************
         * Hardware and Software Information
        *************************/

    std::vector<uint16_t> vendor_id = cia_402::obtainVendorID(CANid, m, deviceFile);
    uint16_t rev_number = cia_402::obtainRevNr(CANid, m, deviceFile);
    std::vector<uint16_t> product_code = cia_402::obtainProdCode(CANid, m, deviceFile);
    std::vector<char> manufacturer_device_name = cia_402::obtainManDevName(CANid,m, deviceFile);
    std::vector<char> manufacturer_hw_version =  cia_402::obtainManHWVersion(CANid, m, deviceFile);
    std::vector<char> manufacturer_sw_version =  cia_402::obtainManSWVersion(CANid, m, deviceFile);


   deviceGroups_402[chainName].getDevices()[CANid]->setManufacturerHWVersion(manufacturer_hw_version);
   deviceGroups_402[chainName].getDevices()[CANid]->setManufacturerSWVersion(manufacturer_sw_version);
   deviceGroups_402[chainName].getDevices()[CANid]->setManufacturerDevName(manufacturer_device_name);
   deviceGroups_402[chainName].getDevices()[CANid]->setVendorID(vendor_id);
   deviceGroups_402[chainName].getDevices()[CANid]->setProdCode(product_code);
   deviceGroups_402[chainName].getDevices()[CANid]->setRevNum(rev_number);
}
/***************************************************************/
//		define init and recover sequence
/***************************************************************/
void cia_402::pre_init(std::string chainName)
{

    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

    for (auto device :deviceGroups_402[chainName].getDevices())
    {
        /*********************************************/
        std::cout << "Starting remote node" << (uint16_t)device.first << std::endl;
        canopen::sendNMT(device.first, NMT_START_REMOTE_NODE,deviceGroups_402[chainName].getDeviceFile());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        TPCANRdMsg m;

        cia_402::process_errors(device.first, &m,deviceGroups_402[chainName].getDeviceFile(), chainName);

    }
}


void cia_402::init(std::string chainName, std::chrono::milliseconds syncInterval)
{
    std::string deviceFile =deviceGroups_402[chainName].getDeviceFile();

    CAN_Close(canopen::h[deviceFile]);

    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

   deviceGroups_402[chainName].setRecoverActive(false);

    if (!canopen::openConnection(deviceGroups_402[chainName].getDeviceFile())){
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    if (deviceGroups_402[chainName].atFirstInit())
    {
        if (std::find(canopen::open_devices.begin(), canopen::open_devices.end(), deviceFile) == canopen::open_devices.end())
        {
            std::cout << "Creating the connection for the" << deviceFile << std::endl;
            canopen::listener_threads[deviceFile] = std::thread(cia_402::defaultListener, chainName);
            canopen::listener_threads[deviceFile].detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            canopen::open_devices.push_back(deviceFile);

        }
    }

    for (auto device :deviceGroups_402[chainName].getDevices())
    {
        uint8_t CANID = device.first;
        std::cout << "Module with CAN-id " << (uint16_t)CANID << " connected" << std::endl;
        cia_402::getErrors(CANID,deviceGroups_402[chainName].getDeviceFile());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (auto device :deviceGroups_402[chainName].getDevices())
    {
        uint8_t CANID = device.first;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Resetting CAN-device with CAN-ID " << (uint16_t)CANID << std::endl;
        canopen::sendNMT((uint16_t)CANID, NMT_RESET_NODE, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        canopen::sendNMT((uint16_t)CANID, NMT_START_REMOTE_NODE, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        /////////////////////////////////////////////////////////////////
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //              canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_FAULT_RESET_1, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(50));

        //              canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_SHUTDOWN, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(50));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));


        //              canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_SWITCH_ON, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(50));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //              canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_ENABLE_OPERATION, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(50));

        //              canopen::sendSDO(CANID, STATUSWORD, deviceFile);
        //              std::this_thread::sleep_for(std::chrono::milliseconds(100));


        cia_402::setMotorState(CANID, MS_SWITCHED_ON_DISABLED, device.second, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        cia_402::setMotorState(CANID, MS_READY_TO_SWITCH_ON, device.second, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        cia_402::setMotorState(CANID, MS_SWITCHED_ON, device.second, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        cia_402::setMotorState(CANID, MS_OPERATION_ENABLED, device.second, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //////////////////////////////////////////////////////////////////////

        canopen::sendSDO((uint16_t)CANID, IP_TIME_UNITS, (uint8_t) syncInterval.count() , deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        canopen::sendSDO((uint16_t)CANID, IP_TIME_INDEX, (uint8_t)IP_TIME_INDEX_MILLISECONDS, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        canopen::sendSDO((uint16_t)CANID, SYNC_TIMEOUT_FACTOR, (uint8_t)SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    for (auto device :deviceGroups_402[chainName].getDevices())
    {
        uint16_t CANID = device.first;
        cia_402::getErrors(CANID, deviceFile);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (deviceGroups_402[chainName].atFirstInit())
       deviceGroups_402[chainName].setFirstInit(false);
}


void cia_402::recover(std::string chainName, std::chrono::milliseconds syncInterval)
{
    std::string deviceFile = deviceGroups_402[chainName].getDeviceFile();
    CAN_Close(canopen::h[deviceFile]);

    std::cout << "Starting (recover)" << std::endl;

   deviceGroups_402[chainName].setRecoverActive(true);

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




    for (auto device :deviceGroups_402[chainName].getDevices())
    {
        uint8_t CANID = device.first;
        std::cout << "Module with CAN-id " << (uint16_t)CANID << " connected (recover)" << std::endl;
        // pre_init();
    }

    for (auto device :deviceGroups_402[chainName].getDevices())
    {
        uint8_t CANID = device.first;

        if(deviceGroups_402[chainName].getDevices()[CANID]->getdeviceStateMachine() == MS_OPERATION_ENABLED)
        {
            std::cout << "Node" << CANID << "is operational" << std::endl;
        }
        else
        {

            canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_HALT, deviceFile);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_DISABLE_INTERPOLATED, deviceFile);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            canopen::sendSDO(CANID, CONTROLWORD, CONTROL_WORD_DISABLE_VOLTAGE, deviceFile);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            canopen::sendSDO(CANID, CONTROLWORD, CONTROLWORD_QUICKSTOP, deviceFile);
            canopen::sendSDO(CANID, STATUSWORD, deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(CANID, MS_SWITCHED_ON_DISABLED, device.second, deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(CANID, MS_READY_TO_SWITCH_ON, device.second, deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(CANID, MS_SWITCHED_ON, device.second, deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            cia_402::setMotorState(CANID, MS_OPERATION_ENABLED, device.second, deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }


       deviceGroups_402[chainName].getDevices()[CANID]->setDesiredPos((double)deviceGroups_402[chainName].getDevices()[CANID]->getActualPos());
       deviceGroups_402[chainName].getDevices()[CANID]->setDesiredVel(0);

        cia_402::sendPos((uint16_t)CANID, (double)deviceGroups_402[chainName].getDevices()[CANID]->getDesiredPos(), chainName);
        cia_402::sendPos((uint16_t)CANID, (double)deviceGroups_402[chainName].getDevices()[CANID]->getDesiredPos(),chainName);

    }

    std::cout << "Concluded (recover)" << std::endl;
   deviceGroups_402[chainName].setRecoverActive(false);
}

/***************************************************************/
//		define state machine functions
/***************************************************************/

void cia_402::setNMTState(uint16_t CANid, std::string targetState){

}

void cia_402::setMotorState(uint16_t CANid, std::string targetState, DeviceGroup::device_ptr device, std::string deviceFile)
{

    while (device->getdeviceStateMachine() != targetState)
    {

        canopen::sendSDO(CANid, STATUSWORD,deviceFile);
        if (device->getdeviceStateMachine() == MS_FAULT)
        {
            canopen::sendSDO(CANid, STATUSWORD,deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if(!device->getFault())
            {
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceGroups_402[chainName].getDeviceFile());
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_0,deviceFile);

                //                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                //                canopen::sendSDO(CANid, STATUSWORD,deviceFile);
                //                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                //                std::cout << "2" << std::endl;
            }
            else
            {
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceGroups_402[chainName].getDeviceFile());
                //                canopen::sendSDO(deviceGroups_402[chainName].getDevices()[CANid]->getCANid(), CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceGroups_402[chainName].getDeviceFile());
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceFile);
                canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_FAULT_RESET_1,deviceFile);
                //                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                //                canopen::sendSDO(CANid, STATUSWORD,deviceFile);
                //                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                //                std::cout << "3" << std::endl;
            }

        }

        if (device->getdeviceStateMachine() == MS_NOT_READY_TO_SWITCH_ON)
        {
            canopen::sendSDO(CANid, STATUSWORD,deviceFile);

        }

        if (device->getdeviceStateMachine() == MS_SWITCHED_ON_DISABLED)
        {
            canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_SHUTDOWN,deviceFile);
        }

        if (device->getdeviceStateMachine() == MS_READY_TO_SWITCH_ON)
        {
            canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_SWITCH_ON,deviceFile);
        }

        if (device->getdeviceStateMachine() == MS_SWITCHED_ON)
        {
            canopen::sendSDO(CANid, CONTROLWORD, CONTROLWORD_ENABLE_OPERATION,deviceFile);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/***************************************************************/
//		define PDO protocol functions
/***************************************************************/

/*
       void initDeviceManagerThread(std::function<void ()> const& deviceManager) {
           std::thread device_manager_thread(deviceManager);
           device_manager_thread.detach();
           std::this_thread::sleep_for(std::chrono::milliseconds(10));
       }
       */

void cia_402::deviceManager(std::string chainName)
{
    // todo: init, recover... (e.g. when to start/stop sending SYNCs)
    std::string deviceFile = deviceGroups_402[chainName].getDeviceFile();

    uint32_t syncInterval = deviceGroups_402[chainName].getSyncInterval();
    while (true) {
        auto tic = std::chrono::high_resolution_clock::now();
        if (!deviceGroups_402[chainName].rActive()){
            for (auto device :deviceGroups_402[chainName].getDevices())
            {
                uint8_t CANID = device.first;
                if (device.second->getInitialized())
                {
                    device.second->updateDesiredPos(std::chrono::milliseconds(syncInterval));
                    cia_402::sendPos(CANID, (double)device.second->getDesiredPos(), chainName);
                }

            }
            canopen::sendSync(deviceFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(syncInterval) - (std::chrono::high_resolution_clock::now() - tic ));
        }

    }
}

std::function< void (uint16_t CANid, double positionValue, std::string chainName) > cia_402::sendPos;

void cia_402::defaultPDOOutgoing(uint16_t CANid, double positionValue, std::string chainName) {
    static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
    TPCANMsg msg;
    msg.ID = 0x200 + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = myControlword & 0xFF;
    msg.DATA[1] = (myControlword >> 8) & 0xFF;
    msg.DATA[2] = 0;
    msg.DATA[3] = 0;
    int32_t mdegPos = cia_402::rad2mdeg(positionValue);
    msg.DATA[4] = mdegPos & 0xFF;
    msg.DATA[5] = (mdegPos >> 8) & 0xFF;
    msg.DATA[6] = (mdegPos >> 16) & 0xFF;
    msg.DATA[7] = (mdegPos >> 24) & 0xFF;
    CAN_Write(canopen::h[deviceGroups_402[chainName].getDeviceFile()], &msg);
}

void cia_402::defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m, std::string chainName) {


    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];



    if (mydata_low == 0)
    {
       deviceGroups_402[chainName].getDevices()[CANid]->setEMCYreleased(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
       deviceGroups_402[chainName].getDevices()[CANid]->setEMCYpressed(false);
    }

    else
    {
       deviceGroups_402[chainName].getDevices()[CANid]->setEMCYpressed(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
       deviceGroups_402[chainName].getDevices()[CANid]->setEMCYreleased(false);
    }


    //std::cout << "EMCY" << (uint16_t)CANid << " is: " << (uint16_t)m.Msg.DATA[1] << std::endl;


}

void cia_402::defaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m, std::string chainName)
{
    double newPos = cia_402::mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24) );

    if (deviceGroups_402[chainName].getDevices()[CANid]->getTimeStamp_msec() != std::chrono::milliseconds(0) ||deviceGroups_402[chainName].getDevices()[CANid]->getTimeStamp_usec() != std::chrono::microseconds(0))
    {
        auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) -deviceGroups_402[chainName].getDevices()[CANid]->getTimeStamp_msec();
        auto deltaTime_usec = std::chrono::microseconds(m.wUsec) -deviceGroups_402[chainName].getDevices()[CANid]->getTimeStamp_usec();
        double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
        double result = (newPos -deviceGroups_402[chainName].getDevices()[CANid]->getActualPos()) / deltaTime_double;
       deviceGroups_402[chainName].getDevices()[CANid]->setActualVel(result);
        if (!deviceGroups_402[chainName].getDevices()[CANid]->getInitialized())
        {
           deviceGroups_402[chainName].getDevices()[CANid]->setDesiredPos(newPos);
           deviceGroups_402[chainName].getDevices()[CANid]->setInitialized(true);
        }
        // std::cout << "actualPos: " <<deviceGroups_402[chainName].getDevices()[CANid]->getActualPos() << "  desiredPos: " <<deviceGroups_402[chainName].getDevices()[CANid]->getDesiredPos() << std::endl;
    }


   deviceGroups_402[chainName].getDevices()[CANid]->setActualPos(newPos);
   deviceGroups_402[chainName].getDevices()[CANid]->setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
   deviceGroups_402[chainName].getDevices()[CANid]->setTimeStamp_usec(std::chrono::microseconds(m.wUsec));

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
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_FAULT);
        }
        else if(switch_on_disabled)
        {
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_SWITCHED_ON_DISABLED);
        }
        else
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_NOT_READY_TO_SWITCH_ON);
    }

    else
    {
        if(switched_on)
        {
            if(op_enable)
            {

                //if(volt_enable)
                // {
               deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_OPERATION_ENABLED);
                // }

            }
            else
               deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_SWITCHED_ON);
        }
        else if(!quick_stop)
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_QUICK_STOP_ACTIVE);

        else
           deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_READY_TO_SWITCH_ON);

    }

    if(fault & op_enable & switched_on & ready_switch_on)
       deviceGroups_402[chainName].getDevices()[CANid]->deviceStateMachine(MS_FAULT_REACTION_ACTIVE);


   deviceGroups_402[chainName].getDevices()[CANid]->setFault(fault);
   deviceGroups_402[chainName].getDevices()[CANid]->setIPMode(ip_mode);
   deviceGroups_402[chainName].getDevices()[CANid]->setHoming(op_specific);
   deviceGroups_402[chainName].getDevices()[CANid]->setOpSpec0(op_specific);
   deviceGroups_402[chainName].getDevices()[CANid]->setOpSpec1(op_specific1);
   deviceGroups_402[chainName].getDevices()[CANid]->setManSpec1(man_specific1);
   deviceGroups_402[chainName].getDevices()[CANid]->setManSpec2(man_specific2);
   deviceGroups_402[chainName].getDevices()[CANid]->setInternalLimits(internal_limit);
   deviceGroups_402[chainName].getDevices()[CANid]->setTargetReached(target_reached);
   deviceGroups_402[chainName].getDevices()[CANid]->setRemote(remote);
   deviceGroups_402[chainName].getDevices()[CANid]->setModeSpec(mode_specific);
   deviceGroups_402[chainName].getDevices()[CANid]->setWarning(warning);
   deviceGroups_402[chainName].getDevices()[CANid]->setSwitchOnDisable(switch_on_disabled);
   deviceGroups_402[chainName].getDevices()[CANid]->setQuickStop(quick_stop);
   deviceGroups_402[chainName].getDevices()[CANid]->setOpEnable(op_enable);
   deviceGroups_402[chainName].getDevices()[CANid]->setVoltageEnabled(volt_enable);
   deviceGroups_402[chainName].getDevices()[CANid]->setReadySwitchON(ready_switch_on);
   deviceGroups_402[chainName].getDevices()[CANid]->setSwitchON(switched_on);

    //       std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " <<deviceGroups_402[chainName].getDevices()[CANid]->getdeviceStateMachine() << std::endl;


}

/***************************************************************/
//		define functions for receiving data
/***************************************************************/

/*
    void initListenerThread(const std::function<void (std::string)> &listener, std::string devName){
        std::thread listener_thread(listener, devName);
        listener_thread.detach();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //std::cout << "Listener thread initialized" << std::endl;
    }
    */

void cia_402::defaultListener(std::string chainName)
{

    while(true)
    {
        std::string devName =deviceGroups_402[chainName].getDeviceFile();

        TPCANRdMsg m;
        errno = LINUX_CAN_Read(canopen::h[devName], &m);
        if (errno)
            perror("LINUX_CAN_Read() error");

        // incoming SYNC
        else if (m.Msg.ID == 0x080){
            //std::cout << std::hex << "SYNC received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        // incoming EMCY
        else if (m.Msg.ID >= 0x081 && m.Msg.ID <= 0x0FF){
            //std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
            if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
                incomingEMCYHandlers[m.Msg.ID](m, chainName);
        }

        // incoming TIME
        else if (m.Msg.ID == 0x100){
            //std::cout << std::hex << "TIME received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }



        // incoming PD0
        else if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x4FF){
            //std::cout << std::hex << "PDO received:  " << (m.Msg.ID - 0x180) << "  " << m.Msg.DATA[0] << " " << m.Msg.DATA[1] << " " << m.Msg.DATA[2] << " " << m.Msg.DATA[3] << " " << m.Msg.DATA[4] << " " << m.Msg.DATA[5] << " " << m.Msg.DATA[6] << " " <<  m.Msg.DATA[7] << " " << std::endl;
            //std::cout << std::hex << "PDO received:  " << (uint16_t)(m.Msg.ID - 0x180) << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " <<  (uint16_t)m.Msg.DATA[7] << " " << std::endl;
            if (cia_402::incomingPDOHandlers.find(m.Msg.ID) != cia_402::incomingPDOHandlers.end())
                cia_402::incomingPDOHandlers[m.Msg.ID](m, chainName);
        }

        // incoming SD0
        else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5FF){
                             std::cout << std::hex << "SDO received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
            canopen::SDOkey sdoKey(m);
            if (incomingErrorHandlers.find(sdoKey) != incomingErrorHandlers.end())
                incomingErrorHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA, chainName);
            if (cia_402::incomingDataHandlers.find(sdoKey) != cia_402::incomingDataHandlers.end())
                cia_402::incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA, chainName);
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
