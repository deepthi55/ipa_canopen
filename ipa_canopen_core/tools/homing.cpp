// Copyright (c) 2012 Fraunhofer Institute
// for Manufacturing Engineering and Automation (IPA)
// See the file license.txt for copying permission.

// This program performs homing (referencing) of a device.
// See the user manual for details:
// https://github.com/ipa-tys/cia_402/blob/master/doc/usermanual.pdf?raw=true

#include <utility>
#include "cia_402.h"

int main(int argc, char *argv[]) {

	if (argc != 3) {
		std::cout << "Arguments:" << std::endl
		<< "(1) device file" << std::endl
		<< "(2) CAN deviceID" << std::endl
		<< "Example: ./homing /dev/pcan32 12" << std::endl;
		return -1;
	}
	std::string deviceFile = std::string(argv[1]);
    std::cout << deviceFile << std::endl;

	uint16_t CANid = std::stoi(std::string(argv[2]));

    // configure cia_402 device objects and custom incoming and outgoing PDOs:
    cia_402 *group1 = new cia_402();

    cia_402::device_group_ptr dgroup(new cia_402::DeviceGroup("name"));
    group1->updatedeviceGroups("name", dgroup);

    std::map <uint8_t, cia_402::DeviceGroup::device_ptr> devs;

    cia_402::DeviceGroup::device_ptr device(new cia_402::Device(CANid) );

    devs[CANid] = device;


    group1->getdeviceGroups()["name"]->setDevices(devs);
    group1->getdeviceGroups()["name"]->setDeviceFile(deviceFile);
    group1->getdeviceGroups()["name"]->setFirstInit(true);


    for (auto it :group1->getdeviceGroups())
    {
        std::cout << it.first << std::endl;
        std::cout << it.second->getDeviceFile() << std::endl;
        for (auto it2 : it.second->getDevices())
        {
            std::cout << (uint16_t)it2.first << std::endl;
        }
    }


    group1->init(group1->getdeviceGroups()["name"], std::chrono::milliseconds(100));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
    canopen::sendSDO(CANid, MODES_OF_OPERATION, MODES_OF_OPERATION_HOMING_MODE, deviceFile);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    canopen::sendSDO(CANid, CONTROLWORD, (uint16_t) (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_START_HOMING), deviceFile);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	std::cout << "Homing complete" << std::endl;
}
