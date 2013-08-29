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


    cia_402 *cia_402_obj= new cia_402();

    std::map <uint8_t, cia_402::DeviceGroup::device_ptr> devs;

    cia_402::DeviceGroup::device_ptr device(new cia_402::Device(CANid) );

    devs[CANid] = device;

    //cia_402::deviceGroups_type deviceGroups;

    deviceGroups_402["name"].setDevices(devs);
    deviceGroups_402["name"].setDeviceFile(deviceFile);

    cia_402_obj->init("name", std::chrono::milliseconds(100));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
    canopen::sendSDO(CANid, MODES_OF_OPERATION, MODES_OF_OPERATION_HOMING_MODE, deviceFile);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    canopen::sendSDO(CANid, CONTROLWORD, (uint16_t) (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_START_HOMING), deviceFile);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	std::cout << "Homing complete" << std::endl;
}
