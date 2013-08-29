// Copyright (c) 2012 Fraunhofer Institute
// for Manufacturing Engineering and Automation (IPA)
// See the file license.txt for copying permission.

// This program performs homing (referencing) of a device.
// See the user manual for details:
// https://github.com/ipa-tys/canopen/blob/master/doc/usermanual.pdf?raw=true

#include <utility>
#include "cia_402.h"

int main(int argc, char *argv[]) {

	if (argc != 6) {
		std::cout << "Arguments:" << std::endl
		<< "(1) device file" << std::endl
		<< "(2) CAN deviceID" << std::endl
		<< "(3) sync rate [msec]" << std::endl
		<< "(4) target velocity [rad/sec]" << std::endl
		<< "(5) acceleration [rad/sec^2]" << std::endl
		<< "(enter acceleration '0' to omit acceleration phase)" << std::endl
		<< "Example 1: ./move_device /dev/pcan32 12 10 0.2 0.05" << std::endl
		<< "Example 2 (reverse direction): "
		<< "./move_device /dev/pcan32 12 10 -0.2 -0.05" << std::endl;
		return -1;
	}
	std::cout << "Interrupt motion with Ctrl-C" << std::endl;
	std::string deviceFile = std::string(argv[1]);
	uint16_t CANid = std::stoi(std::string(argv[2]));

    cia_402 *cia_402_obj = new cia_402();

    std::map <uint8_t,cia_402::DeviceGroup::device_ptr> devs;

    cia_402::DeviceGroup::device_ptr device(new cia_402::Device(CANid) );

    devs[CANid] = device;

    deviceGroups_402["name"].setDevices(devs);
    deviceGroups_402["name"].setDeviceFile(deviceFile);


    uint32_t syncInterval = std::stoi(std::string(argv[3]));
    deviceGroups_402["name"].setSyncInterval(syncInterval);

	double targetVel = std::stod(std::string(argv[4]));
	double accel = std::stod(std::string(argv[5]));

    if (!canopen::openConnection(deviceFile))
    {
           std::cout << "Cannot open CAN device; aborting." << std::endl;
           exit(EXIT_FAILURE);
    }
    else
    {
           std::cout << "Connection to CAN bus established" << std::endl;
    }

    cia_402_obj->pre_init("name");

    cia_402::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m, std::string chainName) { cia_402::defaultPDO_incoming(CANid, m, "name" ); };
    cia_402::sendPos = cia_402_obj->defaultPDOOutgoing;
//////////////////////////////////////////////

//    canopen::listener_threads[deviceFile] = std::thread(cia_402_object->defaultListener, cia_402_object->deviceGroups["name"]);

//    for(auto& thread : canopen::listener_threads)
//    {
//        thread.second.detach();
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//    }

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    canopen::sendSDO(CANid, cia_402_object->CONTROLWORD, cia_402_object->CONTROLWORD_FAULT_RESET_1, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));

//    canopen::sendSDO(CANid, cia_402_object->CONTROLWORD, cia_402_object->CONTROLWORD_SHUTDOWN, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));


//    canopen::sendSDO(CANid, cia_402_object->CONTROLWORD, cia_402_object->CONTROLWORD_SWITCH_ON, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    canopen::sendSDO(CANid, cia_402_object->CONTROLWORD, cia_402_object->CONTROLWORD_ENABLE_OPERATION, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));

//    canopen::sendSDO(CANid, cia_402_object->STATUSWORD, deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    std::cout << "rgfsrestszhdrsvrfsrwetvrerxtrvx" << cia_402_object->deviceGroups["name"]->getDevices()[CANid]->getdeviceStateMachine() << std::endl;

///////////////////////////////////////////
//    canopen::listener_threads[deviceFile] = std::thread(cia_402_object->defaultListener, cia_402_object->deviceGroups["name"]);
//    std::cout << "sjdhafkjdhslwertbsretrv" << std::endl;
//    cia_402_object->setMotorState(CANid, cia_402_object->MS_SWITCHED_ON_DISABLED, cia_402_object->deviceGroups["name"]->getDevices()[CANid], deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    std::cout << "EWRBJEWGRWEHJREJHWGRJWERHJGWEHJRGEHWRJGWEHJGRWEJHGRJWEGRHGWEHJRGWEJ" << std::endl;
//    cia_402_object->setMotorState(CANid, cia_402_object->MS_READY_TO_SWITCH_ON, cia_402_object->deviceGroups["name"]->getDevices()[CANid], deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    std::cout << "sjdhafkjdhslwertbsretrv" << std::endl;

//    cia_402_object->setMotorState(CANid, cia_402_object->MS_SWITCHED_ON, cia_402_object->deviceGroups["name"]->getDevices()[CANid], deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    cia_402_object->setMotorState(CANid, cia_402_object->MS_OPERATION_ENABLED, cia_402_object->deviceGroups["name"]->getDevices()[CANid], deviceFile);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    std::cout << "INITIALIZING THE DEVICE" << std::endl;
    cia_402_obj->init("name", std::chrono::milliseconds(std::stoi(std::string(argv[3]))));//std::chrono::milliseconds(cia_402_object->deviceGroups["name"].getSyncInterval()));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "INITIALIZATION OF THE DEVICE IS CONCLUDED" << std::endl;

    canopen::sendSDO(CANid, MODES_OF_OPERATION, (uint8_t)MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE, deviceFile);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));


    cia_402_obj->manager_threads["name"] = std::thread(cia_402::deviceManager,"name");

    for(auto& thread : cia_402_obj->manager_threads)
    {
           thread.second.detach();
    }

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    deviceGroups_402["name"].getDevices()[CANid]->setInitialized(true);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));


	if (accel != 0) {  // accel of 0 means "move at target vel immediately"
	 	std::chrono::milliseconds accelerationTime( static_cast<int>(round( 1000.0 * targetVel / accel)) );
		double vel = 0;
		auto startTime = std::chrono::high_resolution_clock::now();
		auto tic = std::chrono::high_resolution_clock::now();

		// increasing velocity ramp up to target velocity:
		std::cout << "Accelerating to target velocity" << std::endl;
        while (tic < startTime + accelerationTime)
        {
			tic = std::chrono::high_resolution_clock::now();
			vel = accel * 0.000001 * std::chrono::duration_cast<std::chrono::microseconds>(tic-startTime).count();

            deviceGroups_402["name"].getDevices()[CANid]->setDesiredVel(vel);
            std::cout << vel << std::endl;
            std::cout << deviceGroups_402["name"].getDevices()[CANid]->getDesiredVel() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(deviceGroups_402["name"].getSyncInterval()) - (std::chrono::high_resolution_clock::now() - tic));
		}
	}

	// constant velocity when target vel has been reached:
	std::cout << "Target velocity reached!" << std::endl;

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

}
