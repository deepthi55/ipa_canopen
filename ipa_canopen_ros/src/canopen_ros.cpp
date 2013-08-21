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
 *   ROS package name: ipa_canopen_ros
 *
 * \author
 *   Author: Eduard Herkel, Thiago de Freitas, Tobias Sing
 * \author
 *   Supervised by: Eduard Herkel, Thiago de Freitas, Tobias Sing, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen.
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

#include "ros/ros.h"
#include <urdf/model.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <cia_402.h>
#include <XmlRpcValue.h>
#include <JointLimits.h>

/***************************************************************/
//  Specific types for the ROS NODE
/***************************************************************/

typedef boost::function<bool(cob_srvs::Trigger::Request&, cob_srvs::Trigger::Response&)> TriggerType;
typedef boost::function<void(const brics_actuator::JointVelocities&)> JointVelocitiesType;
typedef boost::function<bool(cob_srvs::SetOperationMode::Request&, cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;

std::map<std::string, JointLimits> joint_limits_;

std::vector<std::string> chainNames;
std::vector<std::string> jointNames;

/***************************************************************/
//			This function is responsible for the init callback
//          related to the specific DeviceGroup
/***************************************************************/

bool CANopenInit(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName)
{


    std::string deviceFile =cia_402::deviceGroups[chainName].getDeviceFile();

    cia_402::init(cia_402::deviceGroups[chainName], std::chrono::milliseconds(cia_402::deviceGroups[chainName].getSyncInterval()));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (auto device : cia_402::deviceGroups[chainName].getDevices())
    {
        canopen::sendSDO(device.second.getCANid(), cia_402::MODES_OF_OPERATION, cia_402::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE, deviceFile);
        std::cout << "Setting IP mode for: " << (uint16_t)device.second.getCANid() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cia_402::manager_threads.push_back(std::thread(cia_402::deviceManager,cia_402::deviceGroups[chainName]));

    for (auto device : cia_402::deviceGroups[chainName].getDevices())
    {
        device.second.setInitialized(true);
       // if(device.second.getHomingError())
         //   return false;

    }

    res.success.data = true;
    res.error_message.data = "";

    return true;
}

/***************************************************************/
//			Responsible for defining the current OperationMode
//          of the DeviceGroup
/***************************************************************/

bool setOperationModeCallback(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res, std::string chainName)
{
    res.success.data = true;  // for now this service is just a dummy, not used elsewhere
    return true;
}

/***************************************************************/
//			Responsible for defining the current Velocities
//          of the DeviceGroup
/***************************************************************/

void setVel(const brics_actuator::JointVelocities &msg, std::string chainName)
{
    if (!cia_402::atFirstInit & !canopen::recover_active)
    {
        std::vector<double> velocities;
        std::vector<double> positions;


        for (auto it : msg.velocities)
        {
            velocities.push_back( it.value);
        }

        for (auto device : cia_402::deviceGroups[chainName].getDevices())
        {
            positions.push_back((double)device.second.getDesiredPos());
        }


        //joint_limits_[chainName].checkVelocityLimits(velocities);
        //joint_limits_[chainName].checkPositionLimits(velocities, positions);

        cia_402::deviceGroups[chainName].setVel(velocities);
    }
}

/***************************************************************/
//			Read parameters from the yaml files for configuring
//          the canopen node
/***************************************************************/
void readParamsFromParameterServer(ros::NodeHandle n)
{

    if (!n.hasParam("device_groups") || !n.hasParam("chains"))
    {
        ROS_ERROR("Missing parameters on parameter server; shutting down node.");
        ROS_ERROR("Please consult the user manual for necessary parameter settings.");
        n.shutdown();
    }

    // Reading the DeviceGroups from the canopenmaster configuration
    XmlRpc::XmlRpcValue busParams;
    n.getParam("device_groups", busParams);

    XmlRpc::XmlRpcValue chainNames_XMLRPC;
    n.getParam("chains", chainNames_XMLRPC);

    for (int i=0; i<busParams.size(); i++)
    {

        auto name = static_cast<std::string>(busParams[i]["name"]);
        cia_402::deviceGroups[name] = cia_402::DeviceGroup();
        cia_402::deviceGroups[name].setBaudRate(static_cast<std::string>(busParams[i]["baudrate"]));
        cia_402::deviceGroups[name].setSyncInterval(static_cast<int>(busParams[i]["sync_interval"]));
        cia_402::deviceGroups[name].setDeviceFile(static_cast<std::string>(busParams[i]["device_file"]));
        // Reading the chains from the canopenmaster configuration
        chainNames.push_back(static_cast<std::string>(chainNames_XMLRPC[i]));
        std::cout << chainNames[i] << std::endl;
    }

    // Looping through the chains to get specific information according to the individual
    // yaml file. p.ex: arm_controller (joint_names, module_ids, ...)

    for (int cN=0; cN<chainNames_XMLRPC.size(); cN++)
    {

        std::map<uint8_t, canopen::Device> devs;
        jointNames.clear();
        XmlRpc::XmlRpcValue jointNames_XMLRPC;
        n.getParam("/" + chainNames[cN] + "/joint_names", jointNames_XMLRPC);

        for (int i=0; i<jointNames_XMLRPC.size(); i++)
        {
            jointNames.push_back(static_cast<std::string>(jointNames_XMLRPC[i]));
            std::cout << jointNames_XMLRPC[i] << std::endl;
        }



        XmlRpc::XmlRpcValue moduleIDs_XMLRPC;
        n.getParam("/" + chainNames[cN] + "/module_ids", moduleIDs_XMLRPC);
        std::vector<uint8_t> moduleIDs;
        for (int i=0; i<moduleIDs_XMLRPC.size(); i++)
        {
            moduleIDs.push_back(static_cast<int>(moduleIDs_XMLRPC[i]));
            std::cout << moduleIDs_XMLRPC[i] << std::endl;
        }

        XmlRpc::XmlRpcValue devices_XMLRPC;
        n.getParam("/" + chainNames[cN] + "/devices", devices_XMLRPC);
        std::vector<std::string> devices;
        for (int i=0; i<devices_XMLRPC.size(); i++)
        {
            devices.push_back(static_cast<std::string>(devices_XMLRPC[i]));
            std::cout << devices_XMLRPC[i] << std::endl;
        }


        XmlRpc::XmlRpcValue device_files_XMLRPC;
        n.getParam("/" + chainNames[cN] + "/device_files", device_files_XMLRPC);
        std::vector<std::string> device_files;
        for (int i=0; i<device_files_XMLRPC.size(); i++)
        {
            device_files.push_back(static_cast<std::string>(device_files_XMLRPC[i]));
            std::cout << device_files_XMLRPC[i] << std::endl;
        }


        std::map<uint8_t, cia_402::Device> dev_vec;

        for (unsigned int i=0; i<jointNames.size(); i++)
        {
            dev_vec [moduleIDs[i]] = cia_402::Device(moduleIDs[i], jointNames[i], chainNames[cN], devices[i]);
            std::cout << moduleIDs[i]<<  jointNames[i]<< chainNames[cN]<< devices[i] << std::endl;
        }
        auto name = static_cast<std::string>(busParams[cN]["name"]);
        cia_402::deviceGroups[name].setDevices(dev_vec);

    }

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "canopen_ros");
    ros::NodeHandle n(""); // ("~");

    readParamsFromParameterServer(n);

    for (auto dg : cia_402::deviceGroups)
    {
        std::string deviceFile = dg.second.getDeviceFile();

        std::cout << deviceFile << std::endl;

        if (!canopen::openConnection(deviceFile))
        {
               ROS_ERROR("Cannot open CAN device; aborting.");
               exit(EXIT_FAILURE);
        }
        else
        {
               std::cout << "Connection to CAN bus established" << std::endl;
        }


        cia_402::pre_init(dg.second);
    }

    /***************************************************************/
    //			defining ROS services and callbacks
    /***************************************************************/

    std::vector<TriggerType> initCallbacks;
    std::vector<ros::ServiceServer> initServices;

    std::vector<SetOperationModeCallbackType> setOperationModeCallbacks;
    std::vector<ros::ServiceServer> setOperationModeServices;

    std::vector<JointVelocitiesType> jointVelocitiesCallbacks;
    std::vector<ros::Subscriber> jointVelocitiesSubscribers;

    std::map<std::string, ros::Publisher> currentOperationModePublishers;
    std::map<std::string, ros::Publisher> statePublishers;

    ros::Publisher jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher diagnosticsPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);


    /***************************************************************/
    //			Configuring the ROS Node and Initializing the callbacks
    /***************************************************************/

    for (auto it : cia_402::deviceGroups)
      {
          ROS_INFO("Configuring %s", it.first.c_str());

          initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
          initServices.push_back( n.advertiseService("/" + it.first + "/init", initCallbacks.back()) );

          setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
          setOperationModeServices.push_back( n.advertiseService("/" + it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );

          jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
          jointVelocitiesSubscribers.push_back( n.subscribe<brics_actuator::JointVelocities>("/" + it.first + "/command_vel", 1, jointVelocitiesCallbacks.back()) );

          currentOperationModePublishers[it.first] = n.advertise<std_msgs::String>("/" + it.first + "/current_operationmode", 1);

          statePublishers[it.first] = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/" + it.first + "/state", 1);

          std::cout <<"IT FIRSTTTT" << it.first <<  std::endl;
      }

      //This defines the loop rate for the ROS node
      double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(10)).count();

      ros::Rate loop_rate(lr);

}
