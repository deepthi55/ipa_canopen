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

std::string deviceFile;

std::map<std::string, JointLimits> joint_limits_;

std::vector<std::string> chainNames;
std::vector<std::string> jointNames;

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

}
