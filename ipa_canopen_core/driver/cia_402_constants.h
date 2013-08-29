#ifndef CIA_402_CONSTANTS_H
#define CIA_402_CONSTANTS_H

/***************************************************************/
//	define NMT constants, variables and functions
/***************************************************************/

const uint8_t NMT_START_REMOTE_NODE = 0x01;
const uint8_t NMT_STOP_REMOTE_NODE = 0x02;
const uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
const uint8_t NMT_RESET_NODE = 0x81;
const uint8_t NMT_RESET_COMMUNICATION = 0x82;

/***************************************************************/
//		define NMT error control constants
/***************************************************************/

const canopen::SDOkey HEARTBEAT(0x1017,0x0);

const uint16_t HEARTBEAT_TIME = 1500;

/***************************************************************/
//		Error Constants for Error Register
/***************************************************************/

static unsigned char const EMC_k_1001_GENERIC        = 0x01;
static unsigned char const EMC_k_1001_CURRENT        = 0x02;
static unsigned char const EMC_k_1001_VOLTAGE        = 0x04;
static unsigned char const EMC_k_1001_TEMPERATURE    = 0x08;
static unsigned char const EMC_k_1001_COMMUNICATION  = 0x10;
static unsigned char const EMC_k_1001_DEV_PROF_SPEC  = 0x20;
static unsigned char const EMC_k_1001_RESERVED       = 0x40;
static unsigned char const EMC_k_1001_MANUFACTURER   = 0x80;

/***************************************************************/
//		define motor state constants
/***************************************************************/

const std::string MS_NOT_READY_TO_SWITCH_ON = "NOT_READY_TO_SWITCH_ON";
const std::string MS_FAULT = "FAULT";
const std::string MS_SWITCHED_ON_DISABLED = "SWITCHED_ON_DISABLED";
const std::string MS_READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON";
const std::string MS_SWITCHED_ON = "SWITCHED_ON";
const std::string MS_OPERATION_ENABLED = "OPERATION_ENABLED";
const std::string MS_QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE";
const std::string MS_FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE";

/***************************************************************/
//		define SDO protocol constants and functions
/***************************************************************/

const canopen::SDOkey STATUSWORD(0x6041, 0x0);
const canopen::SDOkey ERRORWORD(0x1001, 0x0);
const canopen::SDOkey MANUFACTURER(0x1002, 0x0);
const canopen::SDOkey MANUFACTURERDEVICENAME(0x1008, 0x0);
const canopen::SDOkey MANUFACTURERHWVERSION(0x1009, 0x0);
const canopen::SDOkey MANUFACTURERSOFTWAREVERSION(0x100A, 0x0);

const canopen::SDOkey IDENTITYVENDORID(0x1018, 0x01);
const canopen::SDOkey IDENTITYPRODUCTCODE(0x1018, 0x02);
const canopen::SDOkey IDENTITYREVNUMBER(0x1018, 0x03);

/*************************
 * Specific for schunk hardware
 ************************/
const canopen::SDOkey SCHUNKLINE(0x200b, 0x1);
const canopen::SDOkey SCHUNKDETAIL(0x200b, 0x3);
/****************************************
 */

const canopen::SDOkey CONTROLWORD(0x6040, 0x0);
const canopen::SDOkey MODES_OF_OPERATION(0x6060, 0x0);
const canopen::SDOkey MODES_OF_OPERATION_DISPLAY(0x6061, 0x0);
const canopen::SDOkey SYNC_TIMEOUT_FACTOR(0x200e, 0x0);
const canopen::SDOkey IP_TIME_UNITS(0x60C2, 0x1);
const canopen::SDOkey IP_TIME_INDEX(0x60C2, 0x2);
const canopen::SDOkey ERROR_CODE(0x603F, 0x0);
const canopen::SDOkey ABORT_CONNECTION(0x6007, 0x0);
const canopen::SDOkey QUICK_STOP(0x605A, 0x0);
const canopen::SDOkey SHUTDOWN(0x605B, 0x0);
const canopen::SDOkey DISABLE_CODE(0x605C, 0x0);
const canopen::SDOkey HALT(0x605D, 0x0);
const canopen::SDOkey FAULT(0x605E, 0x0);
const canopen::SDOkey MODES(0x6060, 0x0);

const uint16_t CONTROLWORD_SHUTDOWN = 6;
const uint16_t CONTROLWORD_QUICKSTOP = 2;
const uint16_t CONTROLWORD_SWITCH_ON = 7;
const uint16_t CONTROLWORD_ENABLE_OPERATION = 15;
const uint16_t CONTROLWORD_START_HOMING = 16;
const uint16_t CONTROLWORD_ENABLE_IP_MODE = 16;
const uint16_t CONTROLWORD_DISABLE_INTERPOLATED = 7;
const uint16_t CONTROL_WORD_DISABLE_VOLTAGE = 0x7D;
const uint16_t CONTROLWORD_FAULT_RESET_0 = 0x00; //0x00;
const uint16_t CONTROLWORD_FAULT_RESET_1 = 0x80;
const uint16_t CONTROLWORD_HALT = 0x100;

const uint8_t MODES_OF_OPERATION_HOMING_MODE = 0x6;
const uint8_t MODES_OF_OPERATION_PROFILE_POSITION_MODE = 0x1;
const uint8_t MODES_OF_OPERATION_VELOCITY_MODE = 0x2;
const uint8_t MODES_OF_OPERATION_PROFILE_VELOCITY_MODE = 0x3;
const uint8_t MODES_OF_OPERATION_TORQUE_PROFILE_MODE = 0x4;
const uint8_t MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE = 0x7;

const int8_t IP_TIME_INDEX_MILLISECONDS = 0xFD;
const int8_t IP_TIME_INDEX_HUNDREDMICROSECONDS = 0xFC;
const uint8_t SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT = 0;
#endif // CIA_402_CONSTANTS_H
