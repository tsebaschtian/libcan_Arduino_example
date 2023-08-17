#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include <canard.h>

#include <STM32_CAN.h>
#include <ArduinoUniqueID.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

/*
  libcanard library instance and a memory pool for it to use
 */
static CanardInstance canard;
static uint8_t memory_pool[1024];

/*
  in this example we will use dynamic node allocation if MY_NODE_ID is zero
 */
#define MY_NODE_ID 0

/*
  our preferred node ID if nobody else has it
 */
#define PREFERRED_NODE_ID 42

/*
  keep the state of 4 servos, simulating a 4 servo node
 */
#define NUM_SERVOS 4
static struct servo_state
{
    float position; // -1 to 1
    uint64_t last_update_us;
} servos[NUM_SERVOS];

/*
  a set of parameters to present to the user. In this example we don't
  actually save parameters, this is just to show how to handle the
  parameter protocool
 */
static struct parameter
{
    const char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float value;
    float min_value;
    float max_value;
} parameters[] = {
    {"CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, MY_NODE_ID, 0, 127},
    {"MyPID_P", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.2, 0.1, 5.0},
    {"MyPID_I", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.35, 0.1, 5.0},
    {"MyPID_D", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 0.025, 0.001, 1.0},
};

// some convenience macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

/*
  hold our node status as a static variable. It will be updated on any errors
 */
static struct uavcan_protocol_NodeStatus node_status;

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific
 */
uint64_t micros64(void);

/*
  get monotonic time in milliseconds since startup
 */
static uint32_t millis32(void);

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
void getUniqueID(uint8_t id[16]);

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);

/*
  handle a servo ArrayCommand request
*/
static void handle_ArrayCommand(CanardInstance *ins, CanardRxTransfer *transfer);

/*
  handle parameter GetSet request
 */
static void handle_param_GetSet(CanardInstance *ins, CanardRxTransfer *transfer);

/*
  handle parameter executeopcode request
 */
static void handle_param_ExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer);

/*
  data for dynamic node allocation process
 */
static struct
{
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;

/*
  handle a DNA allocation packet
 */
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer);

/*
  ask for a dynamic node allocation
 */
static void request_DNA();

/*
 This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);

/*
 This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 by the local node.
 If the callback returns true, the library will receive the transfer.
 If the callback returns false, the library will ignore the transfer.
 All transfers that are addressed to other nodes are always ignored.

 This function must fill in the out_data_type_signature to be the signature of the message.
 */
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id);

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
static void send_NodeStatus(void);

/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec);

/*
  send servo status at 25Hz
*/
static void send_ServoStatus(void);

/*
  Initializes the can interface
*/

void initCAN();

/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
static void processTxRxOnce();

void runCAN();

#endif