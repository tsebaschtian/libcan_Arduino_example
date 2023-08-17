#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include <canard.h>

#include <STM32_CAN.h>
#include <ArduinoUniqueID.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

#define NUM_SERVOS 4

/*
  in this example we will use dynamic node allocation if MY_NODE_ID is zero
*/
#define MY_NODE_ID 0
/*
  our preferred node ID if nobody else has it
 */
#define PREFERRED_NODE_ID 42

// some convenience macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

class CANhandler
{
public:
  CANhandler();

  void initCAN(); // Initializes the can interface
  void runCAN();

  /*
  keep the state of 4 servos, simulating a 4 servo node
  */
  struct servo_state
  {
    float position; // -1 to 1
    uint64_t last_update_us;
  };
  servo_state servos[NUM_SERVOS];

private:
  static CANhandler *instance; // pointer to instance
  /*
    libcanard library instance and a memory pool for it to use
   */
  CanardInstance canard;
  uint8_t memory_pool[1024];

  struct parameter
  {
    const char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float value;
    float min_value;
    float max_value;
  };
  parameter parameters[4];

  /*
    data for dynamic node allocation process
   */
  struct dna_data
  {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
  };
  dna_data DNA;

  /*
    hold our node status as a static variable. It will be updated on any errors
   */
  struct uavcan_protocol_NodeStatus node_status;

  /*
    get a 64 bit monotonic timestamp in microseconds since start. This
    is platform specific
   */
  uint64_t micros64(void);

  /*
    get monotonic time in milliseconds since startup
   */
  uint32_t millis32(void);

  /*
    get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
   */
  void getUniqueID(uint8_t id[16]);

  /*
    handle a GetNodeInfo request
  */
  void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);

  /*
    handle a servo ArrayCommand request
  */
  void handle_ArrayCommand(CanardInstance *ins, CanardRxTransfer *transfer);

  /*
    handle parameter GetSet request
   */
  void handle_param_GetSet(CanardInstance *ins, CanardRxTransfer *transfer);

  /*
    handle parameter executeopcode request
   */
  void handle_param_ExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer);

  /*
    handle a DNA allocation packet
   */
  void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer);

  /*
    ask for a dynamic node allocation
   */
  void request_DNA();

  /*
   This callback is invoked by the library when a new message or request or response is received.
  */
  void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);

  static void onTransferReceivedStatic(CanardInstance *ins, CanardRxTransfer *transfer)
  {
    if (instance)
    {
      instance->onTransferReceived(ins, transfer);
    }
  }

  /*
   This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
   by the local node.
   If the callback returns true, the library will receive the transfer.
   If the callback returns false, the library will ignore the transfer.
   All transfers that are addressed to other nodes are always ignored.

   This function must fill in the out_data_type_signature to be the signature of the message.
   */
  bool shouldAcceptTransfer(const CanardInstance *ins,
                            uint64_t *out_data_type_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id);

  static bool shouldAcceptTransferStatic(const CanardInstance *ins,
                                         uint64_t *out_data_type_signature,
                                         uint16_t data_type_id,
                                         CanardTransferType transfer_type,
                                         uint8_t source_node_id)
  {
    return instance->shouldAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
  }

  /*
    send the 1Hz NodeStatus message. This is what allows a node to show
    up in the DroneCAN GUI tool and in the flight controller logs
   */
  void send_NodeStatus(void);

  /*
    This function is called at 1 Hz rate from the main loop.
  */
  void process1HzTasks(uint64_t timestamp_usec);

  /*
    send servo status at 25Hz
  */
  void send_ServoStatus(void);

  /*
    Transmits all frames from the TX queue, receives up to one frame.
  */
  void processTxRxOnce();
};

#endif