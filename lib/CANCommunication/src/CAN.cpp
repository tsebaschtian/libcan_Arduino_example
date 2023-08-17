#include <CAN.h>

CANhandler *CANhandler::instance = nullptr;

CANhandler::CANhandler()
{
    instance = this; // Store the instance pointer

    parameters[0] = {"CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, MY_NODE_ID, 0, 127};
    parameters[1] = {"MyPID_P", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.2, 0.1, 5.0};
    parameters[2] = {"MyPID_I", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.35, 0.1, 5.0};
    parameters[3] = {"MyPID_D", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 0.025, 0.001, 1.0};
}

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific
 */
uint64_t CANhandler::micros64()
{
    return (uint64_t)micros();
}

/*
  get monotonic time in milliseconds since startup
 */
uint32_t CANhandler::millis32()
{
    return millis();
}

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
void CANhandler::getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    memcpy(id, UniqueID8, 8);
    memcpy(id + 8, UniqueID8, 8);
}

/*
  handle a GetNodeInfo request
*/
void CANhandler::handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    Serial.print("GetNodeInfo request from ");
    Serial.println(transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = millis32() / 1000UL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = 1;
    pkt.software_version.minor = 2;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char *)pkt.name.data, "ServoNode", sizeof(pkt.name.data));
    pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle a servo ArrayCommand request
*/
void CANhandler::handle_ArrayCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_equipment_actuator_ArrayCommand cmd;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd))
    {
        return;
    }
    uint64_t tnow = micros64();
    for (uint8_t i = 0; i < cmd.commands.len; i++)
    {
        if (cmd.commands.data[i].actuator_id >= NUM_SERVOS)
        {
            // not for us
            continue;
        }
        switch (cmd.commands.data[i].command_type)
        {
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
            servos[cmd.commands.data[i].actuator_id].position = cmd.commands.data[i].command_value;
            break;
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
            // map PWM to -1 to 1, assuming 1500 trim. If the servo has natural PWM
            // support then we should use it directly instead
            servos[cmd.commands.data[i].actuator_id].position = (cmd.commands.data[i].command_value - 1500) / 500.0;
            break;
        }
        Serial.print("Servo Id: ");
        Serial.print(cmd.commands.data[i].actuator_id);
        Serial.print(" value: ");
        Serial.println(cmd.commands.data[i].command_value);

        servos[cmd.commands.data[i].actuator_id].last_update_us = tnow;
    }
}

/*
  handle parameter GetSet request
 */
void CANhandler::handle_param_GetSet(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req))
    {
        return;
    }

    struct parameter *p = NULL;
    if (req.name.len != 0)
    {
        for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
        {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0)
            {
                p = &parameters[i];
                break;
            }
        }
    }
    else if (req.index < ARRAY_SIZE(parameters))
    {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY)
    {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
        switch (p->type)
        {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            p->value = req.value.integer_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            p->value = req.value.real_value;
            break;
        default:
            return;
        }
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL)
    {
        pkt.value.union_tag = p->type;
        switch (p->type)
        {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            pkt.value.integer_value = p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            pkt.value.real_value = p->value;
            break;
        default:
            return;
        }
        pkt.name.len = strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter executeopcode request
 */
void CANhandler::handle_param_ExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req))
    {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
    {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
    {
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle a DNA allocation packet
 */
void CANhandler::handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
    {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        Serial.printf("Allocation request from another allocatee\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0)
    {
        Serial.print("Mismatching allocation response\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data))
    {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        Serial.print("Matching allocation response: ");
        Serial.println(msg.unique_id.len);
    }
    else
    {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
        Serial.print("Node ID allocated: ");
        Serial.println(msg.node_id);
    }
}

/*
  ask for a dynamic node allocation
 */
void CANhandler::request_DNA()
{
    const uint32_t now = millis32();
    uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0)
    {
        allocation_request[0] |= 1; // First part of unique ID
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);

    const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);

    if (uid_size > MaxLenOfUniqueIDInRequest)
    {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&canard,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t)(uid_size + 1));
    if (bcast_res < 0)
    {
        Serial.print("Could not broadcast ID allocation req; error ");
        Serial.println(bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}

/*
 This callback is invoked by the library when a new message or request or response is received.
*/
void CANhandler::onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            handle_GetNodeInfo(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            handle_param_GetSet(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        {
            handle_param_ExecuteOpcode(ins, transfer);
            break;
        }
        }
    }
    if (transfer->transfer_type == CanardTransferTypeBroadcast)
    {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id)
        {
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
        {
            handle_ArrayCommand(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            handle_DNA_Allocation(ins, transfer);
            break;
        }
        }
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
bool CANhandler::shouldAcceptTransfer(const CanardInstance *ins,
                                      uint64_t *out_data_type_signature,
                                      uint16_t data_type_id,
                                      CanardTransferType transfer_type,
                                      uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        }
    }
    if (transfer_type == CanardTransferTypeBroadcast)
    {
        // see if we want to handle a specific broadcast packet
        switch (data_type_id)
        {
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        }
    }
    // we don't want any other messages
    return false;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
void CANhandler::send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void CANhandler::process1HzTasks(uint64_t timestamp_usec)
{
    // Serial.println("1Hz task started");
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  send servo status at 25Hz
*/
void CANhandler::send_ServoStatus(void)
{
    // send a separate status packet for each servo
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    {
        struct uavcan_equipment_actuator_Status pkt;
        memset(&pkt, 0, sizeof(pkt));
        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];

        // make up some synthetic status data
        pkt.actuator_id = i;
        pkt.position = servos[i].position;
        pkt.force = 3.5 * servos[i].position;
        pkt.speed = 0.12; // m/s or rad/s
        pkt.power_rating_pct = 17;

        uint32_t len = uavcan_equipment_actuator_Status_encode(&pkt, buffer);

        // we need a static variable for the transfer ID. This is
        // incremeneted on each transfer, allowing for detection of packet
        // loss
        static uint8_t transfer_id;

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                        UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
    }
}

/*
  Initializes the can interface
*/

STM32_CAN Can(CAN1, DEF); // Use PA11/12 pins for CAN1.

uint32_t next_1hz_service_at = 0;
uint32_t next_25hz_service_at = 0;

void CANhandler::initCAN()
{
    /*
      Initializing the STM32_CAN Interface
    */
    Can.begin();
    Can.setBaudRate(1000000);

    /*
     Initializing the Libcanard instance.
     */
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceivedStatic,
               shouldAcceptTransferStatic,
               NULL);

    if (MY_NODE_ID > 0)
    {
        canardSetLocalNodeID(&canard, MY_NODE_ID);
    }
    else
    {
        Serial.printf("Waiting for DNA node allocation\n");
    }
    next_1hz_service_at = millis32();
    next_25hz_service_at = millis32();
}

/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
void CANhandler::processTxRxOnce()
{
    // Transmitting
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        static CAN_message_t CAN_TX_msg; // needs to be static

        CAN_TX_msg.flags.extended = true;
        CAN_TX_msg.id = txf->id;
        CAN_TX_msg.len = txf->data_len;
        memcpy(CAN_TX_msg.buf, txf->data, txf->data_len);

        Can.write(CAN_TX_msg);
        canardPopTxQueue(&canard);
    }

    // Receiving
    const uint32_t start_ms = millis32();

    CanardCANFrame rx_frame;         // new canard frame for RX
    static CAN_message_t CAN_RX_msg; // new STM32_CAN frame for receiving, needs to be static

    bool rx_res = Can.read(CAN_RX_msg); // read from can interface
    if (rx_res > 0)
    {
        // translate STM32_CAN frame to canard frame
        rx_frame.id = CAN_RX_msg.id;
        rx_frame.data_len = CAN_RX_msg.len;
        rx_frame.iface_id = 0;
        rx_frame.id |= CANARD_CAN_FRAME_EFF;  // Set the EFF bit to 1
        rx_frame.id &= ~CANARD_CAN_FRAME_RTR; // Set the RTR bit to 0
        rx_frame.id &= ~CANARD_CAN_FRAME_ERR; // Set the ERR bit to 0
        memcpy(rx_frame.data, CAN_RX_msg.buf, CAN_RX_msg.len);

        if (false)
        {
            // Serial output for debug stuff
            Serial.print("Channel: ");
            Serial.print(CAN_RX_msg.bus);
            if (CAN_RX_msg.flags.extended == false)
            {
                Serial.print(" Standard ID:");
            }
            else
            {
                Serial.print(" Extended ID:");
            }
            Serial.print(CAN_RX_msg.id, HEX);

            Serial.print(" DLC: ");
            Serial.print(CAN_RX_msg.len);
            if (CAN_RX_msg.flags.remote == false)
            {
                Serial.print(" buf: ");
                for (int i = 0; i < CAN_RX_msg.len; i++)
                {
                    Serial.print("0x");
                    Serial.print(CAN_RX_msg.buf[i], HEX);
                    if (i != (CAN_RX_msg.len - 1))
                        Serial.print(" ");
                }
                Serial.println();
            }
            else
            {
                Serial.println(" Data: REMOTE REQUEST FRAME");
            }

            // same but for canard frame
            Serial.print("Canard frame: ");
            Serial.print(rx_frame.id, HEX);

            Serial.print(" DLC: ");
            Serial.print(rx_frame.data_len);

            Serial.print(" buf: ");
            for (int i = 0; i < rx_frame.data_len; i++)
            {
                Serial.print("0x");
                Serial.print(rx_frame.data[i], HEX);
                if (i != (rx_frame.data_len - 1))
                    Serial.print(" ");
            }
            Serial.println();
        }

        // handle the received frame
        int16_t result = canardHandleRxFrame(&canard, &rx_frame, micros64());
        // Serial.println(result);
    }
}

void CANhandler::runCAN()
{
    processTxRxOnce();

    const uint64_t ts = micros64();

    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        // waiting for DNA
    }

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        // we're still waiting for a DNA allocation of our node ID
        if (millis32() > DNA.send_next_node_id_allocation_request_at_ms)
        {
            request_DNA();
        }
    }
    else
    {

        if (ts / 1000 >= next_1hz_service_at)
        {
            next_1hz_service_at += 1000;
            process1HzTasks(ts);
        }
        if (ts / 1000 >= next_25hz_service_at)
        {
            next_25hz_service_at += 1000 / 25;
            send_ServoStatus();
        }
    }
}