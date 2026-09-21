# Internal interface for controlling an external lightweight recorder over MAVLink.

uint64 timestamp

uint8 MESSAGE_TYPE_COMMAND = 0
uint8 MESSAGE_TYPE_ACK = 1
uint8 MESSAGE_TYPE_STATUS = 2
uint8 message_type

uint8 MAVLINK_CHANNEL_ALL = 255
uint8 mavlink_channel
# Source of a received ACK or STATUS. Zero for an outgoing COMMAND.
uint8 source_system
uint8 source_component

uint8 CMD_START = 0
uint8 CMD_STOP = 1
uint8 CMD_QUERY = 2
uint8 cmd

uint8 RESULT_OK = 0
uint8 RESULT_FAILED = 1
uint8 RESULT_BUSY = 2
uint8 result

uint8 STATE_IDLE = 0
uint8 STATE_RECORDING = 1
uint8 STATE_ERROR = 2
uint8 state

uint32 DATA_RADAR = 1
uint32 DATA_VISION = 2
uint32 DATA_FLIGHT_CONTROL = 4
uint32 DATA_MASK_SUPPORTED = 7
uint32 data_mask

uint16 req_id
char[32] file_name

uint8 ORB_QUEUE_LENGTH = 4
