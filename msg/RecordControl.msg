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
uint8 STATE_STARTING = 3
uint8 STATE_WAIT_DATA = 4
uint8 STATE_VALIDATING = 5
uint8 STATE_STOPPING = 6
uint8 STATE_COMPLETED = 7
uint8 state

uint32 DATA_RADAR = 1
uint32 DATA_VISION = 2
uint32 DATA_FLIGHT_CONTROL = 4
uint32 DATA_MASK_SUPPORTED = 7
uint32 data_mask

uint8 sensor_type        # 哪些传感器: bit0=雷达, bit1=视觉, bit2=LiDAR, bit3=TOF
uint16 record_data_type  # 每传感器2bits数据类型

uint16 req_id
char[32] file_name

uint32 elapsed_s
uint32 frame_count
uint32 written_mib
uint32 free_mib
uint16 error_code

uint8 ORB_QUEUE_LENGTH = 4
