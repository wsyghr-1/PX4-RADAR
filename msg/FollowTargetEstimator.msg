uint64 timestamp                     # time since system start (microseconds)
uint64 last_filter_reset_timestamp   # time of last filter reset (microseconds)

#================== Classification enum of the object ===================
uint8 RVFUSION_OBJUNIT_MSG_CLS_UNKNOWN = 0
uint8 RVFUSION_OBJUNIT_MSG_CLS_CAR = 1
uint8 RVFUSION_OBJUNIT_MSG_CLS_TRUCK = 2
uint8 RVFUSION_OBJUNIT_MSG_CLS_PEDESTRAIN = 3
uint8 RVFUSION_OBJUNIT_MSG_CLS_CYCLIST = 4
uint8 RVFUSION_OBJUNIT_MSG_CLS_BOAT = 5
uint8 RVFUSION_OBJUNIT_MSG_CLS_POWERTOWER = 6
uint8 RVFUSION_OBJUNIT_MSG_CLS_POWERLINE = 7
uint8 RVFUSION_OBJUNIT_MSG_CLS_GROUND = 8
uint8 RVFUSION_OBJUNIT_MSG_CLS_RAILWAY = 9

bool valid              # True if estimator states are okay to be used
bool stale              # True if estimator stopped receiving follow_target messages for some time. The estimate can still be valid, though it might be inaccurate.
uint8 cls               # Classiciation enum of the object

float64 lat_est         # Estimated target latitude
float64 lon_est         # Estimated target longitude
float32 alt_est         # Estimated target altitude

float32[3] pos_est      # Estimated target NED position (m)
float32[3] vel_est      # Estimated target NED velocity (m/s)
float32[3] acc_est      # Estimated target NED acceleration (m^2/s)

uint64 prediction_count
uint64 fusion_count
