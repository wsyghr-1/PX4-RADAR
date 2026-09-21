uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 wheel_rate_integ	# FW only and optional


# rate controller proportional status
float32 rollspeed_prop
float32 pitchspeed_prop
float32 yawspeed_prop

# rate controller derivative status
float32 rollspeed_deri
float32 pitchspeed_deri
float32 yawspeed_deri
