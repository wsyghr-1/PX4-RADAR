/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef GPS_VELOCITY_NED_HPP
#define GPS_VELOCITY_NED_HPP

#include <lib/mathlib/mathlib.h>
#include <uORB/topics/sensor_gps.h>

class MavlinkStreamGpsVelocityNed : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGpsVelocityNed(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_VELOCITY_NED"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_VELOCITY_NED; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_gps_sub.advertised()
		       ? (MAVLINK_MSG_ID_GPS_VELOCITY_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)
		       : 0;
	}

private:
	explicit MavlinkStreamGpsVelocityNed(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};

	static int16_t velocityToCmS(float velocity_m_s)
	{
		const float velocity_cm_s = math::constrain(velocity_m_s * 100.f, -32768.f, 32767.f);
		return static_cast<int16_t>(roundf(velocity_cm_s));
	}

	static uint16_t accuracyToCmS(float accuracy_m_s)
	{
		if (!PX4_ISFINITE(accuracy_m_s) || (accuracy_m_s <= 0.f)) {
			return UINT16_MAX;
		}

		const float accuracy_cm_s = math::constrain(accuracy_m_s * 100.f, 0.f, 65534.f);
		return static_cast<uint16_t>(roundf(accuracy_cm_s));
	}

	bool send() override
	{
		sensor_gps_s gps;

		if (_sensor_gps_sub.update(&gps)) {
			mavlink_gps_velocity_ned_t msg{};

			msg.time_usec = gps.timestamp;

			const bool velocity_valid = gps.vel_ned_valid
						    && (gps.fix_type >= sensor_gps_s::FIX_TYPE_3D)
						    && PX4_ISFINITE(gps.vel_n_m_s)
						    && PX4_ISFINITE(gps.vel_e_m_s)
						    && PX4_ISFINITE(gps.vel_d_m_s);

			msg.valid = velocity_valid ? 1 : 0;

			if (velocity_valid) {
				msg.vn = velocityToCmS(gps.vel_n_m_s);
				msg.ve = velocityToCmS(gps.vel_e_m_s);
				msg.vd = velocityToCmS(gps.vel_d_m_s);
				msg.accuracy = accuracyToCmS(gps.s_variance_m_s);

			} else {
				msg.accuracy = UINT16_MAX;
			}

			mavlink_msg_gps_velocity_ned_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // GPS_VELOCITY_NED_HPP
