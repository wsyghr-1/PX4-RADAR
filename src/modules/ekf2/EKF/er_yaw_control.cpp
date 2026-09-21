/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/**
 * @file er_yaw_control.cpp
 * Control functions for ekf external radar yaw fusion
 */

#include "ekf.h"

void Ekf::controlErYawFusion(const extRadarSample &er_sample, const bool common_starting_conditions_passing,
			     const bool er_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "ER yaw";

	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = er_sample.time_us;
	aid_src.observation = getEulerYaw(er_sample.quat);
	aid_src.observation_variance = math::max(er_sample.orientation_var(2), _params.er_att_noise, sq(0.01f));
	aid_src.innovation = wrap_pi(getEulerYaw(_R_to_earth) - aid_src.observation);

	if (er_reset) {
		_control_status.flags.er_yaw_fault = false;
	}

	// determine if we should use ER yaw aiding
	bool continuing_conditions_passing = (_params.er_ctrl & static_cast<int32_t>(ErCtrl::YAW))
					     && _control_status.flags.tilt_align
					     && !_control_status.flags.er_yaw_fault
					     && PX4_ISFINITE(aid_src.observation)
					     && PX4_ISFINITE(aid_src.observation_variance);

	// if GPS enabled don't allow ER yaw if ER isn't NED
	if (_control_status.flags.gps && _control_status.flags.yaw_align
	    && (er_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED)
	   ) {
		continuing_conditions_passing = false;

		// use delta yaw for innovation logging
		aid_src.innovation = wrap_pi(wrap_pi(getEulerYaw(_R_to_earth) - _er_yaw_pred_prev)
					     - wrap_pi(getEulerYaw(er_sample.quat) - getEulerYaw(_er_sample_prev.quat)));
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& isTimedOut(aid_src.time_last_fuse, (uint32_t)1e6);

	if (_control_status.flags.er_yaw) {
		if (continuing_conditions_passing) {

			if (er_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					//_information_events.flags.reset_yaw_to_radar = true; // TODO
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// ER has reset, but quality isn't sufficient
					// we have no choice but to stop ER and try to resume once quality is acceptable
					stopErYawFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseYaw(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max);

			if (is_fusion_failing) {
				if ((_nb_er_yaw_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					//_information_events.flags.reset_yaw_to_radar = true; // TODO
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_er_yaw_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.er_yaw_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopErYawFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopErYawFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopErYawFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion
			if (er_sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

				if (_control_status.flags.yaw_align) {
					ECL_INFO("starting %s fusion", AID_SRC_NAME);

				} else {
					// reset yaw to ER and set yaw_align
					ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					_control_status.flags.yaw_align = true;
				}

				aid_src.time_last_fuse = _time_delayed_us;
				_information_events.flags.starting_radar_yaw_fusion = true;
				_control_status.flags.er_yaw = true;

			} else if (er_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
				// turn on fusion of external radar yaw measurements
				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);

				// reset yaw to ER
				resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
				aid_src.time_last_fuse = _time_delayed_us;

				_information_events.flags.starting_radar_yaw_fusion = true;
				_control_status.flags.yaw_align = false;
				_control_status.flags.er_yaw = true;
			}

			if (_control_status.flags.er_yaw) {
				_nb_er_yaw_reset_available = 5;
			}
		}
	}
}

void Ekf::stopErYawFusion()
{
#if defined(CONFIG_EKF2_EXTERNAL_RADAR)
	if (_control_status.flags.er_yaw) {
		resetEstimatorAidStatus(_aid_src_er_yaw);

		_control_status.flags.er_yaw = false;
	}
#endif // CONFIG_EKF2_EXTERNAL_RADAR
}
