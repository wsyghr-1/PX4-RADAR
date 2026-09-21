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
 * @file er_control.cpp
 * Control functions for ekf external radar control
 */

#include "ekf.h"

void Ekf::controlExternalRadarFusion()
{
	_er_pos_b_est.predict(_dt_ekf_avg);
	_er_hgt_b_est.predict(_dt_ekf_avg);

	// Check for new external radar data
	extRadarSample er_sample;

	if (_ext_radar_buffer && _ext_radar_buffer->pop_first_older_than(_time_delayed_us, &er_sample)) {

		bool er_reset = (er_sample.reset_counter != _er_sample_prev.reset_counter);

		// determine if we should use the horizontal position observations
		bool quality_sufficient = (_params.er_quality_minimum <= 0) || (er_sample.quality >= _params.er_quality_minimum);

		const bool starting_conditions_passing = quality_sufficient
				&& ((er_sample.time_us - _er_sample_prev.time_us) < ER_MAX_INTERVAL)
				&& ((_params.er_quality_minimum <= 0) || (_er_sample_prev.quality >= _params.er_quality_minimum)) // previous quality sufficient
				&& ((_params.er_quality_minimum <= 0) || (_ext_radar_buffer->get_newest().quality >= _params.er_quality_minimum)) // newest quality sufficient
				&& isNewestSampleRecent(_time_last_ext_radar_buffer_push, ER_MAX_INTERVAL);

		updateErAttitudeErrorFilter(er_sample, er_reset);
		controlErYawFusion(er_sample, starting_conditions_passing, er_reset, quality_sufficient, _aid_src_er_yaw);
		controlErVelFusion(er_sample, starting_conditions_passing, er_reset, quality_sufficient, _aid_src_er_vel);
		controlErPosFusion(er_sample, starting_conditions_passing, er_reset, quality_sufficient, _aid_src_er_pos);
		controlErHeightFusion(er_sample, starting_conditions_passing, er_reset, quality_sufficient, _aid_src_er_hgt);

		if (quality_sufficient) {
			_er_sample_prev = er_sample;
		}

		// record corresponding yaw state for future ER delta heading innovation (logging only)
		_er_yaw_pred_prev = getEulerYaw(_state.quat_nominal);

	} else if ((_control_status.flags.er_pos || _control_status.flags.er_vel || _control_status.flags.er_yaw
		    || _control_status.flags.er_hgt)
		   && isTimedOut(_er_sample_prev.time_us, 2 * ER_MAX_INTERVAL)) {

		// Turn off ER fusion mode if no data has been received
		stopErPosFusion();
		stopErVelFusion();
		stopErYawFusion();
		stopErHgtFusion();

		_er_q_error_initialized = false;

		_warning_events.flags.radar_data_stopped = true;
		ECL_WARN("radar data stopped");
	}
}

void Ekf::updateErAttitudeErrorFilter(extRadarSample &er_sample, bool er_reset)
{
	const Quatf q_error((_state.quat_nominal * er_sample.quat.inversed()).normalized());

	if (!q_error.isAllFinite()) {
		return;
	}

	if (!_er_q_error_initialized || er_reset) {
		_er_q_error_filt.reset(q_error);
		_er_q_error_initialized = true;

	} else {
		_er_q_error_filt.update(q_error);
	}
}
