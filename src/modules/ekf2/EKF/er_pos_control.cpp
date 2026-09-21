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
 * @file er_pos_control.cpp
 * Control functions for ekf external radar position fusion
 */

#include "ekf.h"

static constexpr const char *ER_AID_SRC_NAME = "ER position";


void Ekf::controlErPosFusion(const extRadarSample &er_sample, const bool common_starting_conditions_passing,
			     const bool er_reset, const bool quality_sufficient, estimator_aid_source2d_s &aid_src)
{
	const bool yaw_alignment_changed = (!_control_status_prev.flags.er_yaw && _control_status.flags.er_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use ER position aiding
	bool continuing_conditions_passing = (_params.er_ctrl & static_cast<int32_t>(ErCtrl::HPOS))
					     && _control_status.flags.tilt_align
					     && PX4_ISFINITE(er_sample.pos(0))
					     && PX4_ISFINITE(er_sample.pos(1));

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.er_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	const bool bias_fusion_was_active = _er_pos_b_est.fusionActive();

	// rotate measurement into correct earth frame if required
	Vector3f pos{NAN, NAN, NAN};
	Matrix3f pos_cov{};

	switch (er_sample.pos_frame) {
	case PositionFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			pos = er_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(er_sample.position_var);

			if (_control_status.flags.gps) {
				_er_pos_b_est.setFusionActive();

			} else {
				_er_pos_b_est.setFusionInactive();
			}

		} else {
			continuing_conditions_passing = false;
			_er_pos_b_est.setFusionInactive();
			_er_pos_b_est.reset();
		}

		break;

	case PositionFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.er_yaw) {
			// using ER frame
			pos = er_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(er_sample.position_var);

			_er_pos_b_est.setFusionInactive();
			_er_pos_b_est.reset();

		} else {
			// rotate ER to the EKF reference frame
			const Dcmf R_er_to_ekf = Dcmf(_er_q_error_filt.getState());

			pos = R_er_to_ekf * er_sample.pos - pos_offset_earth;
			pos_cov = R_er_to_ekf * matrix::diag(er_sample.position_var) * R_er_to_ekf.transpose();

			// increase minimum variance to include ER orientation variance
			// TODO: do this properly
			const float orientation_var_max = er_sample.orientation_var.max();

			for (int i = 0; i < 2; i++) {
				pos_cov(i, i) = math::max(pos_cov(i, i), orientation_var_max);
			}

			if (_control_status.flags.gps) {
				_er_pos_b_est.setFusionActive();

			} else {
				_er_pos_b_est.setFusionInactive();
			}
		}

		break;

	default:
		continuing_conditions_passing = false;
		_er_pos_b_est.setFusionInactive();
		_er_pos_b_est.reset();
		break;
	}

#if defined(CONFIG_EKF2_GNSS)
	// increase minimum variance if GPS active (position reference)
	if (_control_status.flags.gps) {
		for (int i = 0; i < 2; i++) {
			pos_cov(i, i) = math::max(pos_cov(i, i), sq(_params.gps_pos_noise));
		}
	}
#endif // CONFIG_EKF2_GNSS

	const Vector2f measurement{pos(0), pos(1)};

	const Vector2f measurement_var{
		math::max(pos_cov(0, 0), sq(_params.er_pos_noise), sq(0.01f)),
		math::max(pos_cov(1, 1), sq(_params.er_pos_noise), sq(0.01f))
	};

	const bool measurement_valid = measurement.isAllFinite() && measurement_var.isAllFinite();

	// bias fusion activated (GPS activated)
	if (!bias_fusion_was_active && _er_pos_b_est.fusionActive()) {
		if (quality_sufficient) {
			// reset the bias estimator
			_er_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

		} else if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.er_pos)) {
			// otherwise stop ER position, when quality is good again it will restart with reset bias
			stopErPosFusion();
		}
	}

	updateHorizontalPositionAidSrcStatus(er_sample.time_us,
					     measurement - _er_pos_b_est.getBias(),        // observation
					     measurement_var + _er_pos_b_est.getBiasVar(), // observation variance
					     math::max(_params.er_pos_innov_gate, 1.f),    // innovation gate
					     aid_src);

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		_er_pos_b_est.setMaxStateNoise(Vector2f(sqrtf(measurement_var(0)), sqrtf(measurement_var(1))));
		_er_pos_b_est.setProcessNoiseSpectralDensity(_params.er_hgt_bias_nsd); // TODO
		_er_pos_b_est.fuseBias(measurement - Vector2f(_state.pos.xy()), measurement_var + Vector2f(getStateVariance<State::pos>()));
	}

	if (!measurement_valid) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	if (_control_status.flags.er_pos) {

		if (continuing_conditions_passing) {
			const bool bias_estimator_change = (bias_fusion_was_active != _er_pos_b_est.fusionActive());
			const bool reset = er_reset || yaw_alignment_changed || bias_estimator_change;

			updateErPosFusion(measurement, measurement_var, quality_sufficient, reset, aid_src);

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", ER_AID_SRC_NAME);
			stopErPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			startErPosFusion(measurement, measurement_var, aid_src);
		}
	}
}

void Ekf::startErPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src)
{
	// activate fusion
	// TODO:  (_params.position_sensor_ref == PositionSensor::ER)
	if (_control_status.flags.gps) {
		ECL_INFO("starting %s fusion", ER_AID_SRC_NAME);
		_er_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);
		_er_pos_b_est.setFusionActive();

	} else {
		ECL_INFO("starting %s fusion, resetting state", ER_AID_SRC_NAME);
		//_position_sensor_ref = PositionSensor::ER;
		_information_events.flags.reset_pos_to_radar = true;
		resetHorizontalPositionTo(measurement, measurement_var);
		_er_pos_b_est.reset();
	}

	aid_src.time_last_fuse = _time_delayed_us;

	_nb_er_pos_reset_available = 5;
	_information_events.flags.starting_radar_pos_fusion = true;
	_control_status.flags.er_pos = true;
}

void Ekf::updateErPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient, bool reset, estimator_aid_source2d_s &aid_src)
{
	if (reset) {

		if (quality_sufficient) {

			if (!_control_status.flags.gps) {
				ECL_INFO("reset to %s", ER_AID_SRC_NAME);
				_information_events.flags.reset_pos_to_radar = true;
				resetHorizontalPositionTo(measurement, measurement_var);
				_er_pos_b_est.reset();

			} else {
				_er_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);
			}

			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			// ER has reset, but quality isn't sufficient
			// we have no choice but to stop ER and try to resume once quality is acceptable
			stopErPosFusion();
			return;
		}

	} else if (quality_sufficient) {
		fuseHorizontalPosition(aid_src);

	} else {
		aid_src.innovation_rejected = true;
	}

	const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

	if (is_fusion_failing) {
		bool pos_xy_fusion_failing = isTimedOut(_time_last_hor_pos_fuse, _params.no_aid_timeout_max);

		if ((_nb_er_pos_reset_available > 0) && quality_sufficient) {
			// Data seems good, attempt a reset
			ECL_WARN("%s fusion failing, resetting", ER_AID_SRC_NAME);

			if (_control_status.flags.gps && !pos_xy_fusion_failing) {
				// reset ER position bias
				_er_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

			} else {
				_information_events.flags.reset_pos_to_radar = true;

				if (_control_status.flags.gps) {
					resetHorizontalPositionTo(measurement - _er_pos_b_est.getBias(), measurement_var + _er_pos_b_est.getBiasVar());
					_er_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

				} else {
					resetHorizontalPositionTo(measurement, measurement_var);
					_er_pos_b_est.reset();
				}
			}

			aid_src.time_last_fuse = _time_delayed_us;

			if (_control_status.flags.in_air) {
				_nb_er_pos_reset_available--;
			}

		} else {
			// A reset did not fix the issue but all the starting checks are not passing
			// This could be a temporary issue, stop the fusion without declaring the sensor faulty
			ECL_WARN("stopping %s, fusion failing", ER_AID_SRC_NAME);
			stopErPosFusion();
		}
	}
}

void Ekf::stopErPosFusion()
{
	if (_control_status.flags.er_pos) {
		resetEstimatorAidStatus(_aid_src_er_pos);

		_control_status.flags.er_pos = false;
	}
}
