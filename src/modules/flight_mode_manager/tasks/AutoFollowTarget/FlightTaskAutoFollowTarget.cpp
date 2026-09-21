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
 * @file FlightTaskAutoFollowTarget.cpp
 *
 * Flight Task for follow-me flight mode. It consumes follow_target_estimator messages from
 * TargetEstimator. The drone then tracks this target's GPS coordinates from a specified
 * angle and distance.
 *
 */

#include "FlightTaskAutoFollowTarget.hpp"
#include <mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <float.h>

using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Quatf;
using matrix::Eulerf;

// Call the constructor of _sticks to establish Follow Target class as a parent as ModuleParams class
// which enables chained paramter update sequence, which keeps the parameters in _sticks up to date.
FlightTaskAutoFollowTarget::FlightTaskAutoFollowTarget() : _sticks(this)
{
	// _target_estimator.Start();
}

FlightTaskAutoFollowTarget::~FlightTaskAutoFollowTarget()
{
	releaseGimbalControl();
	// _target_estimator.Stop();
}

bool FlightTaskAutoFollowTarget::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	if (!_position_setpoint.isAllFinite()) {
		_position_setpoint = _position;
	}

	_lock_position_setpoint = _position;
	PX4_INFO("[AutoFollow::activate] Init lock pos setpoint: %.2f, %.2f, %.2f", (double)_lock_position_setpoint(0),
			(double)_lock_position_setpoint(1), (double)_lock_position_setpoint(2));
	_target_position_velocity_filter.reset(Vector3f{NAN, NAN, NAN});
	_target_position_velocity_filter.setParameters(TARGET_POS_VEL_FILTER_NATURAL_FREQUENCY,
			TARGET_POS_VEL_FILTER_DAMPING_RATIO);


	_yaw_setpoint = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;
	_drone_to_target_heading = _yaw_setpoint;
	_yawspeed_setpoint = NAN;

	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;

	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f pos_prev{last_setpoint.position};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current position
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }
	}

	_position_smoothing.reset(accel_prev, vel_prev, pos_prev);

	_updateTrajConstraints();

	// Ensure planned cruise speed is below the maximum such that the smooth trajectory doesn't get capped
	_max_xy_speed = math::constrain(_param_tgt_vel_max.get(), 0.f, _param_mpc_xy_vel_max.get());
	_target_acceptance_radius = 0.5f;

	// Update the internally tracked Follow Target characteristics
	_follow_angle_rad = matrix::wrap_pi(math::radians(_param_flw_tgt_fa.get()));
	_follow_distance = _param_flw_tgt_dst.get();
	_follow_height = _param_flw_tgt_ht.get();
	_follow_railway_height = _follow_height;
	_follow_drone_moving_en = _param_flw_mv_en.get();
	_prev_follow_drone_moving_en = _follow_drone_moving_en;

	// Reset the orbit angle trajectory generator and set maximum angular acceleration and rate
	_orbit_angle_traj_generator.reset(0.f, 0.f, 0.f);

	// Save the home position z value to enable relative altitude setpoints to arming position (home) altitude
	if (_sub_home_position.get().valid_alt) {
		_home_position_z = _sub_home_position.get().z;

	} else {
		ret = false; // Don't activate Follow Target task if home position is not valid
	}

	return ret;
}

void FlightTaskAutoFollowTarget::_updateTrajConstraints()
{
	// update params of the position smoothing
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setCruiseSpeed(_max_xy_speed);
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setTargetAcceptanceRadius(_target_acceptance_radius);

	// Update the constraints of the trajectories
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
	_position_smoothing.setMaxVelocityXY(_max_xy_speed);
	_position_smoothing.setMaxJerk(_param_mpc_jerk_auto.get());

	_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
	_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
}

void FlightTaskAutoFollowTarget::updateParams()
{
	// Copy previous param values to check if it changes after param update
	const float follow_distance_prev = _param_flw_tgt_dst.get();
	const float follow_height_prev = _param_flw_tgt_ht.get();
	const float follow_angle_prev = _param_flw_tgt_fa.get();

	// Call updateParams in parent class to update parameters from child up until ModuleParam base class
	FlightTask::updateParams();

	// Compare param values and if they have changed, update the properties
	if (!matrix::isEqualF(follow_distance_prev, _param_flw_tgt_dst.get())) {
		_follow_distance = _param_flw_tgt_dst.get();
	}

	if (!matrix::isEqualF(follow_height_prev, _param_flw_tgt_ht.get())) {
		_follow_height = _param_flw_tgt_ht.get();
		_follow_railway_height = _follow_height;
	}

	if (!matrix::isEqualF(follow_angle_prev, _param_flw_tgt_fa.get())) {
		_follow_angle_rad = matrix::wrap_pi(math::radians(_param_flw_tgt_fa.get()));
	}

	_follow_drone_moving_en = _param_flw_mv_en.get();

}

void FlightTaskAutoFollowTarget::updateTargetPositionVelocityFilter(const follow_target_estimator_s
		&follow_target_estimator)
{
	const Vector3f pos_ned_est{follow_target_estimator.pos_est};
	const Vector3f vel_ned_est{follow_target_estimator.vel_est};

	const bool target_estimator_timed_out = ((follow_target_estimator.timestamp - _last_valid_target_estimator_timestamp) >
						TARGET_ESTIMATOR_TIMEOUT_US);
	_last_valid_target_estimator_timestamp = follow_target_estimator.timestamp;

	if (!_target_position_velocity_filter.getState().isAllFinite()
	    || !_target_position_velocity_filter.getRate().isAllFinite() || target_estimator_timed_out) {
		_target_position_velocity_filter.reset(pos_ned_est, vel_ned_est);

	} else {
		_target_position_velocity_filter.update(_deltatime, pos_ned_est, vel_ned_est);
	}
}

void FlightTaskAutoFollowTarget::updateRcAdjustedFollowHeight(const Sticks &sticks)
{
	// Only apply Follow height adjustment if height setpoint and current height are within time window
	if (fabsf(_position_setpoint(2) - _position(2)) < FOLLOW_HEIGHT_USER_ADJUST_SPEED * USER_ADJUSTMENT_ERROR_TIME_WINDOW) {
		// RC Throttle stick input for changing follow height
		const float height_change_speed = FOLLOW_HEIGHT_USER_ADJUST_SPEED * sticks.getThrottleZeroCenteredExpo();
		const float new_height = _follow_railway_height + height_change_speed * _deltatime;
		_follow_railway_height = constrain(new_height, MINIMUM_SAFETY_ALTITUDE, FOLLOW_HEIGHT_MAX);
	}
}
void FlightTaskAutoFollowTarget::updateRcAdjustedFollowDistance(const Sticks &sticks,
		const Vector2f &drone_to_target_vector)
{
	// Only apply Follow distance adjustment if distance setting and current distance are within time window
	if (fabsf(drone_to_target_vector.length() - _follow_distance) < FOLLOW_DISTANCE_USER_ADJUST_SPEED *
	    USER_ADJUSTMENT_ERROR_TIME_WINDOW) {
		// RC Pitch stick input for changing distance
		const float distance_change_speed = FOLLOW_DISTANCE_USER_ADJUST_SPEED * sticks.getPitchExpo();
		const float new_distance = _follow_distance + distance_change_speed * _deltatime;
		_follow_distance = constrain(new_distance, MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL, FOLLOW_DISTANCE_MAX);
	}
}

void FlightTaskAutoFollowTarget::updateRcAdjustedFollowAngle(const Sticks &sticks, const float measured_orbit_angle,
		const float tracked_orbit_angle_setpoint)
{
	// Only apply Follow Angle adjustment if orbit angle setpoint and current orbit angle are within time window
	// Wrap orbit angle difference, to get the shortest angle between them
	if (fabsf(matrix::wrap_pi(measured_orbit_angle - tracked_orbit_angle_setpoint)) < FOLLOW_ANGLE_USER_ADJUST_SPEED *
	    USER_ADJUSTMENT_ERROR_TIME_WINDOW) {
		// RC Roll stick input for changing follow angle. When user commands RC stick input: +Roll (right), angle increases (clockwise)
		// Constrain adjust speed [rad/s] so that drone can actually catch up. Otherwise, the follow angle
		// command can be too ahead that drone's behavior would be un-responsive to RC stick inputs.
		const float angle_adjust_speed_max = min(FOLLOW_ANGLE_USER_ADJUST_SPEED,
						     _param_flw_tgt_max_vel.get() / _follow_distance);
		const float angle_change_speed = angle_adjust_speed_max * sticks.getRollExpo();
		const float new_angle = _follow_angle_rad + angle_change_speed * _deltatime;
		_follow_angle_rad = matrix::wrap_pi(new_angle);
	}
}

float FlightTaskAutoFollowTarget::updateTargetOrientation(const float current_target_orientation,
		const Vector2f &target_velocity, const Vector2f &target_velocity_unfiltered) const
{
	// Only update the target orientation if both filtered & unfiltered target velocity are above the deadzone velocity.
	// This prevents unintuitive follow position setpoints derived from overshooting filtered target velocity, and thus target course
	// when the target abruptly stops.
	if ((target_velocity.norm() >= TARGET_SPEED_DEADZONE_FOR_ORIENTATION_TRACKING)
	    && (target_velocity_unfiltered.norm() >= TARGET_SPEED_DEADZONE_FOR_ORIENTATION_TRACKING)) {
		return atan2f(target_velocity(1), target_velocity(0));
	}

	return current_target_orientation;
}

float FlightTaskAutoFollowTarget::updateOrbitAngleTrajectory(const float target_orientation,
		const float previous_orbit_angle_setpoint)
{
	// Raw target orbit (setpoint) angle, unwrapped to be relative to the previous orbit angle setpoint
	const float unwrapped_raw_orbit_angle = matrix::unwrap_pi(previous_orbit_angle_setpoint,
						target_orientation + _follow_angle_rad);

	// Calculate limits for orbit angular acceleration and velocity rate
	_orbit_angle_traj_generator.setMaxJerk(ORBIT_TRAJECTORY_MAX_JERK / _follow_distance);
	_orbit_angle_traj_generator.setMaxAccel(ORBIT_TRAJECTORY_MAX_ACCELERATION / _follow_distance);
	_orbit_angle_traj_generator.setMaxVel(_param_flw_tgt_max_vel.get() / _follow_distance);

	// Calculate the maximum angular rate setpoint based on remaining angle to raw target
	const float remaining_angle = unwrapped_raw_orbit_angle - previous_orbit_angle_setpoint;
	const float remaining_angle_sign = matrix::sign(remaining_angle);
	const float max_rate = math::trajectory::computeMaxSpeedFromDistance(_orbit_angle_traj_generator.getMaxJerk(),
			       _orbit_angle_traj_generator.getMaxAccel(), fabsf(remaining_angle), 0.f);

	// Set angular rate setpoint, considering the sign direction
	_orbit_angle_traj_generator.updateDurations(max_rate * remaining_angle_sign);

	// Calculate trajectory towards the unwrapped target orbit angle
	_orbit_angle_traj_generator.updateTraj(_deltatime);

	const float unwrapped_orbit_angle_setpoint = _orbit_angle_traj_generator.getCurrentPosition();

	return unwrapped_orbit_angle_setpoint;
}

Vector2f FlightTaskAutoFollowTarget::getOrbitTangentialVelocity(const float orbit_angle_setpoint) const
{
	const float angular_rate_setpoint = _orbit_angle_traj_generator.getCurrentVelocity();
	return Vector2f(-sinf(orbit_angle_setpoint), cosf(orbit_angle_setpoint)) * angular_rate_setpoint * _follow_distance;
}

Vector3f FlightTaskAutoFollowTarget::calculateDesiredDronePosition(const Vector3f &target_position,
		const float orbit_angle_setpoint) const
{
	Vector3f drone_desired_position{NAN, NAN, NAN};

	// Offset from the Target
	const Vector2f offset_vector = Vector2f(cosf(orbit_angle_setpoint), sinf(orbit_angle_setpoint)) * _follow_distance;

	drone_desired_position.xy() = Vector2f(target_position.xy()) + offset_vector;

	// Calculate ground's z value in local frame for terrain tracking mode.
	// _dist_to_ground value takes care of distance sensor value if available, otherwise
	// it uses home z value as reference
	const float ground_z_estimate = _position(2) + _dist_to_ground;

	// Z-position based off curent and initial target altitude
	switch ((kFollowAltitudeMode)_param_flw_tgt_alt_m.get()) {
	case kFollowAltitudeModeTerrain:
		drone_desired_position(2) = ground_z_estimate - _follow_railway_height;
		break;

	case kFollowAltitudeModeTrackTarget:
		drone_desired_position(2) = target_position(2) - _follow_railway_height;
		break;

	case kFollowAltitudeModeConstant:

	// FALLTHROUGH

	default:
		// Calculate the desired Z position relative to the home position
		drone_desired_position(2) = _home_position_z - _follow_railway_height;
	}

	return drone_desired_position;
}


float FlightTaskAutoFollowTarget::calculateGimbalHeight(const kFollowAltitudeMode altitude_mode,
		const float target_pos_z) const
{
	float gimbal_height{0.0f};

	switch (altitude_mode) {
	case kFollowAltitudeModeTerrain:
		// Point the gimbal at the ground level in this tracking mode
		gimbal_height = _dist_to_ground;
		break;

	case kFollowAltitudeModeTrackTarget:
		// Point the gimbal at the target's 3D coordinates
		gimbal_height = -(_position(2) - target_pos_z);
		break;

	case kFollowAltitudeModeConstant:

	//FALLTHROUGH

	default:
		gimbal_height = _home_position_z - _position(2); // Assume target is at home position's altitude
		break;
	}

	return gimbal_height;
}


bool FlightTaskAutoFollowTarget::update()
{
	follow_target_status_s follow_target_status{};
	bool in_emergency_ascent{false};
	float gimbal_pitch{NAN};
	// float raw_orbit_angle_setpoint{NAN};

	// Get the latest target estimator message for target position and velocity
	_follow_target_estimator_sub.update(&_follow_target_estimator);

	if (_follow_target_estimator.timestamp > 0 && _follow_target_estimator.valid) {

		// Automatic inspection Railway
		const bool auto_inspection_railway  = (_follow_target_estimator.cls == 9);
		if (auto_inspection_railway) {
			_follow_railway_height = _follow_height * 1.5f;
		} else {
			_follow_railway_height = _follow_height;
		}

		_boundbox_center_x  = _follow_target_estimator.acc_est[0];
		_boundbox_center_y  = _follow_target_estimator.acc_est[1];

		updateTargetPositionVelocityFilter(_follow_target_estimator);
		const Vector3f target_position_filtered = _target_position_velocity_filter.getState();
		// const Vector3f target_velocity_filtered = _target_position_velocity_filter.getRate();
		const Vector2f drone_to_target_vector  = Vector2f(target_position_filtered.xy()) - Vector2f(_position.xy());

		// Calculate heading to the target (for Yaw setpoint)
		const float drone_to_target_heading = atan2f(drone_to_target_vector(1), drone_to_target_vector(0));
		// Actual orbit angle measured around the target, which is pointing from target to drone, so M_PI_F difference.
		// const float measured_orbit_angle = matrix::wrap_pi(drone_to_target_heading + M_PI_F);

		if (!_target_follow_init_) {
			if (drone_to_target_vector.length() > 1.f) {
				_drone_to_target_heading = matrix::wrap_pi(drone_to_target_heading + M_PI_F);
			} else {
				_drone_to_target_heading = matrix::wrap_pi(_yaw + M_PI_F);
			}
			_target_follow_init_ = true;
		}
		// Update the sticks object to fetch recent data and update follow distance, angle and height via RC commands
		// _sticks.checkAndUpdateStickInputs();

		// if (_sticks.isAvailable()) {
		// 	updateRcAdjustedFollowHeight(_sticks);
		// 	updateRcAdjustedFollowDistance(_sticks, drone_to_target_vector);
		// 	updateRcAdjustedFollowAngle(_sticks, measured_orbit_angle, _orbit_angle_setpoint_rad);
		// }

		// _target_course_rad = updateTargetOrientation(_target_course_rad, target_velocity_filtered.xy(),
		// 		     Vector3f(_follow_target_estimator.vel_est).xy());

		// [Debug] Save raw idealistic orbit angle setpoint for debug message
		// raw_orbit_angle_setpoint = matrix::unwrap_pi(_orbit_angle_setpoint_rad, _target_course_rad + _follow_angle_rad);

		// // Update the new orbit angle (rate constrained)
		// _orbit_angle_setpoint_rad = updateOrbitAngleTrajectory(_target_course_rad, _orbit_angle_setpoint_rad);
		_orbit_angle_setpoint_rad = _drone_to_target_heading;

		// Orbital tangential velocity setpoint from the generated trajectory
		// const Vector2f orbit_tangential_velocity = getOrbitTangentialVelocity(_orbit_angle_setpoint_rad);

		// Calculate desired position by applying orbit angle around the target
		const Vector3f drone_desired_position = calculateDesiredDronePosition(target_position_filtered,
							_orbit_angle_setpoint_rad);

		// NOTE : Currently if Follow Target is activated when the drone is very far away from the target, since trajectory setpoint's velocity & acceleration
		// components are directly applied in the Multicopter-Position-Controller, it can command orbiting velocity / acceleration setpoints even when the
		// drone hasn't reached the target orbit angle setpoint yet. Which will interfere with the drone coming straight towards the orbit angle setpoint.
		// This has to be dealed with in the Multicopter-Rate-Controller, where it should not command acceleration feed forward (setpoint) directly if there's a big pos/vel error.

		// Calculate orbit acceleration
		// const Vector2f orbit_radial_accel = (orbit_tangential_velocity.norm_squared() / _follow_distance) * Vector2f(-cosf(
		// 		_orbit_angle_setpoint_rad), -sinf(_orbit_angle_setpoint_rad));
		// const Vector2f orbit_tangential_accel = _orbit_angle_traj_generator.getCurrentAcceleration() * _follow_distance *
		// 					Vector2f(-sinf(_orbit_angle_setpoint_rad), cosf(_orbit_angle_setpoint_rad));
		// const Vector2f orbit_total_accel = orbit_radial_accel + orbit_tangential_accel;

		// Position + Velocity + Acceleration setpoint
		// if (drone_desired_position.isAllFinite()) {
		// 	if (fabsf(drone_desired_position(2) - _position(2)) < ALT_ACCEPTANCE_THRESHOLD) {
		// 		// Drone is close enough to the altitude target : Apply Horizontal + Velocity Control
		// 		_drone_desired_position_setpoint = drone_desired_position;
		// 	} else {
		// 		// Drone hasn't achieved desired altitude yet : Only apply Vertical Control
		// 		_drone_desired_position_setpoint = _position;
		// 		_drone_desired_position_setpoint(2) = drone_desired_position(2);
		// 	}

		// }
		_drone_desired_position_setpoint = drone_desired_position;
		// else {
		// 	// Desired position is not finite : Don't apply any control
		// 	_position_setpoint = _position;
		// 	_velocity_setpoint.setZero();
		// 	_acceleration_setpoint.setNaN();
		// }


		float gimbal_yaw_rate_setpoint  = (_boundbox_center_x - _camera_center_x) * GIMBAL_ERR_KP;
		float gimbal_pitch_rate_setpoint  = (_boundbox_center_y - _camera_center_y) * GIMBAL_ERR_KP;

		if (auto_inspection_railway) {
			// Yaw setpoint
			if (drone_to_target_vector.longerThan(MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL)) {
				_yaw_setpoint = drone_to_target_heading;

			}
		} else {
			_yawspeed_setpoint = gimbal_yaw_rate_setpoint;
			_yaw_setpoint = NAN;
		}




		// Set Gimbal pitch to track target in the center of the view
		const float gimbal_height = calculateGimbalHeight((kFollowAltitudeMode)_param_flw_tgt_alt_m.get(),
					    target_position_filtered(2));

		if (auto_inspection_railway) {
			gimbal_pitch = 0.f;
		} else {
			gimbal_pitch = pointGimbalAt(drone_to_target_vector.norm(), gimbal_height, gimbal_yaw_rate_setpoint, gimbal_pitch_rate_setpoint);

		}

		if (_drone_desired_position_setpoint.isAllFinite()) {
			if (_follow_drone_moving_en  >= 1) {
				Vector3f waypoints[] = {_position, _drone_desired_position_setpoint, _drone_desired_position_setpoint};

				const bool force_zero_velocity_setpoint = false;
				_updateTrajConstraints();
				PositionSmoothing::PositionSmoothingSetpoints smoothed_setpoints;
				_position_smoothing.generateSetpoints(
					_position,
					waypoints,
					{0.f, 0.f, 0.f},
					_deltatime,
					force_zero_velocity_setpoint,
					smoothed_setpoints
				);

				_jerk_setpoint = smoothed_setpoints.jerk;
				_acceleration_setpoint = smoothed_setpoints.acceleration;
				_velocity_setpoint = smoothed_setpoints.velocity;
				_position_setpoint = smoothed_setpoints.position;
			} else {
				_position_setpoint = _lock_position_setpoint;
				_velocity_setpoint.setZero();
				_acceleration_setpoint.setNaN();
			}


			// Emergency ascent when too close to the ground
			in_emergency_ascent = PX4_ISFINITE(_dist_to_ground) && _dist_to_ground < MINIMUM_SAFETY_ALTITUDE;

			if (in_emergency_ascent) {
				_position_setpoint(0) = _position_setpoint(1) = NAN;
				_position_setpoint(2) = _position(2);
				_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
				_velocity_setpoint(2) = -EMERGENCY_ASCENT_SPEED; // Slowly ascend
			}
		} else {
			// Target estimate is invalid : Stay in current position
			_position_setpoint.setAll(NAN);
			_velocity_setpoint.setAll(0.0f);
		}

	} else {
		// Target estimate is invalid : Stay in current position
		_position_setpoint.setAll(NAN);
		_velocity_setpoint.setAll(0.0f);
	}

	// if (_follow_drone_moving_en < 1) {
	// 	_position_setpoint = _lock_position_setpoint;
	// 	_velocity_setpoint.setZero();
	// 	_acceleration_setpoint.setNaN();
	// }
	if (_prev_follow_drone_moving_en != _follow_drone_moving_en) {
		_lock_position_setpoint = _position;
		PX4_INFO("[AutoFollow] change lock pos setpoint: %.2f, %.2f, %.2f", (double)_lock_position_setpoint(0),
			(double)_lock_position_setpoint(1), (double)_lock_position_setpoint(2));
	}

	_prev_follow_drone_moving_en = _follow_drone_moving_en;



	// Follow Target Status message for debugging
	follow_target_status.timestamp = hrt_absolute_time();

	// Log angle values to check target course estimate & orbit angle trajectory control
	follow_target_status.tracked_target_course = _target_course_rad;
	follow_target_status.follow_angle = _follow_angle_rad;
	follow_target_status.orbit_angle_setpoint = _orbit_angle_setpoint_rad;

	// Log max & actual orbit angular rates to check orbit anglular rate trajectory control
	follow_target_status.angular_rate_setpoint = _orbit_angle_traj_generator.getCurrentVelocity();

	// Log desired Raw Follow position to easily check ideal vs actual follow me performance
	// const Vector3f drone_desired_position_raw = calculateDesiredDronePosition(_target_position_velocity_filter.getState(),
	// 		raw_orbit_angle_setpoint);
	// drone_desired_position_raw.copyTo(follow_target_status.desired_position_raw);
	_drone_desired_position_setpoint.copyTo(follow_target_status.desired_position_raw);
	follow_target_status.in_emergency_ascent = in_emergency_ascent;
	follow_target_status.gimbal_pitch = gimbal_pitch;
	_follow_target_status_pub.publish(follow_target_status);

	_constraints.want_takeoff = _checkTakeoff();

	return true;
}

void FlightTaskAutoFollowTarget::releaseGimbalControl()
{
	// NOTE: If other flight tasks start using gimbal control as well
	// it might be worth moving this release mechanism to a common base
	// class for gimbal-control flight tasks

	vehicle_command_s vehicle_command = {};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vehicle_command.param1 = -3.0f; // Remove control if it had it.
	vehicle_command.param2 = -3.0f; // Remove control if it had it.
	vehicle_command.param3 = -1.0f; // Leave unchanged.
	vehicle_command.param4 = -1.0f; // Leave unchanged.

	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.source_system = _param_mav_sys_id.get();
	vehicle_command.source_component = _param_mav_comp_id.get();
	vehicle_command.target_system = _param_mav_sys_id.get();
	vehicle_command.target_component = _param_mav_comp_id.get();
	vehicle_command.confirmation = false;
	vehicle_command.from_external = false;

	_vehicle_command_pub.publish(vehicle_command);
}

float FlightTaskAutoFollowTarget::pointGimbalAt(const float xy_distance, const float z_distance,
		const float gimbal_yaw_rate_setpoint, const float gimbal_pitch_rate_setpoint)
{
	gimbal_manager_set_attitude_s msg{};
	float pitch_down_angle = 0.0f;

	if (PX4_ISFINITE(z_distance)) {
		pitch_down_angle = atan2f(z_distance, xy_distance);
	}

	if (!PX4_ISFINITE(pitch_down_angle)) {
		pitch_down_angle = 0.0;
	}

	msg.angular_velocity_x = 0.f;
	msg.angular_velocity_y = -gimbal_pitch_rate_setpoint;
	msg.angular_velocity_z = gimbal_yaw_rate_setpoint;

	const Quatf q_gimbal = Quatf(Eulerf(0, -pitch_down_angle, 0));
	q_gimbal.copyTo(msg.q);

	msg.origin_sysid = 1;
	msg.origin_compid = 1;

	msg.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(msg);



	return pitch_down_angle;
}
