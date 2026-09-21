/****************************************************************************
 *
 *   Copyright (c) 2026.
 *
 ****************************************************************************/

#include "MotionDetector.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;

static inline float clamp_unit(float x)
{
	return math::constrain(x, -1.f, 1.f);
}

MotionDetector::MotionDetector() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_updated();
	reset_buffer();
}

MotionDetector::~MotionDetector()
{
	perf_free(_loop_perf);
}

bool MotionDetector::init()
{

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MotionDetector::parameters_updated()
{

	const float win = math::constrain(_param_md_win.get(), 0.1f, 5.0f);
	_param_md_win.set(win);

	const float hz = math::constrain(_param_md_rate_hz.get(), 10.f, 400.f);
	_param_md_rate_hz.set(hz);


	const float a_on = _param_md_a_thr_on.get();
	const float a_off = _param_md_a_thr_off.get();
	if (a_on <= a_off) {
		_param_md_a_thr_on.set(a_off + 0.1f);
	}

	const float g_on = _param_md_g_thr_on.get();
	const float g_off = _param_md_g_thr_off.get();
	if (g_on <= g_off) {
		_param_md_g_thr_on.set(g_off + 0.05f);
	}

	const float ton = math::constrain(_param_md_ton.get(), 0.02f, 2.0f);
	const float toff = math::constrain(_param_md_toff.get(), 0.05f, 3.0f);
	_param_md_ton.set(ton);
	_param_md_toff.set(toff);
}

void MotionDetector::reset_buffer()
{
	_head = 0;
	_tail = 0;
	_count = 0;
	_sum_a2 = 0.f;
	_sum_g2 = 0.f;
}

void MotionDetector::push_sample(const Sample &s)
{

	if (_count >= MAX_SAMPLES) {
		// pop one
		const Sample &old = _buf[_tail];
		_sum_a2 -= old.a2;
		_sum_g2 -= old.g2;

		_tail = (_tail + 1) % MAX_SAMPLES;
		_count--;
	}

	_buf[_head] = s;
	_sum_a2 += s.a2;
	_sum_g2 += s.g2;

	_head = (_head + 1) % MAX_SAMPLES;
	_count++;
}

void MotionDetector::pop_old(hrt_abstime now)
{
	const uint64_t win_us = (uint64_t)(_param_md_win.get() * 1e6f);

	while (_count > 0) {
		const Sample &old = _buf[_tail];

		if (old.ts + win_us >= now) {
			break;
		}

		_sum_a2 -= old.a2;
		_sum_g2 -= old.g2;

		_tail = (_tail + 1) % MAX_SAMPLES;
		_count--;
	}
}

bool MotionDetector::compute_features(float &accel_rms, float &gyro_rms, float &angle_delta, float &angle_rate, float &win_dt) const
{
	if (_count < 2) {
		return false;
	}

	const int newest_index = (_head - 1 + MAX_SAMPLES) % MAX_SAMPLES;
	const Sample &s_old = _buf[_tail];
	const Sample &s_new = _buf[newest_index];

	const float dt = (float)((s_new.ts - s_old.ts) * 1e-6);

	if (dt < 1e-3f) {
		return false;
	}

	win_dt = dt;

	// RMS
	accel_rms = sqrtf(math::max(_sum_a2 / (float)_count, 0.f));
	gyro_rms  = sqrtf(math::max(_sum_g2 / (float)_count, 0.f));

	// angle delta from attitude quaternion: dq = q_old^{-1} * q_new
	const Quatf q_old = s_old.q;
	const Quatf q_new = s_new.q;

	const Quatf dq = q_old.inversed() * q_new;

	// dq.w = cos(theta/2), theta = 2*acos(w)
	const float w = clamp_unit(dq(0)); // Quatf order: (w,x,y,z)
	angle_delta = 2.f * acosf(w);

	angle_rate = angle_delta / dt;

	return true;
}

void MotionDetector::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// parameter update
	if (_parameter_update_sub.updated()) {
		parameter_update_s p{};
		_parameter_update_sub.copy(&p);

		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s gyro{};
	if (!_vehicle_angular_velocity_sub.update(&gyro)) {
		perf_end(_loop_perf);
		return;
	}

	const hrt_abstime now = gyro.timestamp_sample;


	const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.0005f, 0.05f);
	_last_run = now;


	if (!_param_md_en.get()) {
		_state = MotionState::UNKNOWN;

		vehicle_motion_s out{};
		out.timestamp_sample = now;
		out.timestamp = hrt_absolute_time();
		out.moving = false;
		out.state = (uint8_t)_state;
		out.accel_rms = NAN;
		out.gyro_rms = NAN;
		out.angle_delta = NAN;
		out.angle_rate = NAN;
		out.motion_score = NAN;
		_vehicle_motion_pub.publish(out);

		perf_end(_loop_perf);
		return;
	}


	const float min_period_s = 1.f / math::max(_param_md_rate_hz.get(), 1.f);
	const uint64_t min_period_us = (uint64_t)(min_period_s * 1e6f);

	if (_last_process != 0 && (now - _last_process) < min_period_us) {
		perf_end(_loop_perf);
		return;
	}
	_last_process = now;


	if (_vehicle_acceleration_sub.updated()) {
		_have_accel = _vehicle_acceleration_sub.copy(&_accel);
	}

	if (_vehicle_attitude_sub.updated()) {
		_have_attitude = _vehicle_attitude_sub.copy(&_attitude);
	}

	if (!_have_accel || !_have_attitude) {
		_state = MotionState::UNKNOWN;

		vehicle_motion_s out{};
		out.timestamp_sample = now;
		out.timestamp = hrt_absolute_time();
		out.moving = false;
		out.state = (uint8_t)_state;
		out.accel_rms = NAN;
		out.gyro_rms = NAN;
		out.angle_delta = NAN;
		out.angle_rate = NAN;
		out.motion_score = NAN;
		_vehicle_motion_pub.publish(out);

		perf_end(_loop_perf);
		return;
	}


	const Vector3f a_meas{_accel.xyz};      // m/s^2, includes gravity (body FRD)
	const Vector3f w_meas{gyro.xyz};        // rad/s

	const Quatf q{_attitude.q};             // body FRD -> NED
	const Dcmf R{q};                        // body -> NED


	const Vector3f g_ned{0.f, 0.f, CONSTANTS_ONE_G};
	const Vector3f g_body = R.transpose() * g_ned;


	const Vector3f a_lin = a_meas + g_body;


	Sample s{};
	s.ts = now;
	s.a2 = a_lin.dot(a_lin);               // |a_lin|^2
	s.g2 = w_meas.dot(w_meas);             // |gyro|^2
	s.q = q;

	push_sample(s);
	pop_old(now);


	float accel_rms = NAN;
	float gyro_rms = NAN;
	float angle_delta = NAN;
	float angle_rate = NAN;
	float win_dt = NAN;

	if (!compute_features(accel_rms, gyro_rms, angle_delta, angle_rate, win_dt)) {
		_state = MotionState::UNKNOWN;

		vehicle_motion_s out{};
		out.timestamp_sample = now;
		out.timestamp = hrt_absolute_time();
		out.moving = false;
		out.state = (uint8_t)_state;
		out.accel_rms = accel_rms;
		out.gyro_rms = gyro_rms;
		out.angle_delta = angle_delta;
		out.angle_rate = angle_rate;
		out.motion_score = NAN;
		_vehicle_motion_pub.publish(out);

		perf_end(_loop_perf);
		return;
	}

	// === decision with hysteresis + time gate ===
	const float A_ON  = _param_md_a_thr_on.get();
	const float A_OFF = _param_md_a_thr_off.get();
	const float G_ON  = _param_md_g_thr_on.get();
	const float G_OFF = _param_md_g_thr_off.get();
	const float ANG_ON = _param_md_ang_thr_on.get();

	const float T_ON  = _param_md_ton.get();
	const float T_OFF = _param_md_toff.get();

	_prev_state = _state;

	if (_state != MotionState::MOVING) {
		const bool above = (accel_rms > A_ON) || (gyro_rms > G_ON) || (angle_rate > ANG_ON);

		if (above) {
			_above_on_time += dt;
			if (_above_on_time >= T_ON) {
				_state = MotionState::MOVING;
				_below_off_time = 0.f;
			}
		} else {
			_above_on_time = 0.f;

			if (_count >= 5) {
				_state = MotionState::STATIONARY;
			}
		}

	} else {
		const bool below = (accel_rms < A_OFF) && (gyro_rms < G_OFF);

		if (below) {
			_below_off_time += dt;
			if (_below_off_time >= T_OFF) {
				_state = MotionState::STATIONARY;
				_above_on_time = 0.f;
			}
		} else {
			_below_off_time = 0.f;
		}
	}

	vehicle_motion_s out{};
	out.timestamp_sample = now;
	out.timestamp = hrt_absolute_time();
	out.state = (uint8_t)_state;
	out.moving = (_state == MotionState::MOVING);

	out.accel_rms = accel_rms;
	out.gyro_rms = gyro_rms;
	out.angle_delta = angle_delta;
	out.angle_rate = angle_rate;


	const float sA = (A_ON > 1e-3f) ? (accel_rms / A_ON) : 0.f;
	const float sG = (G_ON > 1e-3f) ? (gyro_rms / G_ON) : 0.f;
	const float sR = (ANG_ON > 1e-3f) ? (angle_rate / ANG_ON) : 0.f;
	out.motion_score = 0.5f * sA + 0.35f * sG + 0.15f * sR;

	_vehicle_motion_pub.publish(out);


	if (_state != _prev_state) {
		if (_state == MotionState::MOVING) {
			PX4_INFO("motion: MOVING (a_rms=%.3f, g_rms=%.3f, ang_rate=%.3f)", (double)accel_rms, (double)gyro_rms, (double)angle_rate);
		} else if (_state == MotionState::STATIONARY) {
			PX4_INFO("motion: STATIONARY (a_rms=%.3f, g_rms=%.3f)", (double)accel_rms, (double)gyro_rms);
		} else {
			PX4_INFO("motion: UNKNOWN");
		}
	}

	perf_end(_loop_perf);
}

int MotionDetector::task_spawn(int argc, char *argv[])
{
	MotionDetector *instance = new MotionDetector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MotionDetector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MotionDetector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Detects whether the vehicle is moving based on:
- vehicle_acceleration (body FRD, includes gravity)
- vehicle_angular_velocity (body FRD)
- vehicle_attitude (quaternion body->NED)

It performs gravity compensation to get linear acceleration, computes windowed RMS features and attitude angle delta,
then applies hysteresis + time gating to output a stable moving/stationary state.

Publishes: vehicle_motion (custom uORB message).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motion_detector", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int motion_detector_main(int argc, char *argv[])
{
	return MotionDetector::main(argc, argv);
}
