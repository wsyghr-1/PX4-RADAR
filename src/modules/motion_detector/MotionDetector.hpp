/****************************************************************************
 *
 *   Copyright (c) 2026.
 *
 ****************************************************************************/

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>


#include <uORB/topics/vehicle_motion.h>

using namespace time_literals;

static constexpr float CONSTANTS_ONE_G = 9.80665f;

class MotionDetector : public ModuleBase<MotionDetector>, public ModuleParams, public px4::WorkItem
{
public:
	explicit MotionDetector();
	~MotionDetector() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	void parameters_updated();

	enum class MotionState : uint8_t {
		UNKNOWN = 0,
		STATIONARY = 1,
		MOVING = 2
	};

	// ring buffer for windowed RMS + angle delta
	static constexpr int MAX_SAMPLES = 400; // 例如：100Hz * 4s，留足余量（实际由 MD_WIN 控制裁剪）

	struct Sample {
		hrt_abstime ts{0};
		float a2{0.f};                 // |a_lin|^2
		float g2{0.f};                 // |gyro|^2
		matrix::Quatf q{};             // attitude quaternion (w,x,y,z), body->NED
	};

	void reset_buffer();
	void push_sample(const Sample &s);
	void pop_old(hrt_abstime now);
	bool compute_features(float &accel_rms, float &gyro_rms, float &angle_delta, float &angle_rate, float &win_dt) const;

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publication
	uORB::Publication<vehicle_motion_s> _vehicle_motion_pub{ORB_ID(vehicle_motion)};

	// cached latest input
	vehicle_acceleration_s _accel{};
	vehicle_attitude_s _attitude{};
	bool _have_accel{false};
	bool _have_attitude{false};

	// timing
	hrt_abstime _last_run{0};
	hrt_abstime _last_process{0};

	// window buffer
	Sample _buf[MAX_SAMPLES];
	int _head{0};  // points to newest+1 (write position)
	int _tail{0};  // points to oldest (read position)
	int _count{0};

	float _sum_a2{0.f};
	float _sum_g2{0.f};

	// state machine
	MotionState _state{MotionState::UNKNOWN};
	MotionState _prev_state{MotionState::UNKNOWN};
	float _above_on_time{0.f};
	float _below_off_time{0.f};

	perf_counter_t _loop_perf{nullptr};

	// Parameters (需要你在参数系统里定义这些 MD_* 参数)
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::MD_EN>)        _param_md_en,

		(ParamFloat<px4::params::MD_WIN>)      _param_md_win,        // s
		(ParamFloat<px4::params::MD_RATE_HZ>)  _param_md_rate_hz,    // Hz

		(ParamFloat<px4::params::MD_A_THR_ON>)  _param_md_a_thr_on,   // m/s^2
		(ParamFloat<px4::params::MD_A_THR_OFF>) _param_md_a_thr_off,  // m/s^2

		(ParamFloat<px4::params::MD_G_THR_ON>)  _param_md_g_thr_on,   // rad/s
		(ParamFloat<px4::params::MD_G_THR_OFF>) _param_md_g_thr_off,  // rad/s

		(ParamFloat<px4::params::MD_ANG_THR_ON>) _param_md_ang_thr_on, // rad/s

		(ParamFloat<px4::params::MD_TON>)      _param_md_ton,        // s
		(ParamFloat<px4::params::MD_TOFF>)     _param_md_toff        // s
	)
};
