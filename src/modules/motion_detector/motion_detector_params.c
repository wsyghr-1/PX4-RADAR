/****************************************************************************
 *
 * Motion detector parameters
 *
 ****************************************************************************/

/**
 * Enable motion_detector
 *
 * Enable/disable the motion detector module.
 *
 * @boolean
 * @group motion detector
 */
PARAM_DEFINE_INT32(MD_EN, 0);

/**
 * Motion detector window length
 *
 * Sliding window length used to compute RMS/angle delta features.
 *
 * Unit: s
 * Min: 0.10
 * Max: 5.00
 * @decimal 2
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_WIN, 0.60f);

/**
 * Motion detector processing rate
 *
 * Internal processing update rate limit. The module will not process faster than this rate
 * even if gyro callback fires more frequently.
 *
 * Unit: Hz
 * Min: 10
 * Max: 400
 * @decimal 0
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_RATE_HZ, 100.0f);

/**
 * Linear acceleration RMS threshold (ON)
 *
 * If the RMS of gravity-compensated linear acceleration exceeds this threshold continuously
 * for MD_TON seconds, motion state switches to MOVING.
 *
 * Unit: m/s^2
 * Min: 0.0
 * Max: 50.0
 * @decimal 2
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_A_THR_ON, 1.50f);

/**
 * Linear acceleration RMS threshold (OFF)
 *
 * If the RMS of gravity-compensated linear acceleration stays below this threshold AND
 * gyro RMS stays below MD_G_THR_OFF continuously for MD_TOFF seconds, motion state switches
 * to STATIONARY.
 *
 * Note: Must be smaller than MD_A_THR_ON to provide hysteresis.
 *
 * Unit: m/s^2
 * Min: 0.0
 * Max: 50.0
 * @decimal 2
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_A_THR_OFF, 0.80f);

/**
 * Angular velocity RMS threshold (ON)
 *
 * If the RMS of gyro angular rate exceeds this threshold continuously for MD_TON seconds,
 * motion state switches to MOVING.
 *
 * Unit: rad/s
 * Min: 0.0
 * Max: 20.0
 * @decimal 3
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_G_THR_ON, 0.40f);

/**
 * Angular velocity RMS threshold (OFF)
 *
 * If the RMS of gyro angular rate stays below this threshold AND accel RMS stays below
 * MD_A_THR_OFF continuously for MD_TOFF seconds, motion state switches to STATIONARY.
 *
 * Note: Must be smaller than MD_G_THR_ON to provide hysteresis.
 *
 * Unit: rad/s
 * Min: 0.0
 * Max: 20.0
 * @decimal 3
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_G_THR_OFF, 0.25f);

/**
 * Attitude change rate threshold (ON)
 *
 * If the attitude change rate (angle delta over window length) exceeds this threshold
 * continuously for MD_TON seconds, motion state switches to MOVING.
 *
 * Unit: rad/s
 * Min: 0.0
 * Max: 10.0
 * @decimal 3
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_ANG_THR_ON, 0.35f);

/**
 * Moving confirmation time
 *
 * Time the ON-condition must be continuously met before switching to MOVING.
 *
 * Unit: s
 * Min: 0.02
 * Max: 2.00
 * @decimal 2
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_TON, 0.08f);

/**
 * Stationary confirmation time
 *
 * Time the OFF-condition must be continuously met before switching to STATIONARY.
 *
 * Unit: s
 * Min: 0.05
 * Max: 3.00
 * @decimal 2
 * @group motion detector
 */
PARAM_DEFINE_FLOAT(MD_TOFF, 0.20f);
