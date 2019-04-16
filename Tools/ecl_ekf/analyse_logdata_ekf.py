#! /usr/bin/env python3
"""
the ecl ekf analysis
"""

from typing import Tuple, List, Dict

import numpy as np
from pyulog import ULog

from analysis.detectors import InAirDetector, PreconditionError
from analysis.metrics import calculate_ecl_ekf_metrics
from analysis.checks import perform_ecl_ekf_checks
from analysis.post_processing import get_estimator_check_flags

def analyse_ekf(
        ulog: ULog, check_levels: Dict[str, float], red_thresh: float = 1.0,
        amb_thresh: float = 0.5, min_flight_duration_seconds: float = 5.0,
        in_air_margin_seconds: float = 5.0, pos_checks_when_sensors_not_fused: bool = False) -> \
        Tuple[str, Dict[str, str], Dict[str, float], Dict[str, float]]:
    """
    :param ulog:
    :param check_levels:
    :param red_thresh:
    :param amb_thresh:
    :param min_flight_duration_seconds:
    :param in_air_margin_seconds:
    :param pos_checks_when_sensors_not_fused:
    :return:
    """

    try:
        estimator_status = ulog.get_dataset('estimator_status').data
        print('found estimator_status data')
    except:
        raise PreconditionError('could not find estimator_status data')

    try:
        _ = ulog.get_dataset('ekf2_innovations').data
        print('found ekf2_innovation data')
    except:
        raise PreconditionError('could not find ekf2_innovation data')

    try:
        in_air = InAirDetector(
            ulog, min_flight_time_seconds=min_flight_duration_seconds, in_air_margin_seconds=0.0)
        in_air_no_ground_effects = InAirDetector(
            ulog, min_flight_time_seconds=min_flight_duration_seconds,
            in_air_margin_seconds=in_air_margin_seconds)
    except Exception as e:
        raise PreconditionError(str(e))

    if in_air_no_ground_effects.take_off is None:
        raise PreconditionError('no airtime detected.')

    airtime_info = {
        'in_air_transition_time': round(in_air.take_off + in_air.log_start, 2),
        'on_ground_transition_time': round(in_air.landing + in_air.log_start, 2)}

    control_mode, innov_flags, gps_fail_flags = get_estimator_check_flags(estimator_status)

    sensor_checks, innov_fail_checks = find_checks_that_apply(
        control_mode, estimator_status,
        pos_checks_when_sensors_not_fused=pos_checks_when_sensors_not_fused)

    metrics = calculate_ecl_ekf_metrics(
        ulog, innov_flags, innov_fail_checks, sensor_checks, in_air, in_air_no_ground_effects,
        red_thresh=red_thresh, amb_thresh=amb_thresh)

    check_status, master_status = perform_ecl_ekf_checks(
        metrics, sensor_checks, innov_fail_checks, check_levels)

    return master_status, check_status, metrics, airtime_info


def find_checks_that_apply(
    control_mode: dict, estimator_status: dict, pos_checks_when_sensors_not_fused: bool = False) ->\
        Tuple[List[str], List[str]]:
    """
    finds the checks that apply and stores them in lists for the std checks and the innovation
    fail checks.
    :param control_mode:
    :param estimator_status:
    :param b_pos_only_when_sensors_fused:
    :return: a tuple of two lists that contain strings for the std checks and for the innovation
    fail checks.
    """
    sensor_checks = list()
    innov_fail_checks = list()

    # Do some automated analysis of the status data
    # normal index range is defined by the flight duration
    start_index = np.amin(np.where(status_time > in_air_transition_time))
    end_index = np.amax(np.where(status_time <= on_ground_transition_time))
    num_valid_values = (end_index - start_index + 1)
    # find a late/early index range from 5 sec after in_air_transtion_time to 5 sec before on-ground transition time for mag and optical flow checks to avoid false positives
    # this can be used to prevent false positives for sensors adversely affected by close proximity to the ground
    # don't do this if the log starts or finishes in air or if it is shut off by flag
    late_start_index = np.amin(np.where(status_time > (in_air_transition_time + 5.0)))\
        if (late_start_early_ending and not b_starts_in_air) else start_index
    early_end_index = np.amax(np.where(status_time <= (on_ground_transition_time - 5.0))) \
        if (late_start_early_ending and not b_finishes_in_air) else end_index
    num_valid_values_trimmed = (early_end_index - late_start_index + 1)
    # also find the start and finish indexes for the innovation data
    innov_start_index = np.amin(np.where(innov_time > in_air_transition_time))
    innov_end_index = np.amax(np.where(innov_time <= on_ground_transition_time))
    innov_num_valid_values = (innov_end_index - innov_start_index + 1)
    innov_late_start_index = np.amin(np.where(innov_time > (in_air_transition_time + 5.0))) \
        if (late_start_early_ending and not b_starts_in_air) else innov_start_index
    innov_early_end_index = np.amax(np.where(innov_time <= (on_ground_transition_time - 5.0))) \
        if (late_start_early_ending and not b_finishes_in_air) else innov_end_index
    innov_num_valid_values_trimmed = (innov_early_end_index - innov_late_start_index + 1)
    # define dictionary of test results and descriptions
    test_results = {
        'master_status': ['Pass',
                          'Master check status which can be either Pass Warning or Fail. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'mag_sensor_status': ['Pass',
                              'Magnetometer sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'yaw_sensor_status': ['Pass',
                              'Yaw sensor check summary. This sensor data can be sourced from the magnetometer or an external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'vel_sensor_status': ['Pass',
                              'Velocity sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'pos_sensor_status': ['Pass',
                              'Position sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'hgt_sensor_status': ['Pass',
                              'Height sensor check summary. This sensor data can be sourced from either Baro or GPS or range finder or external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no anomalies were detected and no further investigation is required'],
        'hagl_sensor_status': ['Pass',
                               'Height above ground sensor check summary. This sensor data is normally sourced from a rangefinder sensor. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'tas_sensor_status': ['Pass',
                              'Airspeed sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_sensor_status': ['Pass',
                              'IMU sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_vibration_check': ['Pass',
                              'IMU vibration check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_bias_check': ['Pass',
                              'IMU bias check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_output_predictor_check': ['Pass',
                              'IMU output predictor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'flow_sensor_status': ['Pass',
                               'Optical Flow sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'filter_fault_status': ['Pass',
                               'Internal Filter check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'mag_percentage_red': [float('NaN'),
                               'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 1.0.'],
        'mag_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 0.5.'],
        'magx_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the X-axis magnetic field sensor innovation consistency test.'],
        'magy_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the Y-axis magnetic field sensor innovation consistency test.'],
        'magz_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the Z-axis magnetic field sensor innovation consistency test.'],
        'yaw_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the yaw sensor innovation consistency test.'],
        'mag_test_max': [float('NaN'),
                         'The maximum in-flight value of the magnetic field sensor innovation consistency test ratio.'],
        'mag_test_mean': [float('NaN'),
                          'The mean in-flight value of the magnetic field sensor innovation consistency test ratio.'],
        'vel_percentage_red': [float('NaN'),
                               'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 1.0.'],
        'vel_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 0.5.'],
        'vel_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
        'vel_test_max': [float('NaN'),
                         'The maximum in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
        'vel_test_mean': [float('NaN'),
                          'The mean in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
        'pos_percentage_red': [float('NaN'),
                               'The percentage of in-flight position sensor consolidated innovation consistency test values > 1.0.'],
        'pos_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight position sensor consolidated innovation consistency test values > 0.5.'],
        'pos_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
        'pos_test_max': [float('NaN'),
                         'The maximum in-flight value of the position sensor consolidated innovation consistency test ratio.'],
        'pos_test_mean': [float('NaN'),
                          'The mean in-flight value of the position sensor consolidated innovation consistency test ratio.'],
        'hgt_percentage_red': [float('NaN'),
                               'The percentage of in-flight height sensor innovation consistency test values > 1.0.'],
        'hgt_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight height sensor innovation consistency test values > 0.5.'],
        'hgt_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the height sensor innovation consistency test.'],
        'hgt_test_max': [float('NaN'),
                         'The maximum in-flight value of the height sensor innovation consistency test ratio.'],
        'hgt_test_mean': [float('NaN'),
                          'The mean in-flight value of the height sensor innovation consistency test ratio.'],
        'tas_percentage_red': [float('NaN'),
                               'The percentage of in-flight airspeed sensor innovation consistency test values > 1.0.'],
        'tas_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight airspeed sensor innovation consistency test values > 0.5.'],
        'tas_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the airspeed sensor innovation consistency test.'],
        'tas_test_max': [float('NaN'),
                         'The maximum in-flight value of the airspeed sensor innovation consistency test ratio.'],
        'tas_test_mean': [float('NaN'),
                          'The mean in-flight value of the airspeed sensor innovation consistency test ratio.'],
        'hagl_percentage_red': [float('NaN'),
                                'The percentage of in-flight height above ground sensor innovation consistency test values > 1.0.'],
        'hagl_percentage_amber': [float('NaN'),
                                  'The percentage of in-flight height above ground sensor innovation consistency test values > 0.5.'],
        'hagl_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the height above ground sensor innovation consistency test.'],
        'hagl_test_max': [float('NaN'),
                          'The maximum in-flight value of the height above ground sensor innovation consistency test ratio.'],
        'hagl_test_mean': [float('NaN'),
                           'The mean in-flight value of the height above ground sensor innovation consistency test ratio.'],
        'ofx_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the optical flow sensor X-axis innovation consistency test.'],
        'ofy_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the optical flow sensor Y-axis innovation consistency test.'],
        'filter_faults_max': [float('NaN'),
                              'Largest recorded value of the filter internal fault bitmask. Should always be zero.'],
        'imu_coning_peak': [float('NaN'), 'Peak in-flight value of the IMU delta angle coning vibration metric (rad)'],
        'imu_coning_mean': [float('NaN'), 'Mean in-flight value of the IMU delta angle coning vibration metric (rad)'],
        'imu_hfdang_peak': [float('NaN'),
                            'Peak in-flight value of the IMU delta angle high frequency vibration metric (rad)'],
        'imu_hfdang_mean': [float('NaN'),
                            'Mean in-flight value of the IMU delta angle high frequency vibration metric (rad)'],
        'imu_hfdvel_peak': [float('NaN'),
                            'Peak in-flight value of the IMU delta velocity high frequency vibration metric (m/s)'],
        'imu_hfdvel_mean': [float('NaN'),
                            'Mean in-flight value of the IMU delta velocity high frequency vibration metric (m/s)'],
        'output_obs_ang_err_median': [float('NaN'),
                                      'Median in-flight value of the output observer angular error (rad)'],
        'output_obs_vel_err_median': [float('NaN'),
                                      'Median in-flight value of the output observer velocity error (m/s)'],
        'output_obs_pos_err_median': [float('NaN'), 'Median in-flight value of the output observer position error (m)'],
        'imu_dang_bias_median': [float('NaN'), 'Median in-flight value of the delta angle bias vector length (rad)'],
        'imu_dvel_bias_median': [float('NaN'), 'Median in-flight value of the delta velocity bias vector length (m/s)'],
        'tilt_align_time': [float('NaN'),
                            'The time in seconds measured from startup that the EKF completed the tilt alignment. A nan value indicates that the alignment had completed before logging started or alignment did not complete.'],
        'yaw_align_time': [float('NaN'),
                           'The time in seconds measured from startup that the EKF completed the yaw alignment.'],
        'in_air_transition_time': [round(in_air_transition_time, 1),
                                   'The time in seconds measured from startup that the EKF transtioned into in-air mode. Set to a nan if a transition event is not detected.'],
        'on_ground_transition_time': [round(on_ground_transition_time, 1),
                                      'The time in seconds measured from startup  that the EKF transitioned out of in-air mode. Set to a nan if a transition event is not detected.'],
    }
    # generate test metadata
    # reduction of innovation message data
    if (innov_early_end_index > (innov_late_start_index + 50)):
        # Output Observer Tracking Errors
        test_results['output_obs_ang_err_median'][0] = np.median(
            ekf2_innovations['output_tracking_error[0]'][innov_late_start_index:innov_early_end_index + 1])
        test_results['output_obs_vel_err_median'][0] = np.median(
            ekf2_innovations['output_tracking_error[1]'][innov_late_start_index:innov_early_end_index + 1])
        test_results['output_obs_pos_err_median'][0] = np.median(
            ekf2_innovations['output_tracking_error[2]'][innov_late_start_index:innov_early_end_index + 1])
    # reduction of status message data
    if (early_end_index > (late_start_index + 50)):
        # IMU vibration checks
        temp = np.amax(estimator_status['vibe[0]'][late_start_index:early_end_index])
        if (temp > 0.0):
            test_results['imu_coning_peak'][0] = temp
            test_results['imu_coning_mean'][0] = np.mean(estimator_status['vibe[0]'][late_start_index:early_end_index + 1])
        temp = np.amax(estimator_status['vibe[1]'][late_start_index:early_end_index])
        if (temp > 0.0):
            test_results['imu_hfdang_peak'][0] = temp
            test_results['imu_hfdang_mean'][0] = np.mean(estimator_status['vibe[1]'][late_start_index:early_end_index + 1])
        temp = np.amax(estimator_status['vibe[2]'][late_start_index:early_end_index])
        if (temp > 0.0):
            test_results['imu_hfdvel_peak'][0] = temp
            test_results['imu_hfdvel_mean'][0] = np.mean(estimator_status['vibe[2]'][late_start_index:early_end_index + 1])

    # Magnetometer Sensor Checks
    if (np.amax(control_mode['yaw_aligned']) > 0.5):
        sensor_checks.append('mag')

        innov_fail_checks.append('magx')
        innov_fail_checks.append('magy')
        innov_fail_checks.append('magz')
        innov_fail_checks.append('yaw')

    # Velocity Sensor Checks
    if (np.amax(control_mode['using_gps']) > 0.5):
        sensor_checks.append('vel')
        innov_fail_checks.append('vel')

    # Position Sensor Checks
    if (pos_checks_when_sensors_not_fused or (np.amax(control_mode['using_gps']) > 0.5)
        or (np.amax(control_mode['using_evpos']) > 0.5)):
        sensor_checks.append('pos')
        innov_fail_checks.append('posh')

    # Airspeed Sensor Checks
    # a value > 1.0 means the measurement data for that test has been rejected by the EKF
    if (np.amax(estimator_status['tas_test_ratio']) > 0.0):
        sensor_checks.append('tas')
        innov_fail_checks.append('tas')

    # Height above ground (rangefinder) sensor checks
    if (np.amax(estimator_status['hagl_test_ratio']) > 0.0):
        sensor_checks.append('hagl')
        innov_fail_checks.append('hagl')

    # optical flow sensor checks
    if (np.amax(control_mode['using_optflow']) > 0.5):
        innov_fail_checks.append('ofx')
        innov_fail_checks.append('ofy')

    return sensor_checks, innov_fail_checks







