/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_imu_plugin.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {
namespace {
static constexpr double kDegToRad = 0.017453292519943295;
static constexpr double kTwoPi = 6.2831853071795864769;

Eigen::Vector3d ToEigen(const ignition::math::Vector3d& value) {
  return Eigen::Vector3d(value.X(), value.Y(), value.Z());
}
}

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      velocity_prev_W_(0,0,0)
{
}

GazeboImuPlugin::~GazeboImuPlugin() {
  updateConnection_->~Connection();
}


void GazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
  getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                      imu_parameters_.gyroscope_noise_density,
                      imu_parameters_.gyroscope_noise_density);
  getSdfParam<double>(_sdf, "gyroscopeRandomWalk",
                      imu_parameters_.gyroscope_random_walk,
                      imu_parameters_.gyroscope_random_walk);
  getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                      imu_parameters_.gyroscope_bias_correlation_time,
                      imu_parameters_.gyroscope_bias_correlation_time);
  assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                      imu_parameters_.gyroscope_turn_on_bias_sigma,
                      imu_parameters_.gyroscope_turn_on_bias_sigma);
  getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                      imu_parameters_.accelerometer_noise_density,
                      imu_parameters_.accelerometer_noise_density);
  getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                      imu_parameters_.accelerometer_random_walk,
                      imu_parameters_.accelerometer_random_walk);
  getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                      imu_parameters_.accelerometer_bias_correlation_time,
                      imu_parameters_.accelerometer_bias_correlation_time);
  assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                      imu_parameters_.accelerometer_turn_on_bias_sigma,
                      imu_parameters_.accelerometer_turn_on_bias_sigma);
  getSdfParam<bool>(_sdf, "enableEngineImuVibration",
                    enable_engine_imu_vibration_,
                    enable_engine_imu_vibration_);
  getSdfParam<bool>(_sdf, "engineImuVibrationGateByMotorCommand",
                    engine_imu_vibration_gate_by_motor_command_,
                    engine_imu_vibration_gate_by_motor_command_);
  getSdfParam<std::string>(_sdf, "engineImuVibrationMotorCommandTopic",
                           engine_imu_vibration_motor_command_topic_,
                           engine_imu_vibration_motor_command_topic_);
  getSdfParam<double>(_sdf, "engineImuVibrationMotorSpeedScale",
                      engine_imu_vibration_motor_speed_scale_,
                      engine_imu_vibration_motor_speed_scale_);
  getSdfParam<double>(_sdf, "engineImuVibrationMotorThreshold",
                      engine_imu_vibration_motor_threshold_,
                      engine_imu_vibration_motor_threshold_);
  getSdfParam<double>(_sdf, "engineImuVibrationCommandTimeoutSec",
                      engine_imu_vibration_command_timeout_sec_,
                      engine_imu_vibration_command_timeout_sec_);
  getSdfParam<double>(_sdf, "engineImuVibrationStartSec",
                      engine_imu_vibration_start_sec_,
                      engine_imu_vibration_start_sec_);
  getSdfParam<double>(_sdf, "engineImuVibrationRampSec",
                      engine_imu_vibration_ramp_sec_,
                      engine_imu_vibration_ramp_sec_);
  getSdfParam<double>(_sdf, "engineImuVibrationEnvelopeAmplitude",
                      engine_imu_vibration_envelope_amplitude_,
                      engine_imu_vibration_envelope_amplitude_);
  getSdfParam<double>(_sdf, "engineImuVibrationEnvelopePeriodSec",
                      engine_imu_vibration_envelope_period_sec_,
                      engine_imu_vibration_envelope_period_sec_);
  getSdfParam<double>(_sdf, "engineImuVibrationEnvelopePhaseDeg",
                      engine_imu_vibration_envelope_phase_deg_,
                      engine_imu_vibration_envelope_phase_deg_);
  getSdfParam<double>(_sdf, "engineImuVibrationFreq1Hz",
                      engine_imu_vibration_freq1_hz_,
                      engine_imu_vibration_freq1_hz_);
  getSdfParam<double>(_sdf, "engineImuVibrationFreq2Hz",
                      engine_imu_vibration_freq2_hz_,
                      engine_imu_vibration_freq2_hz_);
  getSdfParam<double>(_sdf, "engineImuVibrationFreq3Hz",
                      engine_imu_vibration_freq3_hz_,
                      engine_imu_vibration_freq3_hz_);
  getSdfParam<double>(_sdf, "engineImuVibrationFreq4Hz",
                      engine_imu_vibration_freq4_hz_,
                      engine_imu_vibration_freq4_hz_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelAmp1Mps2",
                                        engine_imu_vibration_accel_amp1_mps2_,
                                        engine_imu_vibration_accel_amp1_mps2_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelAmp2Mps2",
                                        engine_imu_vibration_accel_amp2_mps2_,
                                        engine_imu_vibration_accel_amp2_mps2_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelAmp3Mps2",
                                        engine_imu_vibration_accel_amp3_mps2_,
                                        engine_imu_vibration_accel_amp3_mps2_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelAmp4Mps2",
                                        engine_imu_vibration_accel_amp4_mps2_,
                                        engine_imu_vibration_accel_amp4_mps2_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroAmp1Radps",
                                        engine_imu_vibration_gyro_amp1_radps_,
                                        engine_imu_vibration_gyro_amp1_radps_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroAmp2Radps",
                                        engine_imu_vibration_gyro_amp2_radps_,
                                        engine_imu_vibration_gyro_amp2_radps_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroAmp3Radps",
                                        engine_imu_vibration_gyro_amp3_radps_,
                                        engine_imu_vibration_gyro_amp3_radps_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroAmp4Radps",
                                        engine_imu_vibration_gyro_amp4_radps_,
                                        engine_imu_vibration_gyro_amp4_radps_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelPhase1Deg",
                                        engine_imu_vibration_accel_phase1_deg_,
                                        engine_imu_vibration_accel_phase1_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelPhase2Deg",
                                        engine_imu_vibration_accel_phase2_deg_,
                                        engine_imu_vibration_accel_phase2_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelPhase3Deg",
                                        engine_imu_vibration_accel_phase3_deg_,
                                        engine_imu_vibration_accel_phase3_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationAccelPhase4Deg",
                                        engine_imu_vibration_accel_phase4_deg_,
                                        engine_imu_vibration_accel_phase4_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroPhase1Deg",
                                        engine_imu_vibration_gyro_phase1_deg_,
                                        engine_imu_vibration_gyro_phase1_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroPhase2Deg",
                                        engine_imu_vibration_gyro_phase2_deg_,
                                        engine_imu_vibration_gyro_phase2_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroPhase3Deg",
                                        engine_imu_vibration_gyro_phase3_deg_,
                                        engine_imu_vibration_gyro_phase3_deg_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "engineImuVibrationGyroPhase4Deg",
                                        engine_imu_vibration_gyro_phase4_deg_,
                                        engine_imu_vibration_gyro_phase4_deg_);

  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>("~/" + model_->GetName() + imu_topic_, 10);

  if (enable_engine_imu_vibration_ && engine_imu_vibration_gate_by_motor_command_) {
    motor_command_sub_ =
        node_handle_->Subscribe(engine_imu_vibration_motor_command_topic_,
                                &GazeboImuPlugin::OnMotorCommand, this);
  }

  // Fill imu message.
  // imu_message_.header.frame_id = frame_id_; TODO Add header
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  for(int i=0; i< 9; i++){
    switch (i){
    case 0:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    case 1:
    case 2:
    case 3:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    case 4:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    case 5:
    case 6:
    case 7:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    case 8:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    }
  }

  gravity_W_ = world_->Gravity();
  imu_parameters_.gravity_magnitude = gravity_W_.Length();

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
      gyroscope_bias_[i] =
          sigma_bon_g * standard_normal_distribution_(random_generator_);
      accelerometer_bias_[i] =
          sigma_bon_a * standard_normal_distribution_(random_generator_);
  }


}

void GazeboImuPlugin::OnMotorCommand(CommandMotorSpeedPtr& motor_command) {
  if (!motor_command || motor_command->motor_speed_size() < 2) {
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 9
  const common::Time now = world_->SimTime();
#else
  const common::Time now = world_->GetSimTime();
#endif

  const double scale = std::max(1e-6, engine_imu_vibration_motor_speed_scale_);
  const double engine_a = std::fabs(motor_command->motor_speed(0)) / scale;
  const double engine_b = std::fabs(motor_command->motor_speed(1)) / scale;
  const bool motor_active =
      std::max(engine_a, engine_b) >= engine_imu_vibration_motor_threshold_;

  if (motor_active && !engine_imu_vibration_motor_active_) {
    engine_imu_vibration_active_since_sec_ = now.Double();
  }

  if (!motor_active) {
    engine_imu_vibration_active_since_sec_ = -1.0;
  }

  engine_imu_vibration_motor_active_ = motor_active;
  has_engine_imu_vibration_command_ = true;
  last_engine_imu_vibration_command_time_ = now;
}

double GazeboImuPlugin::EngineVibrationScale(const double time_sec) {
  if (!enable_engine_imu_vibration_) {
    return 0.0;
  }

  double elapsed = time_sec;

  if (engine_imu_vibration_gate_by_motor_command_) {
    const bool command_recent =
        has_engine_imu_vibration_command_ &&
        (time_sec - last_engine_imu_vibration_command_time_.Double())
            <= engine_imu_vibration_command_timeout_sec_;

    if (!command_recent || !engine_imu_vibration_motor_active_ ||
        engine_imu_vibration_active_since_sec_ < 0.0) {
      return 0.0;
    }

    elapsed = time_sec - engine_imu_vibration_active_since_sec_;

  } else if (time_sec < engine_imu_vibration_start_sec_) {
    return 0.0;
  }

  if (elapsed < engine_imu_vibration_start_sec_) {
    return 0.0;
  }

  double ramp_scale = 1.0;

  if (engine_imu_vibration_ramp_sec_ > 1e-6) {
    const double ramp_fraction =
        (elapsed - engine_imu_vibration_start_sec_) / engine_imu_vibration_ramp_sec_;

    ramp_scale = std::max(0.0, std::min(1.0, ramp_fraction));
  }

  double envelope_scale = 1.0;

  if (engine_imu_vibration_envelope_amplitude_ > 1e-6 &&
      engine_imu_vibration_envelope_period_sec_ > 1e-6) {
    const double envelope_time =
        std::max(0.0, elapsed - engine_imu_vibration_start_sec_);
    envelope_scale = 1.0 + engine_imu_vibration_envelope_amplitude_ *
        std::sin(kTwoPi * envelope_time /
                 engine_imu_vibration_envelope_period_sec_ +
                 engine_imu_vibration_envelope_phase_deg_ * kDegToRad);
    envelope_scale = std::max(0.0, envelope_scale);
  }

  return ramp_scale * envelope_scale;
}

Eigen::Vector3d GazeboImuPlugin::HarmonicTerm(
    const double frequency_hz,
    const ignition::math::Vector3d& amplitude,
    const ignition::math::Vector3d& phase_deg,
    const double time_sec) {
  if (frequency_hz <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d amplitude_eigen = ToEigen(amplitude);
  const Eigen::Vector3d phase_rad = ToEigen(phase_deg) * kDegToRad;
  Eigen::Vector3d signal = Eigen::Vector3d::Zero();
  const double base_phase = kTwoPi * frequency_hz * time_sec;

  for (int axis = 0; axis < 3; ++axis) {
    signal[axis] = amplitude_eigen[axis] * std::sin(base_phase + phase_rad[axis]);
  }

  return signal;
}

void GazeboImuPlugin::addEngineVibration(
    Eigen::Vector3d* linear_acceleration,
    Eigen::Vector3d* angular_velocity,
    const double time_sec) {
  const double scale = EngineVibrationScale(time_sec);

  if (scale <= 0.0) {
    return;
  }

  *linear_acceleration += scale * (
      HarmonicTerm(engine_imu_vibration_freq1_hz_,
                   engine_imu_vibration_accel_amp1_mps2_,
                   engine_imu_vibration_accel_phase1_deg_,
                   time_sec) +
      HarmonicTerm(engine_imu_vibration_freq2_hz_,
                   engine_imu_vibration_accel_amp2_mps2_,
                   engine_imu_vibration_accel_phase2_deg_,
                   time_sec) +
      HarmonicTerm(engine_imu_vibration_freq3_hz_,
                   engine_imu_vibration_accel_amp3_mps2_,
                   engine_imu_vibration_accel_phase3_deg_,
                   time_sec) +
      HarmonicTerm(engine_imu_vibration_freq4_hz_,
                   engine_imu_vibration_accel_amp4_mps2_,
                   engine_imu_vibration_accel_phase4_deg_,
                   time_sec));

  *angular_velocity += scale * (
      HarmonicTerm(engine_imu_vibration_freq1_hz_,
                   engine_imu_vibration_gyro_amp1_radps_,
                   engine_imu_vibration_gyro_phase1_deg_,
                   time_sec) +
      HarmonicTerm(engine_imu_vibration_freq2_hz_,
                   engine_imu_vibration_gyro_amp2_radps_,
                   engine_imu_vibration_gyro_phase2_deg_,
                   time_sec) +
      HarmonicTerm(engine_imu_vibration_freq3_hz_,
                   engine_imu_vibration_gyro_amp3_radps_,
                   engine_imu_vibration_gyro_phase3_deg_,
                   time_sec) +
      HarmonicTerm(engine_imu_vibration_freq4_hz_,
                   engine_imu_vibration_gyro_amp4_radps_,
                   engine_imu_vibration_gyro_phase4_deg_,
                   time_sec));
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void GazeboImuPlugin::addNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  // CHECK(linear_acceleration);
  // CHECK(angular_velocity);
  assert(dt > 0.0);

  // Gyrosocpe
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d =
      sqrt( - sigma_b_g * sigma_b_g * tau_g / 2.0 *
      (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] = (*angular_velocity)[i] +
        gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_);
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d =
      sqrt( - sigma_b_a * sigma_b_a * tau_a / 2.0 *
      (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] = (*linear_acceleration)[i] +
        accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_);
  }

}

// This gets called by the world update start event.
void GazeboImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time  = world_->SimTime();
#else
  common::Time current_time  = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose(); //TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif

  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  // Copy ignition::math::Quaterniond to gazebo::msgs::Quaternion
  gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  orientation->set_x(C_W_I.X());
  orientation->set_y(C_W_I.Y());
  orientation->set_z(C_W_I.Z());
  orientation->set_w(C_W_I.W());

#if GAZEBO_MAJOR_VERSION < 5
  ignition::math::Vector3d velocity_current_W = link_->GetWorldLinearVel();
  // link_->RelativeLinearAccel() does not work sometimes with old gazebo versions.
  // TODO For an accurate simulation, this might have to be fixed. Consider the
  // This issue is solved in gazebo 5.
  ignition::math::Vector3d acceleration = (velocity_current_W - velocity_prev_W_) / dt;
  ignition::math::Vector3d acceleration_I =
      C_W_I.RotateVectorReverse(acceleration - gravity_W_);

  velocity_prev_W_ = velocity_current_W;
#elif GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d acceleration_I = link_->RelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);
#else
  ignition::math::Vector3d acceleration_I = ignitionFromGazeboMath(link_->GetRelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_));
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d angular_vel_I = link_->RelativeAngularVel();
#else
  ignition::math::Vector3d angular_vel_I = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
#endif

  Eigen::Vector3d linear_acceleration_I(acceleration_I.X(),
                                        acceleration_I.Y(),
                                        acceleration_I.Z());
  Eigen::Vector3d angular_velocity_I(angular_vel_I.X(),
                                     angular_vel_I.Y(),
                                     angular_vel_I.Z());

  addEngineVibration(&linear_acceleration_I, &angular_velocity_I, t);
  addNoise(&linear_acceleration_I, &angular_velocity_I, dt);

  // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
  gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
  linear_acceleration->set_x(linear_acceleration_I[0]);
  linear_acceleration->set_y(linear_acceleration_I[1]);
  linear_acceleration->set_z(linear_acceleration_I[2]);

  // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
  gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
  angular_velocity->set_x(angular_velocity_I[0]);
  angular_velocity->set_y(angular_velocity_I[1]);
  angular_velocity->set_z(angular_velocity_I[2]);

  // Fill IMU message.
  // ADD HEaders
  // imu_message_.header.stamp.sec = current_time.sec;
  // imu_message_.header.stamp.nsec = current_time.nsec;
  imu_message_.set_time_usec(_info.simTime.sec * 1000000 + _info.simTime.nsec / 1000);
  imu_message_.set_seq(seq_++);

  // TODO(burrimi): Add orientation estimator.
  // imu_message_.orientation.w = 1;
  // imu_message_.orientation.x = 0;
  // imu_message_.orientation.y = 0;
  // imu_message_.orientation.z = 0;

  imu_message_.set_allocated_orientation(orientation);
  imu_message_.set_allocated_linear_acceleration(linear_acceleration);
  imu_message_.set_allocated_angular_velocity(angular_velocity);

  imu_pub_->Publish(imu_message_);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}
