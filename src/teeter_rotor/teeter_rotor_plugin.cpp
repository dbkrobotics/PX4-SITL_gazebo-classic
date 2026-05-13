#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <algorithm>
#include <cmath>
#include <string>

namespace gazebo
{
class TeeterRotorPlugin : public ModelPlugin
{
public:
  TeeterRotorPlugin() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    std::string base_link_name = "base_link";
    std::string rotor_link_name = "rotor_link";
    std::string blade1_indicator_link_name = "blade_1_pitch_indicator";
    std::string blade2_indicator_link_name = "blade_2_pitch_indicator";
    std::string joint_name = "main_rotor_joint";

    this->ReadString(sdf, "baseLinkName", base_link_name);
    this->ReadString(sdf, "rotorLinkName", rotor_link_name);
    this->ReadString(sdf, "blade1IndicatorLinkName", blade1_indicator_link_name);
    this->ReadString(sdf, "blade2IndicatorLinkName", blade2_indicator_link_name);
    this->ReadString(sdf, "jointName", joint_name);
    this->ReadBool(sdf, "enableCommandTopics", this->enable_command_topics_);
    this->ReadString(sdf, "commandTopicNamespace", this->command_topic_namespace_);

    this->base_link_ = this->model_->GetLink(base_link_name);
    this->rotor_link_ = this->model_->GetLink(rotor_link_name);
    this->blade1_indicator_link_ = this->model_->GetLink(blade1_indicator_link_name);
    this->blade2_indicator_link_ = this->model_->GetLink(blade2_indicator_link_name);
    this->joint_ = this->model_->GetJoint(joint_name);

    if (!this->base_link_) {
      gzerr << "[TeeterRotorPlugin] Base link not found: " << base_link_name << std::endl;
      return;
    }
    if (!this->rotor_link_) {
      gzerr << "[TeeterRotorPlugin] Rotor link not found: " << rotor_link_name << std::endl;
      return;
    }
    if (!this->blade1_indicator_link_) {
      gzerr << "[TeeterRotorPlugin] Blade 1 indicator link not found: " << blade1_indicator_link_name << std::endl;
      return;
    }
    if (!this->blade2_indicator_link_) {
      gzerr << "[TeeterRotorPlugin] Blade 2 indicator link not found: " << blade2_indicator_link_name << std::endl;
      return;
    }
    if (!this->joint_) {
      gzerr << "[TeeterRotorPlugin] Joint not found: " << joint_name << std::endl;
      return;
    }

    // Primary variables.
    this->ReadDouble(sdf, "targetRpm", this->target_rpm_);
    this->ReadDouble(sdf, "payloadWeightLb", this->payload_weight_lb_);
    this->ReadDouble(sdf, "blade1PitchDeg", this->blade1_pitch_deg_);
    this->ReadDouble(sdf, "blade2PitchDeg", this->blade2_pitch_deg_);

    // Azimuth-based cyclic pitch.
    this->ReadBool(sdf, "useCyclicPitch", this->use_cyclic_pitch_);
    this->ReadDouble(sdf, "collectiveDeg", this->collective_deg_);
    this->ReadDouble(sdf, "rollCyclicDeg", this->roll_cyclic_deg_);
    this->ReadDouble(sdf, "pitchCyclicDeg", this->pitch_cyclic_deg_);
    this->ReadDouble(sdf, "cyclicPhaseOffsetDeg", this->cyclic_phase_offset_deg_);

    // Engine dynamics.
    this->ReadBool(sdf, "useEngineDynamics", this->use_engine_dynamics_);
    this->ReadDouble(sdf, "engineThrottle", this->engine_throttle_);
    this->ReadDouble(sdf, "engineRadiusFt", this->engine_radius_ft_);
    this->ReadDouble(sdf, "engineThrustLbEach", this->engine_thrust_lb_each_);
    this->ReadDouble(sdf, "rotorInertiaKgm2", this->rotor_inertia_kgm2_);
    this->ReadDouble(sdf, "kTorque", this->k_torque_);

    // Visual indicators.
    this->ReadBool(sdf, "showPitchIndicators", this->show_pitch_indicators_);
    this->ReadBool(sdf, "animatePitchIndicators", this->animate_pitch_indicators_);
    this->ReadDouble(sdf, "pitchIndicatorAnimationAmplitudeDeg", this->pitch_indicator_animation_amplitude_deg_);
    this->ReadDouble(sdf, "pitchIndicatorAnimationFrequencyHz", this->pitch_indicator_animation_frequency_hz_);
    this->ReadDouble(sdf, "pitchIndicatorVerticalOffsetM", this->pitch_indicator_vertical_offset_m_);

    // Fixed constants.
    this->ReadDouble(sdf, "emptyWeightLb", this->empty_weight_lb_);
    this->ReadDouble(sdf, "kThrust", this->k_thrust_);
    this->ReadDouble(sdf, "maxRpm", this->max_rpm_);

    // Pitch model.
    this->ReadDouble(sdf, "hoverPitchDeg", this->hover_pitch_deg_);
    this->ReadDouble(sdf, "zeroLiftPitchDeg", this->zero_lift_pitch_deg_);
    this->ReadDouble(sdf, "minPitchDeg", this->min_pitch_deg_);
    this->ReadDouble(sdf, "maxPitchDeg", this->max_pitch_deg_);

    // Metadata.
    this->ReadDouble(sdf, "rotorRadiusFt", this->rotor_radius_ft_);
    this->ReadDouble(sdf, "hubRadiusFt", this->hub_radius_ft_);
    this->ReadInt(sdf, "numBlades", this->num_blades_);
    this->ReadDouble(sdf, "bladeChordIn", this->blade_chord_in_);
    this->ReadString(sdf, "airfoil", this->airfoil_);
    this->ReadDouble(sdf, "tipSpeedFps", this->tip_speed_fps_);
    this->ReadDouble(sdf, "designGrossWeightLb", this->design_gross_weight_lb_);
    this->ReadDouble(sdf, "tipWeightLb", this->tip_weight_lb_);
    this->ReadDouble(sdf, "bladeSpanwiseDensityLbFt", this->blade_spanwise_density_lb_ft_);
    this->ReadDouble(sdf, "forwardSpeedMph", this->forward_speed_mph_);
    this->ReadDouble(sdf, "rotorTwistDeg", this->rotor_twist_deg_);
    this->ReadDouble(sdf, "rotorPlaneHeightFt", this->rotor_plane_height_ft_);
    this->ReadDouble(sdf, "teeterAngleLimitDeg", this->teeter_angle_limit_deg_);
    this->ReadDouble(sdf, "preConeAngleDeg", this->pre_cone_angle_deg_);
    this->ReadDouble(sdf, "gyroPrecessionTorqueFtLb", this->gyro_precession_torque_ft_lb_);
    this->ReadDouble(sdf, "torqueRequiredFtLb", this->torque_required_ft_lb_);
    this->ReadDouble(sdf, "equivalentShaftHp", this->equivalent_shaft_hp_);
    this->ReadDouble(sdf, "propulsiveEfficiency", this->propulsive_efficiency_);
    this->ReadInt(sdf, "numEngines", this->num_engines_);
    this->ReadDouble(sdf, "engineHpEach", this->engine_hp_each_);
    this->ReadDouble(sdf, "mainBladeSparOdMm", this->main_blade_spar_od_mm_);
    this->ReadDouble(sdf, "mainBladeSparIdMm", this->main_blade_spar_id_mm_);

    // Unit conversion and derived values.
    this->rotor_radius_m_ = FtToM(this->rotor_radius_ft_);
    this->hub_radius_m_ = FtToM(this->hub_radius_ft_);
    this->blade_chord_m_ = InToM(this->blade_chord_in_);

    this->max_omega_ = RpmToRadPerSec(this->max_rpm_);
    this->target_omega_ = RpmToRadPerSec(this->target_rpm_);
    this->target_omega_ = std::max(0.0, std::min(this->target_omega_, this->max_omega_));
    this->rotor_omega_ = this->target_omega_;

    this->empty_weight_kg_ = LbMassToKg(this->empty_weight_lb_);
    this->payload_mass_kg_ = LbMassToKg(this->payload_weight_lb_);
    this->payload_weight_n_ = LbForceToN(this->payload_weight_lb_);
    this->current_gross_weight_lb_ = this->empty_weight_lb_ + this->payload_weight_lb_;
    this->current_gross_weight_n_ = LbForceToN(this->current_gross_weight_lb_);

    this->engine_radius_m_ = FtToM(this->engine_radius_ft_);
    this->engine_thrust_n_each_ = LbForceToN(this->engine_thrust_lb_each_);
    this->max_drive_torque_nm_ =
      static_cast<double>(this->num_engines_) * this->engine_thrust_n_each_ * this->engine_radius_m_;

    this->blade_effective_span_m_ = this->rotor_radius_m_ - this->hub_radius_m_;
    this->blade_lift_radius_m_ = 0.5 * (this->rotor_radius_m_ + this->hub_radius_m_);
    this->pre_cone_rad_ = DegToRad(this->pre_cone_angle_deg_);
    this->blade_lift_x_m_ = this->hub_radius_m_ + 0.5 * this->blade_effective_span_m_ * std::cos(this->pre_cone_rad_);
    this->blade_lift_z_m_ = 0.5 * this->blade_effective_span_m_ * std::sin(this->pre_cone_rad_);
    this->disk_area_m2_ = M_PI * this->rotor_radius_m_ * this->rotor_radius_m_;
    this->solidity_ = static_cast<double>(this->num_blades_) * this->blade_chord_m_ /
      (M_PI * this->rotor_radius_m_);

    if (this->enable_command_topics_) {
      this->node_.reset(new transport::Node());
      this->node_->Init(this->model_->GetWorld()->Name());

      const std::string cyclic_topic = "~/" + this->command_topic_namespace_ + "/cyclic_cmd";
      const std::string engine_topic = "~/" + this->command_topic_namespace_ + "/engine_cmd";

      this->cyclic_cmd_sub_ =
        this->node_->Subscribe(cyclic_topic, &TeeterRotorPlugin::OnCyclicCmd, this);
      this->engine_cmd_sub_ =
        this->node_->Subscribe(engine_topic, &TeeterRotorPlugin::OnEngineCmd, this);

      gzmsg << "[TeeterRotorPlugin] Runtime command topics enabled:\n"
            << "  /gazebo/" << this->model_->GetWorld()->Name()
            << "/" << this->command_topic_namespace_ << "/cyclic_cmd\n"
            << "  /gazebo/" << this->model_->GetWorld()->Name()
            << "/" << this->command_topic_namespace_ << "/engine_cmd\n"
            << std::endl;
    }

    this->PrintFactSheet();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TeeterRotorPlugin::OnUpdate, this));
  }

private:
  static double FtToM(double ft) { return ft * 0.3048; }
  static double InToM(double in) { return in * 0.0254; }
  static double LbMassToKg(double lbm) { return lbm * 0.45359237; }
  static double LbForceToN(double lbf) { return lbf * 4.4482216152605; }
  static double FtLbToNm(double ftlb) { return ftlb * 1.3558179483314; }
  static double HpToW(double hp) { return hp * 745.699872; }
  static double RpmToRadPerSec(double rpm) { return rpm * 2.0 * M_PI / 60.0; }
  static double RadPerSecToRpm(double omega) { return omega * 60.0 / (2.0 * M_PI); }
  static double DegToRad(double deg) { return deg * M_PI / 180.0; }

  void ReadDouble(sdf::ElementPtr sdf, const std::string &name, double &value)
  {
    if (sdf->HasElement(name)) {
      value = sdf->Get<double>(name);
    }
  }

  void ReadInt(sdf::ElementPtr sdf, const std::string &name, int &value)
  {
    if (sdf->HasElement(name)) {
      value = sdf->Get<int>(name);
    }
  }

  void ReadBool(sdf::ElementPtr sdf, const std::string &name, bool &value)
  {
    if (sdf->HasElement(name)) {
      value = sdf->Get<bool>(name);
    }
  }

  void ReadString(sdf::ElementPtr sdf, const std::string &name, std::string &value)
  {
    if (sdf->HasElement(name)) {
      value = sdf->Get<std::string>(name);
    }
  }

  double PitchFactor(double pitch_deg) const
  {
    const double pitch = std::max(this->min_pitch_deg_, std::min(pitch_deg, this->max_pitch_deg_));
    const double denom = this->hover_pitch_deg_ - this->zero_lift_pitch_deg_;
    if (std::abs(denom) < 1e-6) {
      return 1.0;
    }
    const double factor = (pitch - this->zero_lift_pitch_deg_) / denom;
    return std::max(0.0, std::min(2.0, factor));
  }

  double ClampPitchDeg(double pitch_deg) const
  {
    return std::max(this->min_pitch_deg_, std::min(pitch_deg, this->max_pitch_deg_));
  }

  double BladeLiftNewton(double omega, double pitch_deg) const
  {
    return 0.5 * this->k_thrust_ * omega * omega * this->PitchFactor(pitch_deg);
  }

  double EmptyWeightNewton() const
  {
    return LbForceToN(this->empty_weight_lb_);
  }

  double ClampEngineThrottle(double throttle) const
  {
    return std::max(0.0, std::min(throttle, 1.0));
  }

  double DriveTorqueNewtonMeter() const
  {
    return this->ClampEngineThrottle(this->engine_throttle_) * this->max_drive_torque_nm_;
  }

  double DragTorqueNewtonMeter(double omega) const
  {
    return this->k_torque_ * omega * omega;
  }

  double StepRotorOmega(const common::Time &now)
  {
    if (!this->use_engine_dynamics_) {
      return std::max(0.0, std::min(this->target_omega_, this->max_omega_));
    }

    if (!this->rotor_time_initialized_) {
      this->last_rotor_update_time_ = now;
      this->rotor_time_initialized_ = true;
      return this->rotor_omega_;
    }

    double dt = (now - this->last_rotor_update_time_).Double();
    this->last_rotor_update_time_ = now;

    if (dt < 0.0) {
      dt = 0.0;
    }
    dt = std::min(dt, 0.02);

    const double drive_torque_nm = this->DriveTorqueNewtonMeter();
    const double drag_torque_nm = this->DragTorqueNewtonMeter(this->rotor_omega_);
    const double net_torque_nm = drive_torque_nm - drag_torque_nm;

    const double inertia = std::max(1e-6, this->rotor_inertia_kgm2_);
    this->rotor_omega_ += (net_torque_nm / inertia) * dt;
    this->rotor_omega_ = std::max(0.0, std::min(this->rotor_omega_, this->max_omega_));

    return this->rotor_omega_;
  }

  double Blade1VisualPitchDeg(double t) const
  {
    double b1 = this->blade1_pitch_deg_;
    double b2 = this->blade2_pitch_deg_;
    this->CurrentBladePitchesDeg(b1, b2);

    if (!this->animate_pitch_indicators_) {
      return b1;
    }

    const double phase = 2.0 * M_PI * this->pitch_indicator_animation_frequency_hz_ * t;
    return b1 + this->pitch_indicator_animation_amplitude_deg_ * std::sin(phase);
  }

  double Blade2VisualPitchDeg(double t) const
  {
    double b1 = this->blade1_pitch_deg_;
    double b2 = this->blade2_pitch_deg_;
    this->CurrentBladePitchesDeg(b1, b2);

    if (!this->animate_pitch_indicators_) {
      return b2;
    }

    const double phase = 2.0 * M_PI * this->pitch_indicator_animation_frequency_hz_ * t;
    return b2 + this->pitch_indicator_animation_amplitude_deg_ * std::sin(phase + M_PI);
  }

  void UpdatePitchIndicators(double t)
  {
    if (!this->show_pitch_indicators_) {
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d rotor_pose = this->rotor_link_->WorldPose();
    const double blade1_pitch_rad = DegToRad(this->ClampPitchDeg(this->Blade1VisualPitchDeg(t)));
    const double blade2_pitch_rad = DegToRad(this->ClampPitchDeg(this->Blade2VisualPitchDeg(t)));

    ignition::math::Pose3d blade1_local_pose(
      ignition::math::Vector3d(this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_ + this->pitch_indicator_vertical_offset_m_),
      ignition::math::Quaterniond(blade1_pitch_rad, -this->pre_cone_rad_, 0.0));

    ignition::math::Pose3d blade2_local_pose(
      ignition::math::Vector3d(-this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_ + this->pitch_indicator_vertical_offset_m_),
      ignition::math::Quaterniond(blade2_pitch_rad, -this->pre_cone_rad_, M_PI));

    this->blade1_indicator_link_->SetWorldPose(rotor_pose * blade1_local_pose);
    this->blade2_indicator_link_->SetWorldPose(rotor_pose * blade2_local_pose);
#else
    const gazebo::math::Pose rotor_pose = this->rotor_link_->GetWorldPose();
    const double blade1_pitch_rad = DegToRad(this->ClampPitchDeg(this->Blade1VisualPitchDeg(t)));
    const double blade2_pitch_rad = DegToRad(this->ClampPitchDeg(this->Blade2VisualPitchDeg(t)));

    gazebo::math::Pose blade1_local_pose(
      gazebo::math::Vector3(this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_ + this->pitch_indicator_vertical_offset_m_),
      gazebo::math::Quaternion(blade1_pitch_rad, -this->pre_cone_rad_, 0.0));

    gazebo::math::Pose blade2_local_pose(
      gazebo::math::Vector3(-this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_ + this->pitch_indicator_vertical_offset_m_),
      gazebo::math::Quaternion(blade2_pitch_rad, -this->pre_cone_rad_, M_PI));

    this->blade1_indicator_link_->SetWorldPose(rotor_pose + blade1_local_pose);
    this->blade2_indicator_link_->SetWorldPose(rotor_pose + blade2_local_pose);
#endif
  }


  double WrapAngleRad(double angle) const
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  void StepRotorAzimuth(const common::Time &now, double omega)
  {
    if (!this->azimuth_time_initialized_) {
      this->last_azimuth_update_time_ = now;
      this->azimuth_time_initialized_ = true;
      return;
    }

    double dt = (now - this->last_azimuth_update_time_).Double();
    this->last_azimuth_update_time_ = now;

    if (dt < 0.0) {
      dt = 0.0;
    }
    dt = std::min(dt, 0.02);

    this->rotor_azimuth_rad_ = this->WrapAngleRad(this->rotor_azimuth_rad_ + omega * dt);
  }

  double CyclicBladePitchDeg(double azimuth_rad) const
  {
    if (!this->use_cyclic_pitch_) {
      // This function is only for cyclic mode, but return collective as a safe fallback.
      return this->collective_deg_;
    }

    const double phase_rad = DegToRad(this->cyclic_phase_offset_deg_);
    const double a = azimuth_rad + phase_rad;

    const double pitch =
      this->collective_deg_
      + this->roll_cyclic_deg_ * std::cos(a)
      + this->pitch_cyclic_deg_ * std::sin(a);

    return this->ClampPitchDeg(pitch);
  }

  void CurrentBladePitchesDeg(double &blade1_pitch, double &blade2_pitch) const
  {
    if (this->use_cyclic_pitch_) {
      blade1_pitch = this->CyclicBladePitchDeg(this->rotor_azimuth_rad_);
      blade2_pitch = this->CyclicBladePitchDeg(this->rotor_azimuth_rad_ + M_PI);
    } else {
      blade1_pitch = this->ClampPitchDeg(this->blade1_pitch_deg_);
      blade2_pitch = this->ClampPitchDeg(this->blade2_pitch_deg_);
    }
  }


  void OnCyclicCmd(ConstVector3dPtr &_msg)
  {
    if (!_msg) {
      return;
    }

    this->collective_deg_ = this->ClampPitchDeg(_msg->x());
    this->roll_cyclic_deg_ = std::max(-10.0, std::min(_msg->y(), 10.0));
    this->pitch_cyclic_deg_ = std::max(-10.0, std::min(_msg->z(), 10.0));

    gzmsg << "[TeeterRotorPlugin] cyclic_cmd received: collective="
          << this->collective_deg_
          << " deg, rollCyclic="
          << this->roll_cyclic_deg_
          << " deg, pitchCyclic="
          << this->pitch_cyclic_deg_
          << " deg"
          << std::endl;
  }

  void OnEngineCmd(ConstVector3dPtr &_msg)
  {
    if (!_msg) {
      return;
    }

    this->engine_throttle_ = this->ClampEngineThrottle(_msg->x());
    this->cyclic_phase_offset_deg_ = _msg->y();

    gzmsg << "[TeeterRotorPlugin] engine_cmd received: throttle="
          << this->engine_throttle_
          << ", phaseOffset="
          << this->cyclic_phase_offset_deg_
          << " deg"
          << std::endl;
  }

  void PrintFactSheet() const
  {
    gzmsg << "\n"
          << "================ Teeter Rotor GP-76 CLEAN ================\n"
          << "Engine dynamics: " << (this->use_engine_dynamics_ ? "on" : "off") << "\n"
          << "Engine radius: " << this->engine_radius_ft_ << " ft\n"
          << "Engine thrust each: " << this->engine_thrust_lb_each_ << " lbf\n"
          << "Num engines: " << this->num_engines_ << "\n"
          << "Max drive torque: " << this->max_drive_torque_nm_
          << " N*m / " << this->max_drive_torque_nm_ / 1.3558179483314 << " ft-lb\n"
          << "kTorque: " << this->k_torque_ << " N*m/(rad/s)^2\n"
          << "Rotor inertia: " << this->rotor_inertia_kgm2_ << " kg*m^2\n"
          << "Payload: " << this->payload_weight_lb_ << " lb\n"
          << "Fixed blade pitches: [" << this->blade1_pitch_deg_ << ", "
          << this->blade2_pitch_deg_ << "] deg\n"
          << "Cyclic pitch: " << (this->use_cyclic_pitch_ ? "on" : "off")
          << ", collective: " << this->collective_deg_
          << ", roll cyclic: " << this->roll_cyclic_deg_
          << ", pitch cyclic: " << this->pitch_cyclic_deg_
          << ", phase: " << this->cyclic_phase_offset_deg_ << " deg\n"
          << "Pre-cone: " << this->pre_cone_angle_deg_ << " deg, force point x/z: "
          << this->blade_lift_x_m_ << " / " << this->blade_lift_z_m_ << " m\n"
          << "Command topics: " << (this->enable_command_topics_ ? "on" : "off")
          << ", namespace: " << this->command_topic_namespace_ << "\n"
          << "==========================================================\n"
          << std::endl;
  }

  void OnUpdate()
  {
    if (!this->base_link_ || !this->rotor_link_ || !this->blade1_indicator_link_ ||
        !this->blade2_indicator_link_ || !this->joint_) {
      return;
    }

    const common::Time now = this->model_->GetWorld()->SimTime();
    const double omega_cmd = this->StepRotorOmega(now);
    this->StepRotorAzimuth(now, omega_cmd);

    double blade1_pitch_cmd_deg = this->blade1_pitch_deg_;
    double blade2_pitch_cmd_deg = this->blade2_pitch_deg_;
    this->CurrentBladePitchesDeg(blade1_pitch_cmd_deg, blade2_pitch_cmd_deg);

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d angular_vel(0.0, 0.0, omega_cmd);
#else
    gazebo::math::Vector3 angular_vel(0.0, 0.0, omega_cmd);
#endif

    this->rotor_link_->SetAngularVel(angular_vel);
    this->UpdatePitchIndicators(now.Double());

    const double blade1_lift_n = this->BladeLiftNewton(omega_cmd, blade1_pitch_cmd_deg);
    const double blade2_lift_n = this->BladeLiftNewton(omega_cmd, blade2_pitch_cmd_deg);

#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d rotor_pose = this->rotor_link_->WorldPose();

    const ignition::math::Vector3d blade1_local(this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_);
    const ignition::math::Vector3d blade2_local(-this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_);

    const ignition::math::Vector3d blade1_pos_world =
      rotor_pose.Pos() + rotor_pose.Rot().RotateVector(blade1_local);
    const ignition::math::Vector3d blade2_pos_world =
      rotor_pose.Pos() + rotor_pose.Rot().RotateVector(blade2_local);

    const ignition::math::Vector3d blade1_force_world =
      rotor_pose.Rot().RotateVector(ignition::math::Vector3d(0.0, 0.0, blade1_lift_n));
    const ignition::math::Vector3d blade2_force_world =
      rotor_pose.Rot().RotateVector(ignition::math::Vector3d(0.0, 0.0, blade2_lift_n));

    const ignition::math::Vector3d payload_force_world(0.0, 0.0, -this->payload_weight_n_);
#else
    const gazebo::math::Pose rotor_pose = this->rotor_link_->GetWorldPose();

    const gazebo::math::Vector3 blade1_local(this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_);
    const gazebo::math::Vector3 blade2_local(-this->blade_lift_x_m_, 0.0, this->blade_lift_z_m_);

    const gazebo::math::Vector3 blade1_pos_world =
      rotor_pose.pos + rotor_pose.rot.RotateVector(blade1_local);
    const gazebo::math::Vector3 blade2_pos_world =
      rotor_pose.pos + rotor_pose.rot.RotateVector(blade2_local);

    const gazebo::math::Vector3 blade1_force_world =
      rotor_pose.rot.RotateVector(gazebo::math::Vector3(0.0, 0.0, blade1_lift_n));
    const gazebo::math::Vector3 blade2_force_world =
      rotor_pose.rot.RotateVector(gazebo::math::Vector3(0.0, 0.0, blade2_lift_n));

    const gazebo::math::Vector3 payload_force_world(0.0, 0.0, -this->payload_weight_n_);
#endif

    this->base_link_->AddForceAtWorldPosition(blade1_force_world, blade1_pos_world);
    this->base_link_->AddForceAtWorldPosition(blade2_force_world, blade2_pos_world);
    this->base_link_->AddForce(payload_force_world);

    if ((now - this->last_print_time_).Double() > 1.0) {
      const double rpm = RadPerSecToRpm(omega_cmd);
      const double total_lift = blade1_lift_n + blade2_lift_n;
      const double net_n = total_lift - this->payload_weight_n_ - this->EmptyWeightNewton();

      gzmsg << "[TeeterRotorPlugin] rpm = "
            << rpm
            << ", payload = " << this->payload_weight_lb_ << " lb"
            << ", azimuth = " << this->rotor_azimuth_rad_ * 180.0 / M_PI << " deg"
            << ", blade pitches = [" << blade1_pitch_cmd_deg << ", " << blade2_pitch_cmd_deg << "] deg"
            << ", total lift = " << total_lift << " N"
            << ", net = " << net_n << " N"
            << ", drive torque = " << this->DriveTorqueNewtonMeter() << " N*m"
            << ", drag torque = " << this->DragTorqueNewtonMeter(omega_cmd) << " N*m"
            << std::endl;

      this->last_print_time_ = now;
    }
  }

private:
  // Runtime command topics.
  bool enable_command_topics_{true};
  std::string command_topic_namespace_{"teeter_rotor"};
  transport::NodePtr node_;
  transport::SubscriberPtr cyclic_cmd_sub_;
  transport::SubscriberPtr engine_cmd_sub_;

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  physics::LinkPtr rotor_link_;
  physics::LinkPtr blade1_indicator_link_;
  physics::LinkPtr blade2_indicator_link_;
  physics::JointPtr joint_;
  event::ConnectionPtr update_connection_;

  // Primary variables.
  double target_rpm_{0.0};
  double payload_weight_lb_{360.0};
  double blade1_pitch_deg_{7.27};
  double blade2_pitch_deg_{7.27};

  // Azimuth-based cyclic pitch.
  bool use_cyclic_pitch_{true};
  double collective_deg_{7.27};
  double roll_cyclic_deg_{0.0};
  double pitch_cyclic_deg_{0.0};
  double cyclic_phase_offset_deg_{0.0};
  double rotor_azimuth_rad_{0.0};
  bool azimuth_time_initialized_{false};
  common::Time last_azimuth_update_time_{0};

  // Engine dynamics.
  bool use_engine_dynamics_{true};
  double engine_throttle_{1.0};
  double engine_radius_ft_{3.0};
  double engine_radius_m_{0.9144};
  double engine_thrust_lb_each_{54.0};
  double engine_thrust_n_each_{240.20};
  double rotor_inertia_kgm2_{35.0};
  double k_torque_{0.831};
  double max_drive_torque_nm_{440.6};
  double rotor_omega_{0.0};
  bool rotor_time_initialized_{false};
  common::Time last_rotor_update_time_{0};

  // Visual pitch indicators.
  bool show_pitch_indicators_{true};
  bool animate_pitch_indicators_{false};
  double pitch_indicator_animation_amplitude_deg_{5.0};
  double pitch_indicator_animation_frequency_hz_{0.25};
  double pitch_indicator_vertical_offset_m_{0.18};

  // Fixed constants.
  double empty_weight_lb_{42.0};
  double k_thrust_{3.69};
  double max_rpm_{220.0};

  // Pitch model.
  double hover_pitch_deg_{7.27};
  double zero_lift_pitch_deg_{1.0};
  double min_pitch_deg_{2.0};
  double max_pitch_deg_{12.0};

  // Metadata.
  double rotor_radius_ft_{12.0};
  double hub_radius_ft_{3.0};
  int num_blades_{2};
  double blade_chord_in_{10.5};
  std::string airfoil_{"NACA23015"};
  double tip_speed_fps_{276.0};
  double design_gross_weight_lb_{440.0};
  double tip_weight_lb_{2.53};
  double blade_spanwise_density_lb_ft_{0.3};
  double forward_speed_mph_{20.0};
  double rotor_twist_deg_{6.0};
  double rotor_plane_height_ft_{5.0};
  double teeter_angle_limit_deg_{15.0};
  double pre_cone_angle_deg_{11.0};
  double gyro_precession_torque_ft_lb_{262.0};
  double torque_required_ft_lb_{325.0};
  double equivalent_shaft_hp_{16.0};
  double propulsive_efficiency_{0.85};
  int num_engines_{2};
  double engine_hp_each_{8.0};
  double main_blade_spar_od_mm_{30.0};
  double main_blade_spar_id_mm_{28.0};

  // SI / derived.
  double max_omega_{23.038346};
  double target_omega_{0.0};
  double empty_weight_kg_{19.05};
  double payload_mass_kg_{163.29};
  double payload_weight_n_{1601.36};
  double current_gross_weight_lb_{402.0};
  double current_gross_weight_n_{1788.18};

  double rotor_radius_m_{3.6576};
  double hub_radius_m_{0.9144};
  double blade_chord_m_{0.2667};
  double blade_effective_span_m_{2.7432};
  double blade_lift_radius_m_{2.286};
  double blade_lift_x_m_{2.2600};
  double blade_lift_z_m_{0.2617};
  double pre_cone_rad_{0.191986};
  double disk_area_m2_{42.04};
  double solidity_{0.0464};

  common::Time last_print_time_{0};
};

GZ_REGISTER_MODEL_PLUGIN(TeeterRotorPlugin)
}
