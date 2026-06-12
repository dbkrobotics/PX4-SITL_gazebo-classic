#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <CommandMotorSpeed.pb.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>

namespace gazebo
{
class RisingStarPlugin : public ModelPlugin
{
public:
  RisingStarPlugin() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    std::string base_link_name = "base_link";
    std::string rotor_link_name = "rotor_link";
    std::string teeter_beam_link_name = "teeter_beam_link";
    std::string blade1_pitch_link_name = "blade_1_pitch_link";
    std::string blade2_pitch_link_name = "blade_2_pitch_link";
    std::string payload_link_name = "payload_link";
    std::string joint_name = "main_rotor_joint";
    std::string teeter_hinge_joint_name = "teeter_hinge_joint";

    this->ReadString(sdf, "baseLinkName", base_link_name);
    this->ReadString(sdf, "rotorLinkName", rotor_link_name);
    this->ReadString(sdf, "teeterBeamLinkName", teeter_beam_link_name);
    this->ReadString(sdf, "blade1PitchLinkName", blade1_pitch_link_name);
    this->ReadString(sdf, "blade2PitchLinkName", blade2_pitch_link_name);
    this->ReadString(sdf, "payloadLinkName", payload_link_name);
    this->ReadString(sdf, "jointName", joint_name);
    this->ReadString(sdf, "teeterHingeJointName", teeter_hinge_joint_name);

    this->base_link_ = this->model_->GetLink(base_link_name);
    this->rotor_link_ = this->model_->GetLink(rotor_link_name);
    this->teeter_beam_link_ = this->model_->GetLink(teeter_beam_link_name);
    this->blade1_pitch_link_ = this->model_->GetLink(blade1_pitch_link_name);
    this->blade2_pitch_link_ = this->model_->GetLink(blade2_pitch_link_name);
    this->payload_link_ = this->model_->GetLink(payload_link_name);
    this->joint_ = this->model_->GetJoint(joint_name);
    this->teeter_hinge_joint_ = this->model_->GetJoint(teeter_hinge_joint_name);

    this->node_.reset(new transport::Node());
    this->node_->Init(this->model_->GetWorld()->Name());
    this->payload_visual_pub_ = this->node_->Advertise<msgs::Visual>("~/visual", 10);

    if (!this->base_link_) {
      gzerr << "[RisingStarPlugin] Base link not found: " << base_link_name << std::endl;
      return;
    }
    if (!this->rotor_link_) {
      gzerr << "[RisingStarPlugin] Rotor link not found: " << rotor_link_name << std::endl;
      return;
    }
    if (!this->teeter_beam_link_) {
      gzerr << "[RisingStarPlugin] Teeter beam link not found: " << teeter_beam_link_name << std::endl;
      return;
    }
    if (!this->blade1_pitch_link_) {
      gzerr << "[RisingStarPlugin] Blade 1 pitch link not found: " << blade1_pitch_link_name << std::endl;
      return;
    }
    if (!this->blade2_pitch_link_) {
      gzerr << "[RisingStarPlugin] Blade 2 pitch link not found: " << blade2_pitch_link_name << std::endl;
      return;
    }
    if (!this->payload_link_) {
      gzwarn << "[RisingStarPlugin] Payload link not found: " << payload_link_name
             << ". Payload parameters will not change model mass/inertia." << std::endl;
    }
    if (!this->joint_) {
      gzerr << "[RisingStarPlugin] Joint not found: " << joint_name << std::endl;
      return;
    }
    if (!this->teeter_hinge_joint_) {
      gzerr << "[RisingStarPlugin] Teeter hinge joint not found: " << teeter_hinge_joint_name << std::endl;
      return;
    }

    // Primary variables.
    this->ReadDouble(sdf, "targetRpm", this->target_rpm_);
    this->ReadBool(sdf, "payloadEnabled", this->payload_enabled_);
    this->ReadDouble(sdf, "payloadMassLb", this->payload_mass_lb_);
    this->ReadVector3(sdf, "payloadOffsetFromCogM", this->payload_offset_from_cog_m_);
    this->ReadDouble(sdf, "payloadOuterDiameterIn", this->payload_outer_diameter_in_);
    this->ReadDouble(sdf, "payloadInnerDiameterIn", this->payload_inner_diameter_in_);
    this->ReadDouble(sdf, "payloadThicknessIn", this->payload_thickness_in_);
    this->ReadDouble(sdf, "payloadDisabledScale", this->payload_disabled_scale_);
    this->ReadDouble(sdf, "blade1PitchDeg", this->blade1_pitch_deg_);
    this->ReadDouble(sdf, "blade2PitchDeg", this->blade2_pitch_deg_);

    // Azimuth-based cyclic pitch.
    this->ReadBool(sdf, "useCyclicPitch", this->use_cyclic_pitch_);
    this->ReadDouble(sdf, "collectiveDeg", this->collective_deg_);
    this->ReadDouble(sdf, "rollCyclicDeg", this->roll_cyclic_deg_);
    this->ReadDouble(sdf, "pitchCyclicDeg", this->pitch_cyclic_deg_);
    this->ReadDouble(sdf, "cyclicPhaseOffsetDeg", this->cyclic_phase_offset_deg_);
    this->ReadString(sdf, "cyclicAzimuthSource", this->cyclic_azimuth_source_);
    this->ReadDouble(sdf, "cyclicAzimuthZeroOffsetRad", this->cyclic_azimuth_zero_offset_rad_);
    this->ReadDouble(sdf, "cyclicAzimuthDirection", this->cyclic_azimuth_direction_);
    this->ReadString(sdf, "cyclicCommandFrame", this->cyclic_command_frame_);
    this->ReadBool(sdf, "swapCyclicInputs", this->swap_cyclic_inputs_);
    this->ReadDouble(sdf, "rollCyclicSign", this->roll_cyclic_sign_);
    this->ReadDouble(sdf, "pitchCyclicSign", this->pitch_cyclic_sign_);
    this->ReadBool(sdf, "enableTeeterDynamics", this->enable_teeter_dynamics_);
    this->ReadDouble(sdf, "teeterInertiaKgm2", this->teeter_inertia_kgm2_);
    this->ReadDouble(sdf, "teeterDampingNmPerRadS", this->teeter_damping_nm_per_rad_s_);
    this->ReadDouble(sdf, "teeterStiffnessNmPerRad", this->teeter_stiffness_nm_per_rad_);
    this->ReadDouble(sdf, "teeterMomentSign", this->teeter_moment_sign_);
    this->ReadDouble(sdf, "teeterRateLimitDegS", this->teeter_rate_limit_deg_s_);

    // PX4 actuator bridge.
    this->ReadBool(sdf, "enablePx4ActuatorInput", this->enable_px4_actuator_input_);
    this->ReadString(sdf, "px4ActuatorTopic", this->px4_actuator_topic_);
    this->ReadString(sdf, "px4InputMode", this->px4_input_mode_);
    this->ReadString(sdf, "px4PitchInputMode", this->px4_pitch_input_mode_);
    this->ReadDouble(sdf, "px4MotorSpeedScale", this->px4_motor_speed_scale_);
    this->ReadDouble(sdf, "px4ThrottleZeroRaw", this->px4_throttle_zero_raw_);
    this->ReadDouble(sdf, "px4ThrottleFullRaw", this->px4_throttle_full_raw_);
    this->ReadDouble(sdf, "px4CollectiveZeroRaw", this->px4_collective_zero_raw_);
    this->ReadDouble(sdf, "px4CollectiveFullRaw", this->px4_collective_full_raw_);
    this->ReadDouble(sdf, "minCollectiveCmdDeg", this->min_collective_cmd_deg_);
    this->ReadDouble(sdf, "maxCollectiveCmdDeg", this->max_collective_cmd_deg_);
    this->ReadDouble(sdf, "maxCyclicDeg", this->max_cyclic_deg_);
    this->ReadDouble(sdf, "px4CommandTimeoutSec", this->px4_command_timeout_sec_);
    this->ReadBool(sdf, "printPx4InputDebug", this->print_px4_input_debug_);
    this->ReadDouble(sdf, "px4InputDebugIntervalSec", this->px4_input_debug_interval_sec_);
    this->ReadBool(sdf, "printTeeterStateDebug", this->print_teeter_state_debug_);
    this->ReadDouble(sdf, "teeterStateDebugIntervalSec", this->teeter_state_debug_interval_sec_);

    this->ReadBool(sdf, "visualInspectionMode", this->visual_inspection_mode_);
    this->ReadDouble(sdf, "visualInspectionBladePitchDeg", this->visual_inspection_blade_pitch_deg_);
    this->ReadDouble(sdf, "visualInspectionRotorAzimuthDeg", this->visual_inspection_rotor_azimuth_deg_);
    this->ReadDouble(sdf, "visualInspectionTeeterDeg", this->visual_inspection_teeter_deg_);
    this->ReadBool(sdf, "updateBladePitchVisualsDuringFlight", this->update_blade_pitch_visuals_during_flight_);

    // Engine dynamics.
    this->ReadBool(sdf, "useEngineDynamics", this->use_engine_dynamics_);
    this->ReadDouble(sdf, "engineThrottle", this->engine_throttle_);
    this->ReadDouble(sdf, "engineRadiusFt", this->engine_radius_ft_);
    this->ReadDouble(sdf, "engineThrustLbEach", this->engine_thrust_lb_each_);
    this->ReadDouble(sdf, "rotorInertiaKgm2", this->rotor_inertia_kgm2_);
    this->ReadDouble(sdf, "kTorque", this->k_torque_);

    // Fixed constants.
    this->ReadDouble(sdf, "emptyWeightLb", this->empty_weight_lb_);
    this->ReadDouble(sdf, "kThrust", this->k_thrust_);
    this->ReadDouble(sdf, "maxRpm", this->max_rpm_);

    // Pitch model.
    this->ReadDouble(sdf, "hoverPitchDeg", this->hover_pitch_deg_);
    this->ReadDouble(sdf, "zeroLiftPitchDeg", this->zero_lift_pitch_deg_);
    this->ReadDouble(sdf, "minPitchDeg", this->min_pitch_deg_);
    this->ReadDouble(sdf, "maxPitchDeg", this->max_pitch_deg_);
    this->ReadBool(sdf, "useBladeElementLift", this->use_blade_element_lift_);
    this->ReadDouble(sdf, "airDensityKgM3", this->air_density_kgm3_);
    this->ReadDouble(sdf, "bladeLiftSlopePerRad", this->blade_lift_slope_per_rad_);
    this->ReadInt(sdf, "bladeElementSections", this->blade_element_sections_);
    this->ReadDouble(sdf, "bladeElementLiftScale", this->blade_element_lift_scale_);

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
    this->cyclic_azimuth_direction_ = this->cyclic_azimuth_direction_ >= 0.0 ? 1.0 : -1.0;

    this->empty_weight_kg_ = LbMassToKg(this->empty_weight_lb_);
    this->payload_mass_kg_ = this->payload_enabled_ ? LbMassToKg(this->payload_mass_lb_) : 0.0;
    this->physical_payload_weight_n_ = this->payload_enabled_ ? LbForceToN(this->payload_mass_lb_) : 0.0;
    this->current_gross_weight_lb_ = this->empty_weight_lb_
      + (this->payload_enabled_ ? this->payload_mass_lb_ : 0.0);
    this->current_gross_weight_n_ = LbForceToN(this->current_gross_weight_lb_);

    this->engine_radius_m_ = FtToM(this->engine_radius_ft_);
    this->engine_thrust_n_each_ = LbForceToN(this->engine_thrust_lb_each_);
    this->max_drive_torque_nm_ =
      static_cast<double>(this->num_engines_) * this->engine_thrust_n_each_ * this->engine_radius_m_;
    this->engine_available_power_w_ =
      HpToW(this->equivalent_shaft_hp_) * this->propulsive_efficiency_;

    this->blade_effective_span_m_ = this->rotor_radius_m_ - this->hub_radius_m_;
    this->blade_lift_radius_m_ = 0.5 * (this->rotor_radius_m_ + this->hub_radius_m_);
    this->pre_cone_rad_ = DegToRad(this->pre_cone_angle_deg_);
    this->blade_lift_span_m_ = this->hub_radius_m_ + 0.5 * this->blade_effective_span_m_ * std::cos(this->pre_cone_rad_);
    this->blade_lift_z_m_ = 0.5 * this->blade_effective_span_m_ * std::sin(this->pre_cone_rad_);
    this->disk_area_m2_ = M_PI * this->rotor_radius_m_ * this->rotor_radius_m_;
    this->solidity_ = static_cast<double>(this->num_blades_) * this->blade_chord_m_ /
      (M_PI * this->rotor_radius_m_);

    if (this->use_blade_element_lift_ && this->blade_element_lift_scale_ <= 0.0) {
      const double design_lift_n = LbForceToN(this->design_gross_weight_lb_);
      const double design_blade_lift_n =
        this->BladeElementLiftNewtonUnscaled(this->max_omega_, this->hover_pitch_deg_);
      const double design_total_lift_n =
        std::max(1.0, static_cast<double>(this->num_blades_) * design_blade_lift_n);
      this->blade_element_lift_scale_ = design_lift_n / design_total_lift_n;
    }

    this->ConfigurePayloadLink();

    if (this->enable_px4_actuator_input_ && !this->visual_inspection_mode_) {
      this->px4_motor_speed_sub_ =
        this->node_->Subscribe(this->px4_actuator_topic_,
          &RisingStarPlugin::OnPx4MotorSpeed, this);

      std::cout << "[RisingStarPlugin] PX4 actuator bridge enabled. Topic: "
                << this->px4_actuator_topic_
                << ", input debug: " << (this->print_px4_input_debug_ ? "on" : "off")
                << ", interval: " << this->px4_input_debug_interval_sec_
                << " sec"
                << std::endl;
    }

    this->PrintFactSheet();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RisingStarPlugin::OnUpdate, this));
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

  void ReadVector3(sdf::ElementPtr sdf, const std::string &name, ignition::math::Vector3d &value)
  {
    if (sdf->HasElement(name)) {
      value = sdf->Get<ignition::math::Vector3d>(name);
    }
  }

  void ConfigurePayloadLink()
  {
    if (!this->payload_link_) {
      return;
    }

    const bool enabled = this->payload_enabled_ && this->payload_mass_lb_ > 0.0;
    const double mass_kg = enabled ? this->payload_mass_kg_ : this->disabled_payload_mass_kg_;
    const double geometry_scale = enabled ? 1.0 : std::max(0.01, this->payload_disabled_scale_);
    const double outer_radius_m = geometry_scale *
      std::max(0.001, 0.5 * InToM(this->payload_outer_diameter_in_));
    const double inner_radius_m = geometry_scale *
      std::max(0.0, 0.5 * InToM(this->payload_inner_diameter_in_));
    const double thickness_m = geometry_scale *
      std::max(0.001, InToM(this->payload_thickness_in_));
    const double radius_term = outer_radius_m * outer_radius_m + inner_radius_m * inner_radius_m;
    const double ixx_iyy = (mass_kg / 12.0) * (3.0 * radius_term + thickness_m * thickness_m);
    const double izz = 0.5 * mass_kg * radius_term;

    physics::InertialPtr inertial(new physics::Inertial);
    inertial->SetMass(mass_kg);
    inertial->SetCoG(0.0, 0.0, 0.0);
    inertial->SetInertiaMatrix(ixx_iyy, ixx_iyy, izz, 0.0, 0.0, 0.0);
    this->payload_link_->SetInertial(inertial);
    this->payload_link_->SetGravityMode(true);
    this->payload_link_->SetScale(ignition::math::Vector3d(
      geometry_scale,
      geometry_scale,
      geometry_scale));
    this->SetPayloadVisualVisible(enabled);

    const ignition::math::Pose3d payload_pose(
      this->payload_offset_from_cog_m_,
      ignition::math::Quaterniond(0.0, 0.0, 0.0));
    this->payload_link_->SetInitialRelativePose(payload_pose);
    this->payload_link_->SetRelativePose(payload_pose);
  }

  void SetPayloadVisualVisible(bool visible)
  {
    if (!this->payload_visual_pub_ || !this->payload_link_) {
      return;
    }

    msgs::Visual visual_msg;
    visual_msg.set_name(this->payload_link_->GetScopedName() + "::payload_plate_visual");
    visual_msg.set_parent_name(this->payload_link_->GetScopedName());
    visual_msg.set_visible(visible);
    visual_msg.set_transparency(visible ? 0.0 : 1.0);
    this->payload_visual_pub_->Publish(visual_msg);
  }

  void SetVisualVisible(physics::LinkPtr link, const std::string &visual_name, bool visible)
  {
    if (!this->payload_visual_pub_ || !link) {
      return;
    }

    msgs::Visual visual_msg;
    visual_msg.set_name(link->GetScopedName() + "::" + visual_name);
    visual_msg.set_parent_name(link->GetScopedName());
    visual_msg.set_visible(visible);
    visual_msg.set_transparency(visible ? 0.0 : 1.0);
    this->payload_visual_pub_->Publish(visual_msg);
  }

  void UpdateBladeVisualMode()
  {
    const bool pitch_link_visible =
      this->visual_inspection_mode_ || this->update_blade_pitch_visuals_during_flight_;
    const bool fixed_blade_visible = !pitch_link_visible;

    this->SetVisualVisible(this->teeter_beam_link_, "blade_1_visual", fixed_blade_visible);
    this->SetVisualVisible(this->teeter_beam_link_, "blade_2_visual", fixed_blade_visible);
    this->SetVisualVisible(this->blade1_pitch_link_, "blade_1_visual", pitch_link_visible);
    this->SetVisualVisible(this->blade2_pitch_link_, "blade_2_visual", pitch_link_visible);
  }

  void SetBladeVisualPitch(double blade1_pitch_deg, double blade2_pitch_deg)
  {
    if (!this->teeter_beam_link_ || !this->blade1_pitch_link_ || !this->blade2_pitch_link_) {
      return;
    }

    const double blade1_pitch_rad = DegToRad(blade1_pitch_deg);
    const double blade2_pitch_rad = DegToRad(blade2_pitch_deg);

#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d teeter_pose = this->teeter_beam_link_->WorldPose();
    const ignition::math::Pose3d blade1_pose(
      ignition::math::Vector3d(0.0, -this->blade_lift_span_m_, this->blade_lift_z_m_),
      ignition::math::Quaterniond(blade1_pitch_rad, -this->pre_cone_rad_, -0.5 * M_PI));
    const ignition::math::Pose3d blade2_pose(
      ignition::math::Vector3d(0.0, this->blade_lift_span_m_, this->blade_lift_z_m_),
      ignition::math::Quaterniond(blade2_pitch_rad, -this->pre_cone_rad_, 0.5 * M_PI));
    this->blade1_pitch_link_->SetWorldPose(teeter_pose * blade1_pose);
    this->blade2_pitch_link_->SetWorldPose(teeter_pose * blade2_pose);
#else
    const gazebo::math::Pose teeter_pose = this->teeter_beam_link_->GetWorldPose();
    const gazebo::math::Pose blade1_pose(
      gazebo::math::Vector3(0.0, -this->blade_lift_span_m_, this->blade_lift_z_m_),
      gazebo::math::Quaternion(blade1_pitch_rad, -this->pre_cone_rad_, -0.5 * M_PI));
    const gazebo::math::Pose blade2_pose(
      gazebo::math::Vector3(0.0, this->blade_lift_span_m_, this->blade_lift_z_m_),
      gazebo::math::Quaternion(blade2_pitch_rad, -this->pre_cone_rad_, 0.5 * M_PI));
    this->blade1_pitch_link_->SetWorldPose(teeter_pose + blade1_pose);
    this->blade2_pitch_link_->SetWorldPose(teeter_pose + blade2_pose);
#endif
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
    if (this->use_blade_element_lift_) {
      return this->blade_element_lift_scale_ *
        this->BladeElementLiftNewtonUnscaled(omega, pitch_deg);
    }

    return 0.5 * this->k_thrust_ * omega * omega * this->PitchFactor(pitch_deg);
  }

  double BladeElementLiftNewtonUnscaled(double omega, double pitch_deg) const
  {
    const int sections = std::max(1, this->blade_element_sections_);
    const double root_radius = std::max(0.0, std::min(this->hub_radius_m_, this->rotor_radius_m_));
    const double span = std::max(0.0, this->rotor_radius_m_ - root_radius);
    double lift_n = 0.0;

    for (int i = 0; i < sections; ++i) {
      const double r0 = root_radius + span * static_cast<double>(i) / static_cast<double>(sections);
      const double r1 = root_radius + span * static_cast<double>(i + 1) / static_cast<double>(sections);
      const double r = 0.5 * (r0 + r1);
      const double mu = span > 1e-6 ? (r - root_radius) / span : 0.0;
      const double local_pitch_deg = pitch_deg + (0.5 - mu) * this->rotor_twist_deg_;
      const double alpha_rad = DegToRad(local_pitch_deg - this->zero_lift_pitch_deg_);
      const double cl = std::max(0.0, this->blade_lift_slope_per_rad_ * alpha_rad);
      const double local_speed = omega * r;
      const double dr = r1 - r0;

      lift_n += 0.5 * this->air_density_kgm3_ * local_speed * local_speed *
        this->blade_chord_m_ * cl * dr;
    }

    return lift_n;
  }

  double EmptyWeightNewton() const
  {
    return LbForceToN(this->empty_weight_lb_);
  }

  double ClampEngineThrottle(double throttle) const
  {
    return std::max(0.0, std::min(throttle, 1.0));
  }

  double DriveTorqueNewtonMeter(double omega) const
  {
    const double torque_from_power_nm =
      this->engine_available_power_w_ / std::max(omega, 1e-3);
    const double drive_torque_limit_nm =
      std::min(this->max_drive_torque_nm_, torque_from_power_nm);

    return this->ClampEngineThrottle(this->engine_throttle_) * drive_torque_limit_nm;
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

    const double drive_torque_nm = this->DriveTorqueNewtonMeter(this->rotor_omega_);
    const double drag_torque_nm = this->DragTorqueNewtonMeter(this->rotor_omega_);
    const double net_torque_nm = drive_torque_nm - drag_torque_nm;

    const double inertia = std::max(1e-6, this->rotor_inertia_kgm2_);
    this->rotor_omega_ += (net_torque_nm / inertia) * dt;
    this->rotor_omega_ = std::max(0.0, std::min(this->rotor_omega_, this->max_omega_));

    return this->rotor_omega_;
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

  double JointRotorAzimuthRad() const
  {
    if (!this->joint_) {
      return this->rotor_azimuth_rad_;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    const double joint_position = this->joint_->Position(0);
#else
    const double joint_position = this->joint_->GetAngle(0).Radian();
#endif

    return this->WrapAngleRad(
      this->cyclic_azimuth_direction_ * joint_position + this->cyclic_azimuth_zero_offset_rad_);
  }

  double CyclicRotorAzimuthRad() const
  {
    if (this->cyclic_azimuth_source_ == "joint" ||
        this->cyclic_azimuth_source_ == "as5600") {
      return this->JointRotorAzimuthRad();
    }

    return this->rotor_azimuth_rad_;
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
      - this->pitch_cyclic_deg_ * std::sin(a);

    return this->ClampPitchDeg(pitch);
  }

  void CurrentBladePitchesDeg(double &blade1_pitch, double &blade2_pitch) const
  {
    if (this->use_cyclic_pitch_) {
      const double cyclic_azimuth_rad = this->CyclicRotorAzimuthRad();
      blade1_pitch = this->CyclicBladePitchDeg(cyclic_azimuth_rad);
      blade2_pitch = this->CyclicBladePitchDeg(cyclic_azimuth_rad + M_PI);
    } else {
      blade1_pitch = this->ClampPitchDeg(this->blade1_pitch_deg_);
      blade2_pitch = this->ClampPitchDeg(this->blade2_pitch_deg_);
    }
  }

  double StepTeeterAngle(const common::Time &now, double blade1_lift_n, double blade2_lift_n)
  {
    const double limit_rad = std::max(0.0, DegToRad(this->teeter_angle_limit_deg_));

    if (!this->enable_teeter_dynamics_) {
      this->teeter_angle_rad_ = 0.0;
      this->teeter_rate_rad_s_ = 0.0;
      return this->teeter_angle_rad_;
    }

    if (!this->teeter_time_initialized_) {
      this->last_teeter_update_time_ = now;
      this->teeter_time_initialized_ = true;
      return this->teeter_angle_rad_;
    }

    double dt = (now - this->last_teeter_update_time_).Double();
    this->last_teeter_update_time_ = now;

    if (dt < 0.0) {
      dt = 0.0;
    }

    dt = std::min(dt, 0.02);

    // Blade 1 is on -Y (body right) and blade 2 is on +Y (body left) in rotor
    // coordinates. Vertical lift imbalance therefore creates a teeter moment
    // about the rotor X axis.
    const double lift_moment_nm =
      this->teeter_moment_sign_ * this->blade_lift_span_m_ * (blade2_lift_n - blade1_lift_n);
    const double restoring_moment_nm = -this->teeter_stiffness_nm_per_rad_ * this->teeter_angle_rad_;
    const double damping_moment_nm = -this->teeter_damping_nm_per_rad_s_ * this->teeter_rate_rad_s_;
    const double inertia = std::max(1e-6, this->teeter_inertia_kgm2_);
    const double teeter_accel_rad_s2 =
      (lift_moment_nm + restoring_moment_nm + damping_moment_nm) / inertia;

    this->teeter_rate_rad_s_ += teeter_accel_rad_s2 * dt;

    const double rate_limit_rad_s = std::max(0.0, DegToRad(this->teeter_rate_limit_deg_s_));
    if (rate_limit_rad_s > 0.0) {
      this->teeter_rate_rad_s_ = std::max(-rate_limit_rad_s,
        std::min(this->teeter_rate_rad_s_, rate_limit_rad_s));
    }

    this->teeter_angle_rad_ += this->teeter_rate_rad_s_ * dt;

    if (limit_rad > 0.0) {
      if (this->teeter_angle_rad_ > limit_rad) {
        this->teeter_angle_rad_ = limit_rad;
        this->teeter_rate_rad_s_ = std::min(0.0, this->teeter_rate_rad_s_);

      } else if (this->teeter_angle_rad_ < -limit_rad) {
        this->teeter_angle_rad_ = -limit_rad;
        this->teeter_rate_rad_s_ = std::max(0.0, this->teeter_rate_rad_s_);
      }
    }

    return this->teeter_angle_rad_;
  }

  double TeeterHingeAngleRad() const
  {
    if (!this->teeter_hinge_joint_) {
      return 0.0;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    return this->teeter_hinge_joint_->Position(0);
#else
    return this->teeter_hinge_joint_->GetAngle(0).Radian();
#endif
  }


  double Clamp01(double value) const
  {
    return std::max(0.0, std::min(value, 1.0));
  }

  double NormalizePx4MotorSpeed(double motor_speed) const
  {
    const double scale = std::max(1e-6, this->px4_motor_speed_scale_);
    return this->Clamp01(motor_speed / scale);
  }

  double NormalizePx4MotorSpeedRange(double motor_speed, double zero_raw, double full_raw) const
  {
    const double span = std::max(1e-6, full_raw - zero_raw);
    return this->Clamp01((motor_speed - zero_raw) / span);
  }

  double Map01ToSigned(double value01, double max_abs) const
  {
    return (2.0 * this->Clamp01(value01) - 1.0) * max_abs;
  }

  void OnPx4MotorSpeed(const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> &_msg)
  {
    if (!_msg) {
      return;
    }

    if (_msg->motor_speed_size() < 4) {
      gzerr << "[RisingStarPlugin] PX4 motor_speed message has fewer than 4 channels: "
            << _msg->motor_speed_size() << std::endl;
      return;
    }

    const bool all_outputs_zero =
      std::fabs(_msg->motor_speed(0)) < 1e-6 &&
      std::fabs(_msg->motor_speed(1)) < 1e-6 &&
      std::fabs(_msg->motor_speed(2)) < 1e-6 &&
      std::fabs(_msg->motor_speed(3)) < 1e-6;

    const double ch0 = all_outputs_zero ? 0.0 :
      this->NormalizePx4MotorSpeedRange(_msg->motor_speed(0),
        this->px4_throttle_zero_raw_, this->px4_throttle_full_raw_);
    const double ch1 = all_outputs_zero ? 0.0 :
      this->NormalizePx4MotorSpeedRange(_msg->motor_speed(1),
        this->px4_collective_zero_raw_, this->px4_collective_full_raw_);
    const bool direct_blade_pitch = this->px4_pitch_input_mode_ == "blade_pitch";
    const double ch2 = all_outputs_zero ? (direct_blade_pitch ? 0.0 : 0.5) :
      this->NormalizePx4MotorSpeed(_msg->motor_speed(2));
    const double ch3 = all_outputs_zero ? (direct_blade_pitch ? 0.0 : 0.5) :
      this->NormalizePx4MotorSpeed(_msg->motor_speed(3));

    this->px4_engine_throttle_cmd_ = this->ClampEngineThrottle(direct_blade_pitch ? 0.5 * (ch0 + ch1) : ch0);

    if (direct_blade_pitch) {
      this->px4_blade1_pitch_deg_cmd_ =
        this->min_pitch_deg_ + ch2 * (this->max_pitch_deg_ - this->min_pitch_deg_);
      this->px4_blade2_pitch_deg_cmd_ =
        this->min_pitch_deg_ + ch3 * (this->max_pitch_deg_ - this->min_pitch_deg_);
      this->px4_collective_deg_cmd_ = 0.5 *
        (this->px4_blade1_pitch_deg_cmd_ + this->px4_blade2_pitch_deg_cmd_);
      this->px4_roll_cyclic_deg_cmd_ = 0.0;
      this->px4_pitch_cyclic_deg_cmd_ = 0.0;

    } else {
      this->px4_collective_deg_cmd_ =
        this->min_collective_cmd_deg_
        + ch1 * (this->max_collective_cmd_deg_ - this->min_collective_cmd_deg_);
      this->px4_roll_cyclic_deg_cmd_ =
        this->Map01ToSigned(ch2, this->max_cyclic_deg_);
      this->px4_pitch_cyclic_deg_cmd_ =
        this->Map01ToSigned(ch3, this->max_cyclic_deg_);
    }

    this->last_px4_command_time_ = this->model_->GetWorld()->SimTime();
    this->has_px4_command_ = true;

    const double debug_interval = std::max(0.02, this->px4_input_debug_interval_sec_);
    if (this->print_px4_input_debug_ &&
        (this->last_px4_command_time_ - this->last_px4_print_time_).Double() > debug_interval) {
      std::cout << "[RisingStarPlugin][PX4 INPUT] "
                << "raw=["
                << _msg->motor_speed(0) << ", "
                << _msg->motor_speed(1) << ", "
                << _msg->motor_speed(2) << ", "
                << _msg->motor_speed(3) << "] "
                << "norm=["
                << ch0 << ", "
                << ch1 << ", "
                << ch2 << ", "
                << ch3 << "] "
                << "cmd={"
                << "throttle:" << this->px4_engine_throttle_cmd_
                << ", pitch_mode:" << this->px4_pitch_input_mode_;

      if (direct_blade_pitch) {
        std::cout << ", blade_pitch_deg:[" << this->px4_blade1_pitch_deg_cmd_
                  << ", " << this->px4_blade2_pitch_deg_cmd_ << "]";

      } else {
        std::cout << ", collective_deg:" << this->px4_collective_deg_cmd_
                  << ", roll_cyclic_deg:" << this->px4_roll_cyclic_deg_cmd_
                  << ", pitch_cyclic_deg:" << this->px4_pitch_cyclic_deg_cmd_;
      }

      std::cout
                << "}"
                << std::endl;

      this->last_px4_print_time_ = this->last_px4_command_time_;
    }
  }

  bool Px4CommandActive(const common::Time &now) const
  {
    if (!this->enable_px4_actuator_input_ || !this->has_px4_command_) {
      return false;
    }

    return (now - this->last_px4_command_time_).Double() <= this->px4_command_timeout_sec_;
  }

  void ApplyPx4CommandsIfActive(const common::Time &now)
  {
    if (this->visual_inspection_mode_) {
      this->engine_throttle_ = 0.0;
      this->collective_deg_ = this->ClampPitchDeg(this->visual_inspection_blade_pitch_deg_);
      this->blade1_pitch_deg_ = this->collective_deg_;
      this->blade2_pitch_deg_ = this->collective_deg_;
      this->roll_cyclic_deg_ = 0.0;
      this->pitch_cyclic_deg_ = 0.0;
      this->use_cyclic_pitch_ = false;
      return;
    }

    if (!this->Px4CommandActive(now)) {
      if (this->enable_px4_actuator_input_) {
        this->engine_throttle_ = 0.0;
        this->collective_deg_ = this->min_collective_cmd_deg_;
        this->blade1_pitch_deg_ = this->min_pitch_deg_;
        this->blade2_pitch_deg_ = this->min_pitch_deg_;
        this->roll_cyclic_deg_ = 0.0;
        this->pitch_cyclic_deg_ = 0.0;
        this->use_cyclic_pitch_ = this->px4_pitch_input_mode_ != "blade_pitch";
      }
      return;
    }

    if (this->px4_pitch_input_mode_ == "blade_pitch") {
      this->engine_throttle_ = this->px4_engine_throttle_cmd_;
      this->blade1_pitch_deg_ = this->ClampPitchDeg(this->px4_blade1_pitch_deg_cmd_);
      this->blade2_pitch_deg_ = this->ClampPitchDeg(this->px4_blade2_pitch_deg_cmd_);
      this->collective_deg_ = this->ClampPitchDeg(0.5 * (this->blade1_pitch_deg_ + this->blade2_pitch_deg_));
      this->roll_cyclic_deg_ = 0.0;
      this->pitch_cyclic_deg_ = 0.0;
      this->use_cyclic_pitch_ = false;
      return;
    }

    double roll_cyclic_cmd = this->px4_roll_cyclic_deg_cmd_;
    double pitch_cyclic_cmd = this->px4_pitch_cyclic_deg_cmd_;

    if (this->swap_cyclic_inputs_) {
      std::swap(roll_cyclic_cmd, pitch_cyclic_cmd);
    }

    roll_cyclic_cmd *= this->roll_cyclic_sign_ >= 0.0 ? 1.0 : -1.0;
    pitch_cyclic_cmd *= this->pitch_cyclic_sign_ >= 0.0 ? 1.0 : -1.0;

    if (this->cyclic_command_frame_ == "world") {
#if GAZEBO_MAJOR_VERSION >= 8
      const ignition::math::Pose3d base_pose = this->base_link_->WorldPose();
      const ignition::math::Vector3d cyclic_world(roll_cyclic_cmd, pitch_cyclic_cmd, 0.0);
      const ignition::math::Vector3d cyclic_body = base_pose.Rot().RotateVectorReverse(cyclic_world);
      pitch_cyclic_cmd = cyclic_body.X();
      roll_cyclic_cmd = cyclic_body.Y();
#else
      const gazebo::math::Pose base_pose = this->base_link_->GetWorldPose();
      const gazebo::math::Vector3 cyclic_world(roll_cyclic_cmd, pitch_cyclic_cmd, 0.0);
      const gazebo::math::Vector3 cyclic_body = base_pose.rot.RotateVectorReverse(cyclic_world);
      pitch_cyclic_cmd = cyclic_body.x;
      roll_cyclic_cmd = cyclic_body.y;
#endif
    }

    this->engine_throttle_ = this->px4_engine_throttle_cmd_;
    this->collective_deg_ = this->ClampPitchDeg(this->px4_collective_deg_cmd_);
    this->roll_cyclic_deg_ = std::max(-this->max_cyclic_deg_,
      std::min(roll_cyclic_cmd, this->max_cyclic_deg_));
    this->pitch_cyclic_deg_ = std::max(-this->max_cyclic_deg_,
      std::min(pitch_cyclic_cmd, this->max_cyclic_deg_));
    this->use_cyclic_pitch_ = true;
  }

  void PrintFactSheet() const
  {
    gzmsg << "\n"
          << "================ Rising Star GP-76 CLEAN ================\n"
          << "Engine dynamics: " << (this->use_engine_dynamics_ ? "on" : "off") << "\n"
          << "Engine radius: " << this->engine_radius_ft_ << " ft\n"
          << "Engine thrust each: " << this->engine_thrust_lb_each_ << " lbf\n"
          << "Num engines: " << this->num_engines_ << "\n"
          << "Max drive torque: " << this->max_drive_torque_nm_
          << " N*m / " << this->max_drive_torque_nm_ / 1.3558179483314 << " ft-lb\n"
          << "Engine available rotor power: " << this->engine_available_power_w_
          << " W / " << this->engine_available_power_w_ / HpToW(1.0) << " hp\n"
          << "kTorque: " << this->k_torque_ << " N*m/(rad/s)^2\n"
          << "Rotor inertia: " << this->rotor_inertia_kgm2_ << " kg*m^2\n"
          << "Blade element lift: " << (this->use_blade_element_lift_ ? "on" : "off")
          << ", sections: " << this->blade_element_sections_
          << ", scale: " << this->blade_element_lift_scale_ << "\n"
          << "Physical payload: " << (this->payload_enabled_ ? "on" : "off")
          << ", mass: " << (this->payload_enabled_ ? this->payload_mass_lb_ : 0.0) << " lb"
          << ", offset from CoG: [" << this->payload_offset_from_cog_m_.X()
          << ", " << this->payload_offset_from_cog_m_.Y()
          << ", " << this->payload_offset_from_cog_m_.Z() << "] m\n"
          << "Fixed blade pitches: [" << this->blade1_pitch_deg_ << ", "
          << this->blade2_pitch_deg_ << "] deg\n"
          << "Cyclic pitch: " << (this->use_cyclic_pitch_ ? "on" : "off")
          << ", collective: " << this->collective_deg_
          << ", roll cyclic: " << this->roll_cyclic_deg_
          << ", pitch cyclic: " << this->pitch_cyclic_deg_
          << ", phase: " << this->cyclic_phase_offset_deg_ << " deg\n"
          << "Cyclic azimuth: source=" << this->cyclic_azimuth_source_
          << ", zero offset: " << this->cyclic_azimuth_zero_offset_rad_
          << " rad, direction: " << this->cyclic_azimuth_direction_ << "\n"
          << "Cyclic command frame: " << this->cyclic_command_frame_ << "\n"
          << "Cyclic input tuning: swap=" << (this->swap_cyclic_inputs_ ? "true" : "false")
          << ", roll sign=" << (this->roll_cyclic_sign_ >= 0.0 ? 1 : -1)
          << ", pitch sign=" << (this->pitch_cyclic_sign_ >= 0.0 ? 1 : -1) << "\n"
          << "Teeter dynamics: " << (this->enable_teeter_dynamics_ ? "on" : "off")
          << ", limit: " << this->teeter_angle_limit_deg_
          << " deg, inertia: " << this->teeter_inertia_kgm2_
          << " kg*m^2, damping: " << this->teeter_damping_nm_per_rad_s_
          << " N*m/(rad/s), stiffness: " << this->teeter_stiffness_nm_per_rad_
          << " N*m/rad\n"
          << "Pre-cone: " << this->pre_cone_angle_deg_ << " deg, force point y/z: "
          << this->blade_lift_span_m_ << " / " << this->blade_lift_z_m_ << " m\n"
          << "PX4 bridge: " << (this->enable_px4_actuator_input_ ? "on" : "off")
          << ", topic: " << this->px4_actuator_topic_
          << ", scale: " << this->px4_motor_speed_scale_ << "\n"
          << "Visual inspection mode: " << (this->visual_inspection_mode_ ? "on" : "off")
          << ", blade pitch: " << this->visual_inspection_blade_pitch_deg_
          << " deg, rotor azimuth: " << this->visual_inspection_rotor_azimuth_deg_
          << " deg, teeter: " << this->visual_inspection_teeter_deg_
          << " deg, blade visual flight update: "
          << (this->update_blade_pitch_visuals_during_flight_ ? "on" : "off") << "\n"
          << "PX4 mapping: ch0 engine A, ch1 engine B, ch2 blade 1 pitch, ch3 blade 2 pitch\n"
          << "==========================================================\n"
          << std::endl;
  }

  void OnUpdate()
  {
    if (!this->base_link_ || !this->rotor_link_ || !this->teeter_beam_link_ ||
        !this->joint_ || !this->teeter_hinge_joint_) {
      return;
    }

    const common::Time now = this->model_->GetWorld()->SimTime();
    this->UpdateBladeVisualMode();
    this->ApplyPx4CommandsIfActive(now);
    const double omega_cmd = this->visual_inspection_mode_ ? 0.0 : this->StepRotorOmega(now);

    if (this->visual_inspection_mode_) {
      this->rotor_azimuth_rad_ = DegToRad(this->visual_inspection_rotor_azimuth_deg_);
#if GAZEBO_MAJOR_VERSION >= 6
      this->joint_->SetPosition(0, this->rotor_azimuth_rad_);
      this->teeter_hinge_joint_->SetPosition(0, DegToRad(this->visual_inspection_teeter_deg_));
#else
      this->joint_->SetAngle(0, this->rotor_azimuth_rad_);
      this->teeter_hinge_joint_->SetAngle(0, DegToRad(this->visual_inspection_teeter_deg_));
#endif
    } else {
      this->StepRotorAzimuth(now, omega_cmd);
    }

    double blade1_pitch_cmd_deg = this->blade1_pitch_deg_;
    double blade2_pitch_cmd_deg = this->blade2_pitch_deg_;
    this->CurrentBladePitchesDeg(blade1_pitch_cmd_deg, blade2_pitch_cmd_deg);

    if (this->visual_inspection_mode_ || this->update_blade_pitch_visuals_during_flight_) {
      this->SetBladeVisualPitch(blade1_pitch_cmd_deg, blade2_pitch_cmd_deg);
    }

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d angular_vel(0.0, 0.0, omega_cmd);
#else
    gazebo::math::Vector3 angular_vel(0.0, 0.0, omega_cmd);
#endif

    this->rotor_link_->SetAngularVel(angular_vel);
    const double blade1_lift_n = this->BladeLiftNewton(omega_cmd, blade1_pitch_cmd_deg);
    const double blade2_lift_n = this->BladeLiftNewton(omega_cmd, blade2_pitch_cmd_deg);
    const double total_lift_n = blade1_lift_n + blade2_lift_n;
    this->teeter_angle_rad_ = this->TeeterHingeAngleRad();
    this->teeter_rate_rad_s_ = this->teeter_hinge_joint_->GetVelocity(0);

#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d teeter_pose = this->teeter_beam_link_->WorldPose();
    const ignition::math::Quaterniond rotor_disk_to_world = teeter_pose.Rot();

    const ignition::math::Vector3d blade1_local(0.0, -this->blade_lift_span_m_, this->blade_lift_z_m_);
    const ignition::math::Vector3d blade2_local(0.0, this->blade_lift_span_m_, this->blade_lift_z_m_);

    const ignition::math::Vector3d blade1_pos_world =
      teeter_pose.Pos() + rotor_disk_to_world.RotateVector(blade1_local);
    const ignition::math::Vector3d blade2_pos_world =
      teeter_pose.Pos() + rotor_disk_to_world.RotateVector(blade2_local);

    const ignition::math::Vector3d blade1_force_world =
      rotor_disk_to_world.RotateVector(ignition::math::Vector3d(0.0, 0.0, blade1_lift_n));
    const ignition::math::Vector3d blade2_force_world =
      rotor_disk_to_world.RotateVector(ignition::math::Vector3d(0.0, 0.0, blade2_lift_n));

#else
    const gazebo::math::Pose teeter_pose = this->teeter_beam_link_->GetWorldPose();
    const gazebo::math::Quaternion rotor_disk_to_world = teeter_pose.rot;

    const gazebo::math::Vector3 blade1_local(0.0, -this->blade_lift_span_m_, this->blade_lift_z_m_);
    const gazebo::math::Vector3 blade2_local(0.0, this->blade_lift_span_m_, this->blade_lift_z_m_);

    const gazebo::math::Vector3 blade1_pos_world =
      teeter_pose.pos + rotor_disk_to_world.RotateVector(blade1_local);
    const gazebo::math::Vector3 blade2_pos_world =
      teeter_pose.pos + rotor_disk_to_world.RotateVector(blade2_local);

    const gazebo::math::Vector3 blade1_force_world =
      rotor_disk_to_world.RotateVector(gazebo::math::Vector3(0.0, 0.0, blade1_lift_n));
    const gazebo::math::Vector3 blade2_force_world =
      rotor_disk_to_world.RotateVector(gazebo::math::Vector3(0.0, 0.0, blade2_lift_n));

#endif

    this->teeter_beam_link_->AddForceAtWorldPosition(blade1_force_world, blade1_pos_world);
    this->teeter_beam_link_->AddForceAtWorldPosition(blade2_force_world, blade2_pos_world);

    const double state_debug_interval = std::max(0.02, this->teeter_state_debug_interval_sec_);
    if (this->print_teeter_state_debug_ &&
        (now - this->last_print_time_).Double() > state_debug_interval) {
      const double rpm = RadPerSecToRpm(omega_cmd);
      const double net_n = total_lift_n - this->physical_payload_weight_n_
        - this->EmptyWeightNewton();

      std::cout << "[RisingStarPlugin] rpm = "
                << rpm
                << ", payload = " << (this->payload_enabled_ ? this->payload_mass_lb_ : 0.0) << " lb"
                << ", azimuth = " << this->CyclicRotorAzimuthRad() * 180.0 / M_PI
                << " deg (" << this->cyclic_azimuth_source_ << ")"
                << ", blade pitches = [" << blade1_pitch_cmd_deg << ", " << blade2_pitch_cmd_deg << "] deg"
                << ", teeter = " << this->teeter_angle_rad_ * 180.0 / M_PI
                << " deg, teeter_rate = " << this->teeter_rate_rad_s_ * 180.0 / M_PI << " deg/s"
                << ", cyclic = [" << this->roll_cyclic_deg_ << ", " << this->pitch_cyclic_deg_ << "] deg"
                << ", total lift = " << total_lift_n << " N"
                << ", net = " << net_n << " N"
                << ", drive torque = " << this->DriveTorqueNewtonMeter(omega_cmd) << " N*m"
                << ", drag torque = " << this->DragTorqueNewtonMeter(omega_cmd) << " N*m"
                << std::endl;

      this->last_print_time_ = now;
    }
  }

private:
  // PX4 actuator bridge.
  bool enable_px4_actuator_input_{true};
  std::string px4_actuator_topic_{"~/command/motor_speed"};
  std::string px4_input_mode_{"motor_speed"};
  std::string px4_pitch_input_mode_{"cyclic"};
  double px4_motor_speed_scale_{1000.0};
  double px4_throttle_zero_raw_{500.0};
  double px4_throttle_full_raw_{1000.0};
  double px4_collective_zero_raw_{500.0};
  double px4_collective_full_raw_{1000.0};
  double min_collective_cmd_deg_{2.0};
  double max_collective_cmd_deg_{12.0};
  double max_cyclic_deg_{2.0};
  double px4_command_timeout_sec_{0.5};
  bool print_px4_input_debug_{false};
  double px4_input_debug_interval_sec_{0.2};
  bool print_teeter_state_debug_{false};
  double teeter_state_debug_interval_sec_{1.0};

  bool visual_inspection_mode_{false};
  double visual_inspection_blade_pitch_deg_{5.0};
  double visual_inspection_rotor_azimuth_deg_{0.0};
  double visual_inspection_teeter_deg_{0.0};
  bool update_blade_pitch_visuals_during_flight_{false};

  bool has_px4_command_{false};
  double px4_engine_throttle_cmd_{0.0};
  double px4_collective_deg_cmd_{7.27};
  double px4_roll_cyclic_deg_cmd_{0.0};
  double px4_pitch_cyclic_deg_cmd_{0.0};
  double px4_blade1_pitch_deg_cmd_{7.27};
  double px4_blade2_pitch_deg_cmd_{7.27};
  common::Time last_px4_command_time_{0};
  common::Time last_px4_print_time_{0};

  transport::NodePtr node_;
  transport::SubscriberPtr px4_motor_speed_sub_;
  transport::PublisherPtr payload_visual_pub_;

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  physics::LinkPtr rotor_link_;
  physics::LinkPtr teeter_beam_link_;
  physics::LinkPtr blade1_pitch_link_;
  physics::LinkPtr blade2_pitch_link_;
  physics::LinkPtr payload_link_;
  physics::JointPtr joint_;
  physics::JointPtr teeter_hinge_joint_;
  event::ConnectionPtr update_connection_;

  // Primary variables.
  double target_rpm_{0.0};
  bool payload_enabled_{true};
  double payload_mass_lb_{360.0};
  double disabled_payload_mass_kg_{0.5};
  ignition::math::Vector3d payload_offset_from_cog_m_{0.0, 0.0, 0.194};
  double payload_outer_diameter_in_{17.375};
  double payload_inner_diameter_in_{1.0};
  double payload_thickness_in_{10.0};
  double payload_disabled_scale_{0.08};
  double blade1_pitch_deg_{7.27};
  double blade2_pitch_deg_{7.27};

  // Azimuth-based cyclic pitch.
  bool use_cyclic_pitch_{true};
  double collective_deg_{7.27};
  double roll_cyclic_deg_{0.0};
  double pitch_cyclic_deg_{0.0};
  double cyclic_phase_offset_deg_{0.0};
  std::string cyclic_azimuth_source_{"joint"};
  double cyclic_azimuth_zero_offset_rad_{0.0};
  double cyclic_azimuth_direction_{1.0};
  std::string cyclic_command_frame_{"body"};
  bool swap_cyclic_inputs_{false};
  double roll_cyclic_sign_{1.0};
  double pitch_cyclic_sign_{1.0};
  bool enable_teeter_dynamics_{false};
  double teeter_inertia_kgm2_{80.0};
  double teeter_damping_nm_per_rad_s_{600.0};
  double teeter_stiffness_nm_per_rad_{0.0};
  double teeter_moment_sign_{1.0};
  double teeter_rate_limit_deg_s_{180.0};
  double teeter_angle_rad_{0.0};
  double teeter_rate_rad_s_{0.0};
  bool teeter_time_initialized_{false};
  common::Time last_teeter_update_time_{0};
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
  double engine_available_power_w_{10141.5};
  double rotor_omega_{0.0};
  bool rotor_time_initialized_{false};
  common::Time last_rotor_update_time_{0};

  // Fixed constants.
  double empty_weight_lb_{42.0};
  double k_thrust_{3.69};
  double max_rpm_{220.0};

  // Pitch model.
  double hover_pitch_deg_{7.27};
  double zero_lift_pitch_deg_{1.0};
  double min_pitch_deg_{2.0};
  double max_pitch_deg_{12.0};
  bool use_blade_element_lift_{false};
  double air_density_kgm3_{1.225};
  double blade_lift_slope_per_rad_{6.283185307179586};
  int blade_element_sections_{12};
  double blade_element_lift_scale_{0.0};

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
  double physical_payload_weight_n_{1601.36};
  double current_gross_weight_lb_{402.0};
  double current_gross_weight_n_{1788.18};

  double rotor_radius_m_{3.6576};
  double hub_radius_m_{0.9144};
  double blade_chord_m_{0.2667};
  double blade_effective_span_m_{2.7432};
  double blade_lift_radius_m_{2.286};
  double blade_lift_span_m_{2.2600};
  double blade_lift_z_m_{0.2617};
  double pre_cone_rad_{0.191986};
  double disk_area_m2_{42.04};
  double solidity_{0.0464};

  common::Time last_print_time_{0};
};

GZ_REGISTER_MODEL_PLUGIN(RisingStarPlugin)
}
