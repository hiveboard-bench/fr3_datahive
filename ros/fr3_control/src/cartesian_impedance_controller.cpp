#include <fr3_control/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <fr3_control/pseudo_inversion.h>

namespace fr3_control
{

  bool CartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw,
                                          ros::NodeHandle &node_handle)
  {
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
      return false;
    }
    // Frame the published setpoint and error are expressed in. The controller works
    // entirely in the robot base frame, so this only labels the messages.
    node_handle.param<std::string>("base_frame", base_frame_, arm_id + "_link0");
    int decimation = 10;
    node_handle.param<int>("diagnostic_decimation", decimation, 10);
    diagnostic_decimation_ = decimation > 0 ? static_cast<uint32_t>(decimation) : 1;

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("CartesianImpedanceController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting joint handles: "
                         << ex.what());
        return false;
      }
    }

    sub_equilibrium_pose_ = node_handle.subscribe(
        "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    nullspace_reference_service_ = node_handle.advertiseService(
        "set_nullspace_reference", &CartesianImpedanceController::setNullspaceReference, this);

    publisher_jacobian_.init(node_handle, "franka_jacobian", 1);
    publisher_commanded_pose_.init(node_handle, "commanded_pose", 1);
    publisher_pose_error_.init(node_handle, "pose_error", 1);

    // The "/" is deliberately absent, matching serl, so an existing
    // dynamic_reconfigure client or launch file that seeds gains onto the param
    // server keeps working against this fork without being rewritten. See
    dynamic_reconfigure_compliance_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ =
        std::make_unique<dynamic_reconfigure::Server<fr3_control::compliance_paramConfig>>(
            dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    Ki_.setZero();
    error_.setZero();
    error_i_.setZero();

    return true;
  }

  void CartesianImpedanceController::starting(const ros::Time & /*time*/)
  {
    franka::RobotState initial_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // Command the arm its own pose, so spawning this controller does not move it.
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    position_d_target_ = position_d_;
    orientation_d_target_ = orientation_d_;

    // Overwrites anything a subscriber delivered between init() and here, which is
    // what we want: on start the arm should be told to stay exactly where it is.
    TargetPose initial{position_d_.x(), position_d_.y(), position_d_.z(),
                       orientation_d_.x(), orientation_d_.y(), orientation_d_.z(),
                       orientation_d_.w()};
    target_pose_buffer_.initRT(initial);

    // The redundant elbow is pulled toward this configuration. Snapshotting it here
    // means it is the pose at *controller spawn*; the operator then hand-guides the
    // arm somewhere else before engaging, which is what leaves it stale. Call
    // ~set_nullspace_reference at engage to re-anchor it.
    q_d_nullspace_ = q_initial;
    error_i_.setZero();
    reanchor_nullspace_.store(false, std::memory_order_relaxed);
  }

  void CartesianImpedanceController::update(const ros::Time &time, const ros::Duration &period)
  {
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    jacobian_array_ = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array_.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // Re-anchoring happens here rather than in the service callback so the write to
    // q_d_nullspace_ stays on this thread and needs no lock.
    if (reanchor_nullspace_.exchange(false, std::memory_order_acq_rel))
    {
      q_d_nullspace_ = q;
    }

    // One lock-free read per tick. serl instead lets the subscriber thread write
    // these straight into the members this loop is reading.
    const TargetPose &target = *target_pose_buffer_.readFromRT();
    position_d_target_ << target[0], target[1], target[2];
    orientation_d_target_.coeffs() << target[3], target[4], target[5], target[6];

    // Translational error, before clipping.
    Eigen::Matrix<double, 6, 1> raw_error;
    raw_error.head(3) << position - position_d_;

    // Orientation error as the vector part of the difference quaternion, rotated
    // into the base frame to match the translational part.
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    raw_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    raw_error.tail(3) << -transform.linear() * raw_error.tail(3);

    // Clip before the error becomes torque. This is the property the whole
    // controller choice rests on: the proportional wrench can never exceed
    // stiffness x clip, independently of how far the setpoint has run away.
    std::array<bool, 6> saturated{};
    error_ = raw_error;
    for (int i = 0; i < 3; i++)
    {
      error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
      error_(i + 3) =
          std::min(std::max(error_(i + 3), rotational_clip_min_(i)), rotational_clip_max_(i));
      saturated[i] = error_(i) != raw_error(i);
      saturated[i + 3] = error_(i + 3) != raw_error(i + 3);
    }

    // Scaled by the real timestep, so Ki is per second. serl accumulates once per
    // tick with no dt, which at 1 kHz makes the same number act 1000x stronger.
    //
    // Also note what is NOT here: serl zeroes this integrator inside
    // equilibriumPoseCallback, on every incoming setpoint. Under teleop that
    // arrives at 20 Hz, so its integrator never accumulates more than 50 ms and can
    // never remove a steady-state offset, which is the only thing an integral term
    // is for. This fork integrates continuously and relies on the clamps below.
    //
    // SAFETY: the clamps are the only bound on this term, because Ki acts on the
    // integrated error and NOT on the clipped error. The measured "force ceiling =
    // stiffness x clip" holds exactly while Ki is 0 (the default); switching Ki on
    // raises the real ceiling to K*clip + Ki*clamp. At the cfg maximum that is
    // 20 N + 100*0.1 = 30 N, not 20 N. Re-measure the ceiling if Ki is ever raised.
    const double dt = period.toSec();
    error_i_.head(3) =
        (error_i_.head(3) + error_.head(3) * dt).cwiseMax(-0.1).cwiseMin(0.1);
    error_i_.tail(3) =
        (error_i_.tail(3) + error_.tail(3) * dt).cwiseMax(-0.3).cwiseMin(0.3);

    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error_ -
                                        cartesian_damping_ * (jacobian * dq) - Ki_ * error_i_);

    // Per-joint absolute stiffness, critically damped at 2*sqrt(k). serl instead
    // multiplies joint 1's gain into the shared one and then damps the product
    // twice over, so its joint 1 ends up at roughly twice critical damping. Keeping
    // the two stiffnesses independent is the point of 1.4; the damping follows from
    // whichever stiffness each joint actually has.
    Eigen::Matrix<double, 7, 1> k_nullspace;
    k_nullspace.setConstant(nullspace_stiffness_);
    k_nullspace(0) = joint1_nullspace_stiffness_;

    Eigen::Matrix<double, 7, 1> joint_error =
        (q_d_nullspace_ - q).cwiseMax(-nullspace_clip_).cwiseMin(nullspace_clip_);
    Eigen::Matrix<double, 7, 1> tau_nullspace_raw;
    for (int i = 0; i < 7; i++)
    {
      tau_nullspace_raw(i) =
          k_nullspace(i) * joint_error(i) - 2.0 * std::sqrt(k_nullspace(i)) * dq(i);
    }
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         tau_nullspace_raw;

    // Clamped after the projection, not before, so the bound holds on the torque
    // actually commanded. A clamp that binds does leak out of the nullspace, but
    // the projector already leaks anyway: pseudoInverse is damped at lambda=0.2,
    // so this is never an exact projector to begin with.
    tau_nullspace =
        tau_nullspace.cwiseMax(-nullspace_torque_max_).cwiseMin(nullspace_torque_max_);

    tau_d << tau_task + tau_nullspace + coriolis;
    tau_d << saturateTorqueRate(tau_d, tau_J_d);

    // Nothing upstream can produce this now that the setpoint is validated, but a
    // NaN reaching setCommand trips a libfranka reflex and drops the episode; hold
    // the previous torque instead.
    if (!tau_d.allFinite())
    {
      ROS_ERROR_THROTTLE(1.0, "CartesianImpedanceController: non-finite torque, holding last command");
    }
    else
    {
      for (size_t i = 0; i < 7; ++i)
      {
        joint_handles_[i].setCommand(tau_d(i));
      }
    }

    // Gains ramp on their own coefficient so that changing tracking bandwidth does
    // not also change how abruptly a reconfigure lands.
    cartesian_stiffness_ =
        gain_filter_ * cartesian_stiffness_target_ + (1.0 - gain_filter_) * cartesian_stiffness_;
    cartesian_damping_ =
        gain_filter_ * cartesian_damping_target_ + (1.0 - gain_filter_) * cartesian_damping_;
    Ki_ = gain_filter_ * Ki_target_ + (1.0 - gain_filter_) * Ki_;
    nullspace_stiffness_ =
        gain_filter_ * nullspace_stiffness_target_ + (1.0 - gain_filter_) * nullspace_stiffness_;
    joint1_nullspace_stiffness_ = gain_filter_ * joint1_nullspace_stiffness_target_ +
                                  (1.0 - gain_filter_) * joint1_nullspace_stiffness_;

    // Re-anchoring at engage only zeroes this error once; the arm then moves and it
    // grows back, which is why a one-shot reset never held. Leaking the reference
    // continuously turns the nullspace spring into a damper on elbow drift, and the
    // error can no longer accumulate past what the leak time constant admits.
    q_d_nullspace_ += nullspace_leak_ * (q - q_d_nullspace_);

    // The dominant lag term in the stack. tau = -dt/ln(1-setpoint_filter_).
    position_d_ = setpoint_filter_ * position_d_target_ + (1.0 - setpoint_filter_) * position_d_;
    orientation_d_ = orientation_d_.slerp(setpoint_filter_, orientation_d_target_);

    publishDiagnostics(time, raw_error, saturated);
  }

  void CartesianImpedanceController::publishDiagnostics(const ros::Time &time,
                                                        const Eigen::Matrix<double, 6, 1> &raw_error,
                                                        const std::array<bool, 6> &saturated)
  {
    if (++tick_ % diagnostic_decimation_ != 0)
    {
      return;
    }

    if (publisher_jacobian_.trylock())
    {
      for (size_t i = 0; i < jacobian_array_.size(); i++)
      {
        publisher_jacobian_.msg_.zero_jacobian[i] = jacobian_array_[i];
      }
      publisher_jacobian_.unlockAndPublish();
    }

    // The setpoint the plant actually tracked this tick, after filtering -- not the
    // target that was published at it. This is what a recorded action should carry.
    if (publisher_commanded_pose_.trylock())
    {
      auto &msg = publisher_commanded_pose_.msg_;
      msg.header.stamp = time;
      msg.header.frame_id = base_frame_;
      msg.pose.position.x = position_d_.x();
      msg.pose.position.y = position_d_.y();
      msg.pose.position.z = position_d_.z();
      msg.pose.orientation.x = orientation_d_.x();
      msg.pose.orientation.y = orientation_d_.y();
      msg.pose.orientation.z = orientation_d_.z();
      msg.pose.orientation.w = orientation_d_.w();
      publisher_commanded_pose_.unlockAndPublish();
    }

    if (publisher_pose_error_.trylock())
    {
      auto &msg = publisher_pose_error_.msg_;
      msg.header.stamp = time;
      msg.header.frame_id = base_frame_;
      for (size_t i = 0; i < 3; i++)
      {
        msg.translation[i] = raw_error(i);
        msg.rotation[i] = raw_error(i + 3);
      }
      for (size_t i = 0; i < 6; i++)
      {
        msg.saturated[i] = saturated[i];
      }
      publisher_pose_error_.unlockAndPublish();
    }
  }

  Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void CartesianImpedanceController::complianceParamCallback(
      fr3_control::compliance_paramConfig &config, uint32_t /*level*/)
  {
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << config.translational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << config.rotational_stiffness * Eigen::Matrix3d::Identity();

    cartesian_damping_target_.setIdentity();
    cartesian_damping_target_.topLeftCorner(3, 3)
        << config.translational_damping * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << config.rotational_damping * Eigen::Matrix3d::Identity();

    Ki_target_.setIdentity();
    Ki_target_.topLeftCorner(3, 3) << config.translational_Ki * Eigen::Matrix3d::Identity();
    Ki_target_.bottomRightCorner(3, 3) << config.rotational_Ki * Eigen::Matrix3d::Identity();

    nullspace_stiffness_target_ = config.nullspace_stiffness;
    joint1_nullspace_stiffness_target_ = config.joint1_nullspace_stiffness;
    nullspace_leak_ = config.nullspace_leak;
    nullspace_clip_ = config.nullspace_clip;
    nullspace_torque_max_ = config.nullspace_torque_max;

    translational_clip_min_ << -config.translational_clip_neg_x, -config.translational_clip_neg_y,
        -config.translational_clip_neg_z;
    translational_clip_max_ << config.translational_clip_x, config.translational_clip_y,
        config.translational_clip_z;
    rotational_clip_min_ << -config.rotational_clip_neg_x, -config.rotational_clip_neg_y,
        -config.rotational_clip_neg_z;
    rotational_clip_max_ << config.rotational_clip_x, config.rotational_clip_y,
        config.rotational_clip_z;

    setpoint_filter_ = config.setpoint_filter;
    gain_filter_ = config.gain_filter;
  }

  void CartesianImpedanceController::equilibriumPoseCallback(
      const geometry_msgs::PoseStampedConstPtr &msg)
  {
    // No quaternion sign handling here, unlike serl. q and -q are the same
    // rotation, and serl flips the incoming one to stay continuous with the last
    // command so that slerp does not take the long way round. Eigen already does
    // exactly that internally -- Quaternion.h's slerp ends with
    // `if(d<Scalar(0)) scale1 = -scale1;` -- and this target is used nowhere but
    // that slerp. Keeping the flip would mean keeping a quaternion shared between
    // this thread and starting(), for no effect.
    TargetPose target{msg->pose.position.x, msg->pose.position.y,
                      msg->pose.position.z, msg->pose.orientation.x,
                      msg->pose.orientation.y, msg->pose.orientation.z,
                      msg->pose.orientation.w};

    // An uninitialised geometry_msgs/Quaternion is all zeros, and slerp turns that
    // into NaN torque a tick later. Reject rather than normalise: a pose that is
    // already valid must reach the buffer untouched.
    for (double value : target)
    {
      if (!std::isfinite(value))
      {
        ROS_WARN_THROTTLE(1.0, "CartesianImpedanceController: ignoring non-finite equilibrium pose");
        return;
      }
    }
    const double quaternion_norm =
        Eigen::Vector4d(target[3], target[4], target[5], target[6]).norm();
    if (std::abs(quaternion_norm - 1.0) > kQuaternionNormTolerance)
    {
      ROS_WARN_THROTTLE(1.0,
                        "CartesianImpedanceController: ignoring equilibrium pose whose "
                        "quaternion norm is %.6f",
                        quaternion_norm);
      return;
    }

    target_pose_buffer_.writeFromNonRT(target);
  }

  bool CartesianImpedanceController::setNullspaceReference(
      std_srvs::Trigger::Request & /*request*/, std_srvs::Trigger::Response &response)
  {
    reanchor_nullspace_.store(true, std::memory_order_release);
    response.success = true;
    response.message = "nullspace reference will be re-anchored on the next control tick";
    return true;
  }

} // namespace fr3_control

PLUGINLIB_EXPORT_CLASS(fr3_control::CartesianImpedanceController,
                       controller_interface::ControllerBase)
