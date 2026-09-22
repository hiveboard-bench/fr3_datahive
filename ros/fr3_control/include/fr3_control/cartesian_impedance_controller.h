#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/Trigger.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <fr3_control/PoseError.h>
#include <fr3_control/ZeroJacobian.h>
#include <fr3_control/compliance_paramConfig.h>

namespace fr3_control
{

    class CartesianImpedanceController
        : public controller_interface::MultiInterfaceController<
              franka_hw::FrankaModelInterface,
              hardware_interface::EffortJointInterface,
              franka_hw::FrankaStateInterface>
    {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;

    private:
        // Flat [x, y, z, qx, qy, qz, qw]. Deliberately not Eigen types: this crosses
        // threads through a RealtimeBuffer, and a plain array of doubles has no
        // alignment requirements to get wrong.
        using TargetPose = std::array<double, 7>;

        static constexpr double kQuaternionNormTolerance = 1e-3;

        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
            const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

        void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        bool setNullspaceReference(std_srvs::Trigger::Request &request,
                                   std_srvs::Trigger::Response &response);
        void complianceParamCallback(fr3_control::compliance_paramConfig &config, uint32_t level);
        void publishDiagnostics(const ros::Time &time,
                                const Eigen::Matrix<double, 6, 1> &raw_error,
                                const std::array<bool, 6> &saturated);

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        std::array<double, 42> jacobian_array_{};

        // libfranka rejects a torque step above 1 Nm per millisecond. This limits the
        // torque RATE, never its magnitude -- the magnitude ceiling comes from the
        // error clip times the stiffness.
        const double delta_tau_max_{1.0};

        // Two filters, both per 1 kHz tick. tau = -dt/ln(1-alpha).
        double setpoint_filter_{0.005};
        double gain_filter_{0.005};

        // Absolute, in Nm/rad. joint1_nullspace_stiffness_ is joint 1's own stiffness,
        // NOT a multiplier on nullspace_stiffness_ the way serl has it.
        double nullspace_stiffness_{0.2};
        double nullspace_stiffness_target_{0.2};
        double joint1_nullspace_stiffness_{20.0};
        double joint1_nullspace_stiffness_target_{20.0};

        // The nullspace reference leaks toward the measured configuration at this
        // coefficient per 1 kHz tick, which is what stops the joint error growing
        // without bound over an episode. tau = -dt/ln(1-alpha); 0 pins the reference
        // and restores the pure-anchor behaviour.
        double nullspace_leak_{0.0005};

        // Clip and hard ceiling for the nullspace term, in rad and Nm. tau_task is
        // bounded by stiffness x clip; without these the nullspace path is the one
        // route to the joints that no clip covers.
        double nullspace_clip_{0.2};
        double nullspace_torque_max_{5.0};

        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
        Eigen::Matrix<double, 6, 6> Ki_;
        Eigen::Matrix<double, 6, 6> Ki_target_;

        Eigen::Matrix<double, 3, 1> translational_clip_min_;
        Eigen::Matrix<double, 3, 1> translational_clip_max_;
        Eigen::Matrix<double, 3, 1> rotational_clip_min_;
        Eigen::Matrix<double, 3, 1> rotational_clip_max_;

        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Matrix<double, 6, 1> error_;
        Eigen::Matrix<double, 6, 1> error_i_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;

        // Written by the subscriber thread, read once per tick by update(). This is the
        // only state shared between the two, which is what makes the lock-free buffer
        // sufficient rather than a mutex.
        realtime_tools::RealtimeBuffer<TargetPose> target_pose_buffer_;

        // Set by the service thread, consumed and cleared inside update() so the
        // snapshot itself happens on the realtime thread.
        std::atomic<bool> reanchor_nullspace_{false};

        ros::Subscriber sub_equilibrium_pose_;
        ros::ServiceServer nullspace_reference_service_;

        std::unique_ptr<dynamic_reconfigure::Server<fr3_control::compliance_paramConfig>>
            dynamic_server_compliance_param_;
        ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

        realtime_tools::RealtimePublisher<fr3_control::ZeroJacobian> publisher_jacobian_;
        realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> publisher_commanded_pose_;
        realtime_tools::RealtimePublisher<fr3_control::PoseError> publisher_pose_error_;

        // Diagnostics are decimated rather than published every tick. Serialising three
        // messages at 1 kHz is real work on the realtime thread, and CPU pressure on
        // this workstation has already been measured corrupting the control loop
        // 100 Hz is far above what the 10 Hz recorder
        // and any human-facing plot need.
        uint32_t tick_{0};
        uint32_t diagnostic_decimation_{10};
        std::string base_frame_;
    };

} // namespace fr3_control
