#include <memory>
#include <thread>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ArmBenchScanController : public rclcpp::Node {
public:
    ArmBenchScanController() : Node("arm_controller") {
        // -------- params --------
        this->declare_parameter("bench_edge_x", 0.85);
        this->declare_parameter("bench_edge_margin", 0.05);

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // -------- pubs --------
        pub_status_ = this->create_publisher<std_msgs::msg::String>("/arm_scan_status", 10);
        pub_done_ = this->create_publisher<std_msgs::msg::Bool>("/arm_scan_done", 10);

        // -------- subs --------
        sub_auto_ = this->create_subscription<std_msgs::msg::String>("/auto_state", 10, std::bind(&ArmBenchScanController::auto_cb, this, std::placeholders::_1));

        // -------- timers --------
        timer_startup_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&ArmBenchScanController::init_moveit, this));
    }

private:
    void init_moveit() {
        timer_startup_->cancel();

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_->setPlanningTime(5.0);
        move_group_->allowReplanning(true);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);

        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized.");
        //RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        //RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());

        std::thread([this]() {
            rclcpp::sleep_for(std::chrono::seconds(2));  // wait for joint states
            RCLCPP_INFO(this->get_logger(), "Moving to home pose...");
            bool ok = move_named_pose("retracted");
            if (ok) {
                RCLCPP_INFO(this->get_logger(), "System Ready. Waiting for 'scan_start'...");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to 'retracted'.");
            }
        }).detach();
    }


    // -------- helpers --------
    void publish_status(const std::string& status) {
        std_msgs::msg::String msg;
        msg.data = status;
        pub_status_->publish(msg);
    }

    bool move_named_pose(const std::string& target) {

        if (!move_group_->setNamedTarget(target)) {
            RCLCPP_ERROR(this->get_logger(), "Named target '%s' not found.", target.c_str());
            return false;
        }

        move_group_->setStartStateToCurrentState();
        move_group_->setPlanningTime(5.0);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = move_group_->plan(plan);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Planning to named target '%s' failed.", target.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Plan to '%s' successful. Executing...", target.c_str());

        auto exec_result = move_group_->execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Execution to named target '%s' failed.", target.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Reached named target: %s", target.c_str());
        return true;
    }

    bool move_to_xyz(double x, double y, double z) {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.header.stamp = this->now();

        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;

        // Keep a fixed orientation for now
        target.pose.orientation.x = 0.0;
        target.pose.orientation.y = 0.0;
        target.pose.orientation.z = 0.0;
        target.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(),"Trying pose target: x=%.3f y=%.3f z=%.3f", x, y, z);

        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(target, "zed_camera_center");
        move_group_->setPlanningTime(5.0);
        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = move_group_->plan(plan);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Planning to pose target failed.");
            move_group_->clearPoseTargets();
            return false;
        }

        auto exec_result = move_group_->execute(plan);
        move_group_->clearPoseTargets();

        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Execution to pose target failed.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Reached pose target.");
        return true;
    }

    bool cartesian_scan(double x_start, double x_end, double y, double z)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;

        geometry_msgs::msg::Pose target = start_pose;

        target.position.x = x_start;
        target.position.y = y;
        target.position.z = z;

        waypoints.push_back(target);

        target.position.x = x_end;

        waypoints.push_back(target);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double eef_step = 0.01;   // 1 cm resolution
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory);

        RCLCPP_INFO(this->get_logger(), "Cartesian path success: %.2f%%", fraction * 100.0);

        if (fraction < 0.9)
            return false;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }


    // -------- callbacks --------
    void auto_cb(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "scan_start" && !busy_) {
            busy_ = true;
            std::thread(&ArmBenchScanController::run_scan_sequence, this).detach();
        }
    }


    // -------- main logic --------
    
    void run_scan_sequence() {
        publish_status("moving_to_scan_start");
        if (!move_named_pose("scan_start")) {
            publish_status("scan_failed");
            busy_ = false;
            return;
        }

        publish_status("test_scanning");

        cartesian_scan(0.3, 0.7, 0.5, 0.6);

        publish_status("returning_home");
        move_named_pose("retracted");

        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        pub_done_->publish(done_msg);
        publish_status("scan_done");
        busy_ = false;
    }

    

    // Members
    bool busy_ = false;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_done_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_auto_;
    rclcpp::TimerBase::SharedPtr timer_startup_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmBenchScanController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}