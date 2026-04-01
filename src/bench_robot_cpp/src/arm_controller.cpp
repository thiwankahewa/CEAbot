#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <shape_msgs/msg/solid_primitive.hpp>

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
        pub_trigger_ = this->create_publisher<std_msgs::msg::Int32>("/zed_capture_trigger", 10);

        // -------- subs --------
        sub_auto_ = this->create_subscription<std_msgs::msg::String>("/auto_state", 10, std::bind(&ArmBenchScanController::auto_cb, this, std::placeholders::_1));
        sub_capture_done_ = this->create_subscription<std_msgs::msg::Int32>("/zed_capture_done", 10,std::bind(&ArmBenchScanController::capture_done_cb, this, std::placeholders::_1));
        sub_map_ready_ = this->create_subscription<std_msgs::msg::Bool>("/zed_local_map_ready", 10,std::bind(&ArmBenchScanController::map_ready_cb, this, std::placeholders::_1));

        // -------- timers --------
        timer_startup_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ArmBenchScanController::init_moveit, this));
    }

private:
    struct ViewPose
        {
            double x;
            double y;
            double z;
            double qx;
            double qy;
            double qz;
            double qw;

        };

    void init_moveit() {
        timer_startup_->cancel();

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_->setPlanningTime(5.0);
        move_group_->allowReplanning(true);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);

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

    void trigger_capture(int capture_id)
    {
        std_msgs::msg::Int32 msg;
        msg.data = capture_id;
        pub_trigger_->publish(msg);
    }

    bool move_named_pose(const std::string& target) {

        if (!move_group_->setNamedTarget(target)) {
            RCLCPP_ERROR(this->get_logger(), "Named target '%s' not found.", target.c_str());
            return false;
        }

        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = move_group_->plan(plan);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Planning to named target '%s' failed.", target.c_str());
            return false;
        }

        auto exec_result = move_group_->execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Execution to named target '%s' failed.", target.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Reached named target: %s", target.c_str());
        return true;
    }

    bool move_camera_to_xyz(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.header.stamp = now();

        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        target.pose.orientation.x = qx;
        target.pose.orientation.y = qy;
        target.pose.orientation.z = qz;
        target.pose.orientation.w = qw;

        RCLCPP_INFO(get_logger(), "Trying pose target: x=%.3f y=%.3f z=%.3f", x, y, z);

        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();
        move_group_->setStartStateToCurrentState();
        auto constraints = make_z_box_constraint("base_link", "zed_camera_center");
        move_group_->setPathConstraints(constraints);
        move_group_->setPoseTarget(target, "zed_camera_center");
        move_group_->setPlanningTime(5.0);
        move_group_->setMaxVelocityScalingFactor(0.20);
        move_group_->setMaxAccelerationScalingFactor(0.20);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = move_group_->plan(plan);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Planning to pose target failed.");
            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();
            return false;
        }

        auto exec_result = move_group_->execute(plan);
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Execution to pose target failed.");
            return false;
        }

        return true;
    }

    moveit_msgs::msg::Constraints make_z_box_constraint(const std::string& base_frame,const std::string& link_name)
    {
        moveit_msgs::msg::Constraints constraints;
        constraints.name = "keep_ee_below_zmax";

        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = base_frame;
        pc.link_name = link_name;
        pc.weight = 1.0;

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions.resize(3);

        // Make x/y large enough to cover your working area.
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 2.0;
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 2.0;

        // z allowed range: 0.00 to 0.29
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.5;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;

        // Center of box is halfway between 0 and 0.29
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.25;

        pc.constraint_region.primitives.push_back(box);
        pc.constraint_region.primitive_poses.push_back(box_pose);

        constraints.position_constraints.push_back(pc);
        return constraints;
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

    void capture_done_cb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        last_capture_done_id_ = msg->data;
    }

    void map_ready_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        map_ready_ = msg->data;
    }

    bool wait_for_capture_done(int capture_id, double timeout_sec = 8.0)
    {
        rclcpp::Time start = now();
        rclcpp::Rate rate(20.0);

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(sync_mutex_);
                if (last_capture_done_id_ == capture_id) {
                    return true;
                }
            }

            if ((now() - start).seconds() > timeout_sec) {
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool wait_for_map_ready(double timeout_sec = 12.0)
    {
        rclcpp::Time start = now();
        rclcpp::Rate rate(20.0);

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(sync_mutex_);
                if (map_ready_) {
                    return true;
                }
            }

            if ((now() - start).seconds() > timeout_sec) {
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    void reset_sync_flags()
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        last_capture_done_id_ = -1;
        map_ready_ = false;
    }

    


    // -------- main logic --------
    
    void run_scan_sequence() {

        reset_sync_flags();

        /*publish_status("moving_to_scan_start");
        if (!move_named_pose("scan_start")) {
            publish_status("scan_failed");
            busy_ = false;
            return;
        }*/

        std::vector<ViewPose> views = {
               // left
            {-0.008, -0.185, 0.334, -0.001, -0.710, -0.005, 0.705},   // center
            {0.626, -0.190, 0.338, -0.001, -0.710, -0.005, 0.705}   // right
        };

        for (size_t i = 0; i < views.size(); ++i) {
            const int capture_id = static_cast<int>(i + 1);

            publish_status("moving_to_view_" + std::to_string(capture_id));
            if (!move_camera_to_xyz(views[i].x, views[i].y, views[i].z, views[i].qx, views[i].qy, views[i].qz,views[i].qw )) {
                publish_status("scan_failed");
                move_named_pose("retracted");
                busy_ = false;
                return;
            }

            publish_status("settling_view_" + std::to_string(capture_id));
            rclcpp::sleep_for(std::chrono::milliseconds(700));

            publish_status("capturing_view_" + std::to_string(capture_id));
            trigger_capture(capture_id);

            if (!wait_for_capture_done(capture_id, 10.0)) {
                RCLCPP_ERROR(get_logger(), "Capture %d timeout.", capture_id);
                publish_status("capture_timeout");
                move_named_pose("retracted");
                busy_ = false;
                return;
            }
        }

        publish_status("waiting_local_map");
        if (!wait_for_map_ready(15.0)) {
            RCLCPP_ERROR(get_logger(), "Map ready timeout.");
            publish_status("map_timeout");
            move_named_pose("retracted");
            busy_ = false;
            return;
        }

        publish_status("local_map_ready");


        publish_status("returning_home");
        move_named_pose("retracted");

        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        pub_done_->publish(done_msg);
        publish_status("scan_done");
        busy_ = false;
    }

    

private:
    bool busy_ = false;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_done_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_trigger_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_auto_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_capture_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_map_ready_;

    rclcpp::TimerBase::SharedPtr timer_startup_;

    std::mutex sync_mutex_;
    int last_capture_done_id_ = -1;
    bool map_ready_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmBenchScanController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}