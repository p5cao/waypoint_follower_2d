# include <rclcpp/rclcpp.hpp>
# include "nav_msgs/msg/odometry.hpp"
# include "geometry_msgs/msg/twist.hpp"
# include "geometry_msgs/msg/pose_stamped.hpp"
# include "geometry_msgs/msg/pose_with_covariance.hpp"
# include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
# include <tf2/utils.h>
# include <algorithm>
# include <chrono>

using namespace std::placeholders;

class WaypointFollower2D : public rclcpp::Node
{
    public:
        WaypointFollower2D() : Node("waypoint_follower_2d_node"),
        reached_goal_(true) 
        {
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                rclcpp::QoS(10),
                std::bind(&WaypointFollower2D::odomCallback, this, _1)
            );

            goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal",
                rclcpp::QoS(10),
                std::bind(&WaypointFollower2D::goalCallback, this, _1)
            );

            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                rclcpp::QoS(10)
            );

            // to run a function continuously, need to bind it to a wall timer
            // if dont set the wall timer, the controller will only run when a new /goal message is received
            // control_timer_ = this->create_wall_timer(
            //     std::chrono::milliseconds(100), // 10 Hz
            //     std::bind(&WaypointFollower2D::compute_velocity_command, this)
            // );

            RCLCPP_INFO(this->get_logger(), "Waypoint follower 2D node started ...");

        };
    private:
        
        void compute_velocity_command ()
        {

            if (!estimated_pose_.has_value() || !estimated_pose_.has_value()){
                return ;
            }

            // just make /cmd_vel_ a local variable inside the function
            geometry_msgs::msg::Twist cmd_vel_;

            double x_odom_ = estimated_pose_->pose.position.x;
            double y_odom_ = estimated_pose_->pose.position.y;

            double yaw_odom_ = tf2::getYaw(estimated_pose_->pose.orientation); // convert estimated quaternion to yaw

            double x_goal_ = goal_pose_->position.x;
            double y_goal_ = goal_pose_->position.y;

            double yaw_goal_ = tf2::getYaw(goal_pose_->orientation); // convert estimated goal quaternion to yaw


            //Calculate the distance from the current pose to the goal.
            double distance_ = std::hypot(x_goal_ - x_odom_, y_goal_ - y_odom_);

            double yaw_error_ = yaw_goal_ - yaw_odom_;

            // wrap to [-π, π]
            yaw_error_ = atan2(sin(yaw_error_), cos(yaw_error_));

            double k_linear = 0.5;
            double k_angular = 0.5;

            double yaw_velocity_ = 0.0;
            double forward_velocity_ = 0.0;

            // rotate then drive approach
            if (distance_ >= 0.2){
                reached_goal_ = false;
                // step 3: large yaw error - rotate first, dont drive yeat
                if (std::abs(yaw_error_)>=1)
                {
                    yaw_velocity_ = std::clamp(k_angular * yaw_error_, -1.0, 1.0);
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.angular.z = yaw_velocity_;
                    RCLCPP_INFO(this->get_logger(), "Yaw error is %.2f rad,  at %.2f rad/s", yaw_error_, yaw_velocity_);
                }
                // step 2: yaw is aligned - drive forward
                else
                {   
                    forward_velocity_ = std::clamp(k_linear * distance_, 0.0, 0.5);
                    cmd_vel_.linear.x = forward_velocity_;
                    cmd_vel_.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Distance to goal is %.2f m, moving forward at %.2f m/s", distance_, forward_velocity_);
                }
            }
            // step 4: close enough -- stop
            else
            {   
                reached_goal_ = true;
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.0;
            }

            // if (distance_ >= 0.2){
            //     reached_goal_ = false;
            //     double forward_velocity_ = k_linear * distance_;
            //     // Clamp to velocity limits
            //     forward_velocity_ = std::clamp(forward_velocity_, 0.0, 0.5);
            //     cmd_vel_.linear.x = forward_velocity_;
            //     RCLCPP_INFO(this->get_logger(), "Distance to goal is %.2f m, moving forward at %.2f m/s", distance_, forward_velocity_);
            // } 
            // else if (yaw_error_>=0.1 || yaw_error_<=-0.1)
            // {
            //     reached_goal_ = false;
            //     double yaw_velocity_ = k_angular * yaw_error_;
            //     // Clamp to velocity limits
            //     cmd_vel_.angular.z  = std::clamp(yaw_velocity_, -1.0, 1.0);

            //     RCLCPP_INFO(this->get_logger(), "Yaw error is %.2f rad,  at %.2f rad/s", yaw_error_, yaw_velocity_);
            // }
           
            // if (reached_goal_)
            // {
            //     cmd_vel_.linear.x = 0.0;
            //     cmd_vel_.linear.y = 0.0;
            //     cmd_vel_.linear.z = 0.0;
            //     cmd_vel_.angular.x = 0.0; 
            //     cmd_vel_.angular.y = 0.0;
            //     cmd_vel_.angular.z = 0.0;
            //     RCLCPP_INFO(this->get_logger(), "Goal reached! Moving stopped.");
            // }

            cmd_vel_publisher_->publish(cmd_vel_);
        };


        void odomCallback (const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            estimated_pose_ = msg->pose; //geometry::msgs::PoseWithVairance::pose

            // also call from odomCallback, keep publishing until the goal is reached
            if (goal_pose_.has_value() && !reached_goal_){
                 compute_velocity_command();
            }

        };

        void goalCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            goal_pose_ = msg->pose; //geometry::msgs::PoseStamped::pose
            reached_goal_ = false;

            if (!estimated_pose_.has_value()) {
                RCLCPP_WARN(this->get_logger(), "No odometry received yet, ignoring goal.");
                return;
            }
            compute_velocity_command(); // trigger here

        };



        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        //rclcpp::TimerBase::SharedPtr control_timer_; 

        std::optional<geometry_msgs::msg::PoseWithCovariance> estimated_pose_;
        //std::optional<geometry_msgs::msg::TwistWithCovariance> estimated_twist_;
        std::optional<geometry_msgs::msg::Pose> goal_pose_;
        // std::optional<geometry_msgs::msg::Twist> cmd_vel_; std::optional will crash the cmd_vel_

        bool reached_goal_;


};




int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointFollower2D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}