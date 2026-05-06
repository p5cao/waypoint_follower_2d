# include <rclcpp/rclcpp.hpp>
# include "nav_msgs/msg/odometry.hpp"
# include "geometry_msgs/msg/twist.hpp"
# include "geometry_msgs/msg/pose_stamped.hpp"
# include "geometry_msgs/msg/pose_with_covariance.hpp"
# include "tf2/impl/utils.hpp"


using namespace std::placeholders;

class WaypointFollower2D : public rclcpp::Node
{
    public:
        WaypointFollower2D() : Node("waypoint_follower_2d_node") {
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                rclcpp::QoS(10),
                std::bind(&odomCallback, this, _1)
            );

            goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal",
                rclcpp::QoS(10),
                std::bind(&goalCallback, this, _1)
            );

            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                rclcpp::QoS(10)
            );

        };
    private:
        
        const geometry_msgs::msg::Twist::SharedPtr compute_velocity_command (

        )
        {
            double x = estimated_pose_->pose.position.x;
            double y = estimated_pose_->pose.position.x;

            double yaw = tf2::impl::getYaw(estimated_pose_->pose.orientation); // convert estimated quaternion to yaw

        };


        void odomCallback (const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            estimated_pose_ = msg->pose;
        };

        void goalCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            goal_pose_ = msg->pose;
        };



        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        std::optional<geometry_msgs::msg::PoseWithCovariance> estimated_pose_;
        //std::optional<geometry_msgs::msg::TwistWithCovariance> estimated_twist_;
        std::optional<geometry_msgs::msg::Pose> goal_pose_;


};




int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointFollower2D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
