#ifndef YOCS_VELOCITY_SMOOTHER_HPP_
#define YOCS_VELOCITY_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/logger.hpp>

namespace yocs_velocity_smoother
{

  class VelocitySmoother : public rclcpp::Node
  {
  public:
    VelocitySmoother();

    ~VelocitySmoother() {}

    void spin();
    void shutdown() { shutdown_req_ = true; }
    std::mutex locker;

  private:
    enum RobotFeedbackType
    {
      NONE,
      ODOMETRY,
      COMMANDS
    } robot_feedback_;

    bool quiet_;
    double speed_lim_v_x_, speed_lim_v_y_, accel_lim_v_, decel_lim_v_;
    double speed_lim_w_, accel_lim_w_, decel_lim_w_;
    double decel_factor_;
    double frequency_;

    geometry_msgs::msg::Twist last_cmd_vel_;
    geometry_msgs::msg::Twist current_vel_;
    geometry_msgs::msg::Twist target_vel_;

    bool shutdown_req_;
    bool input_active_;
    double cb_avg_time_;
    rclcpp::Time last_cb_time_;
    std::vector<double> period_record_;
    unsigned int pr_next_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_in_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smooth_vel_pub_;

    void velocityCB(const geometry_msgs::msg::Twist &msg);
    void robotVelCB(const geometry_msgs::msg::Twist &msg);
    void odometryCB(const nav_msgs::msg::Odometry &msg);

    double sign(double x) { return x < 0.0 ? -1.0 : +1.0; }

    double median(std::vector<double> values)
    {
      // Return the median element of an doubles vector
      nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
      return values[values.size() / 2];
    }
  };
} // namespace yocs_velocity_smoother

#endif // YOCS_VELOCITY_SMOOTHER_HPP_
