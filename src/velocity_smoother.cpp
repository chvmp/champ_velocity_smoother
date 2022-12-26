#include <yocs_velocity_smoother/velocity_smoother.hpp>

#define PERIOD_RECORD_SIZE 5
#define ZERO_VEL_COMMAND geometry_msgs::msg::Twist();
#define IS_ZERO_VEOCITY(a) ((a.linear.x == 0.0) && (a.linear.y == 0.0) && (a.angular.z == 0.0))

using std::placeholders::_1;

namespace yocs_velocity_smoother
{
  VelocitySmoother::VelocitySmoother()
      : Node("velocity_smoother"),
        robot_feedback_(RobotFeedbackType::NONE),
        quiet_(false),
        shutdown_req_(false),
        input_active_(false),
        cb_avg_time_(0),
        pr_next_(0)
  {
    declare_parameter("speed_lim_v_x", 1.0);
    declare_parameter("accel_lim_v", 0.5);
    declare_parameter("speed_lim_w", 5.0);
    declare_parameter("accel_lim_w", 2.5);
    declare_parameter("decel_factor", 1.0);
    declare_parameter("frequency", 20.0);
    declare_parameter("robot_feedback_", (int)NONE);

    speed_lim_v_x_ = get_parameter("speed_lim_v_x").as_double();
    speed_lim_v_y_ = speed_lim_v_x_ * 0.6;
    accel_lim_v_ = get_parameter("accel_lim_v").as_double();
    decel_factor_ = get_parameter("decel_factor").as_double();
    decel_lim_v_ = decel_factor_ * accel_lim_v_;
    speed_lim_w_ = get_parameter("speed_lim_w").as_double();
    accel_lim_w_ = get_parameter("accel_lim_w").as_double();
    decel_lim_w_ = decel_factor_ * accel_lim_w_;
    frequency_ = get_parameter("frequency").as_double();
    robot_feedback_ = static_cast<RobotFeedbackType>(get_parameter("robot_feedback_").as_int());

    smooth_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    raw_in_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "raw_cmd_vel", rclcpp::QoS(10), std::bind(&VelocitySmoother::velocityCB, this, _1));
    current_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "current_velocity", rclcpp::QoS(10), std::bind(&VelocitySmoother::robotVelCB, this, _1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(10), std::bind(&VelocitySmoother::odometryCB, this, _1));
  }

  void VelocitySmoother::velocityCB(const geometry_msgs::msg::Twist &msg)
  {
    // Estimate commands frequency; we do continuously as it can be very different depending on the
    // publisher type, and we don't want to impose extra constraints to keep this package flexible
    if (period_record_.size() < PERIOD_RECORD_SIZE)
    {
      period_record_.push_back((rclcpp::Clock().now() - last_cb_time_).seconds());
    }
    else
    {
      period_record_[pr_next_] = (rclcpp::Clock().now() - last_cb_time_).seconds();
    }

    pr_next_++;
    pr_next_ %= period_record_.size();
    last_cb_time_ = rclcpp::Clock().now();

    if (period_record_.size() <= PERIOD_RECORD_SIZE / 2)
    {
      // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
      cb_avg_time_ = 0.1;
    }
    else
    {
      // enough; recalculate with the latest input
      cb_avg_time_ = median(period_record_);
    }

    input_active_ = true;

    // Bound speed with the maximum values
    locker.lock();
    target_vel_.linear.x =
        msg.linear.x > 0.0 ? std::min(msg.linear.x, speed_lim_v_x_) : std::max(msg.linear.x, -speed_lim_v_x_);
    target_vel_.linear.y =
        msg.linear.y > 0.0 ? std::min(msg.linear.y, speed_lim_v_y_) : std::max(msg.linear.y, -speed_lim_v_y_);
    target_vel_.angular.z =
        msg.angular.z > 0.0 ? std::min(msg.angular.z, speed_lim_w_) : std::max(msg.angular.z, -speed_lim_w_);
    locker.unlock();
  }

  void VelocitySmoother::odometryCB(const nav_msgs::msg::Odometry &msg)
  {
    if (robot_feedback_ == ODOMETRY)
      current_vel_ = msg.twist.twist;
  }

  void VelocitySmoother::robotVelCB(const geometry_msgs::msg::Twist &msg)
  {
    if (robot_feedback_ == COMMANDS)
      current_vel_ = msg;
  }

  void VelocitySmoother::spin()
  {
    double period = 1.0 / frequency_;
    rclcpp::Rate spin_rate(frequency_);

    while (!shutdown_req_ && rclcpp::ok())
    {
      locker.lock();
      double accel_lim_v_(accel_lim_v_);
      double accel_lim_w_(accel_lim_w_);
      double decel_factor(decel_factor_);
      double decel_lim_v_(decel_lim_v_);
      double decel_lim_w_(decel_lim_w_);
      locker.unlock();

      if ((input_active_ == true) && (cb_avg_time_ > 0.0) &&
          ((rclcpp::Clock().now() - last_cb_time_).seconds() > std::min(3.0 * cb_avg_time_, 0.5)))
      {
        // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
        // this, just in case something went wrong with our input, or he just forgot good manners...
        // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
        // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
        // several messages arrive with the same time and so lead to a zero median
        input_active_ = false;
        if (IS_ZERO_VEOCITY(target_vel_) == false)
        {
          if (!quiet_)
          {
            RCLCPP_WARN_STREAM(
                get_logger(),
                "Velocity Smoother : input got inactive leaving us a non-zero target velocity (" << target_vel_.linear.x << ", " << target_vel_.linear.y << ", " << target_vel_.angular.z << "), zeroing...[%s]");
          }
          target_vel_ = ZERO_VEL_COMMAND;
        }
      }

      // check if the feedback is off from what we expect
      // don't care about min / max velocities here, just for rough checking
      double period_buffer = 2.0;

      double v_deviation_lower_bound = last_cmd_vel_.linear.x - decel_lim_v_ * period * period_buffer;
      double v_deviation_upper_bound = last_cmd_vel_.linear.x + accel_lim_v_ * period * period_buffer;

      double w_deviation_lower_bound = last_cmd_vel_.angular.z - decel_lim_w_ * period * period_buffer;
      double angular_max_deviation = last_cmd_vel_.angular.z + accel_lim_w_ * period * period_buffer;

      bool v_different_from_feedback = current_vel_.linear.x < v_deviation_lower_bound || current_vel_.linear.x > v_deviation_upper_bound;
      bool w_different_from_feedback = current_vel_.angular.z < w_deviation_lower_bound || current_vel_.angular.z > angular_max_deviation;

      if ((robot_feedback_ != NONE) && (input_active_ == true) && (cb_avg_time_ > 0.0) &&
          (((rclcpp::Clock().now() - last_cb_time_).seconds() > 5.0 * cb_avg_time_) || // 5 missing msgs
           v_different_from_feedback || w_different_from_feedback))
      {
        // If the publisher has been inactive for a while, or if our current commanding differs a lot
        // from robot velocity feedback, we cannot trust the former; relay on robot's feedback instead
        // This might not work super well using the odometry if it has a high delay
        if (!quiet_)
        {
          // this condition can be unavoidable due to preemption of current velocity control on
          // velocity multiplexer so be quiet if we're instructed to do so
          RCLCPP_WARN(
              get_logger(),
              "Velocity Smoother : using robot velocity feedback %s instead of last command: %s",
              std::string(robot_feedback_ == ODOMETRY ? "odometry" : "end commands").c_str(),
              ((rclcpp::Clock().now() - last_cb_time_).seconds() > 5.0 * cb_avg_time_) ? "not received for a while" : "deviates a lot from robot feedback");
        }
        target_vel_ = current_vel_;
      }

      geometry_msgs::msg::Twist cmd_vel;

      if ((target_vel_.linear.x != last_cmd_vel_.linear.x) ||
          (target_vel_.linear.y != last_cmd_vel_.linear.y) ||
          (target_vel_.angular.z != last_cmd_vel_.angular.z))
      {
        // Try to reach target velocity ensuring that we don't exceed the acceleration limits
        cmd_vel = target_vel_;

        double v_inc, v_inc_y, w_inc, max_v_inc, max_v_inc_y, max_w_inc;

        v_inc = target_vel_.linear.x - last_cmd_vel_.linear.x;
        if ((robot_feedback_ == ODOMETRY) && (current_vel_.linear.x * target_vel_.linear.x < 0.0))
        {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_v_inc = decel_lim_v_ * period;
        }
        else
        {
          max_v_inc = ((v_inc * target_vel_.linear.x > 0.0) ? accel_lim_v_ : decel_lim_v_) * period;
        }

        v_inc_y = target_vel_.linear.y - last_cmd_vel_.linear.y;
        if ((robot_feedback_ == ODOMETRY) && (current_vel_.linear.y * target_vel_.linear.y < 0.0))
        {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_v_inc_y = decel_lim_v_ * period;
        }
        else
        {
          max_v_inc_y = ((v_inc_y * target_vel_.linear.y > 0.0) ? accel_lim_v_ : decel_lim_v_) * period;
        }

        w_inc = target_vel_.angular.z - last_cmd_vel_.angular.z;
        if ((robot_feedback_ == ODOMETRY) && (current_vel_.angular.z * target_vel_.angular.z < 0.0))
        {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_w_inc = decel_lim_w_ * period;
        }
        else
        {
          max_w_inc = ((w_inc * target_vel_.angular.z > 0.0) ? accel_lim_w_ : decel_lim_w_) * period;
        }

        // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
        // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
        // which velocity (v or w) must be overconstrained to keep the direction provided as command
        double MA = sqrtf(v_inc * v_inc + w_inc * w_inc);
        double MB = sqrtf(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

        double Av = std::abs(v_inc) / MA;
        double Aw = std::abs(w_inc) / MA;
        double Bv = max_v_inc / MB;
        double Bw = max_w_inc / MB;
        double theta = atan2f(Bw, Bv) - atan2f(Aw, Av);

        if (theta < 0)
        {
          // overconstrain linear velocity
          max_v_inc = (max_w_inc * std::abs(v_inc)) / std::abs(w_inc);
        }
        else
        {
          // overconstrain angular velocity
          max_w_inc = (max_v_inc * std::abs(w_inc)) / std::abs(v_inc);
        }

        if (std::abs(v_inc) > max_v_inc)
        {
          // we must limit linear velocity
          cmd_vel.linear.x = last_cmd_vel_.linear.x + sign(v_inc) * max_v_inc;
        }

        if (std::abs(v_inc_y) > max_v_inc_y)
        {
          // we must limit linear velocity
          cmd_vel.linear.y = last_cmd_vel_.linear.y + sign(v_inc_y) * max_v_inc_y;
        }

        if (std::abs(w_inc) > max_w_inc)
        {
          // we must limit angular velocity
          cmd_vel.angular.z = last_cmd_vel_.angular.z + sign(w_inc) * max_w_inc;
        }
        smooth_vel_pub_->publish(cmd_vel);
        last_cmd_vel_ = cmd_vel;
      }
      else if (input_active_ == true)
      {
        // We already reached target velocity; just keep resending last command while input is active
        cmd_vel = last_cmd_vel_;
        smooth_vel_pub_->publish(cmd_vel);
      }

      spin_rate.sleep();
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto smoother = std::make_shared<yocs_velocity_smoother::VelocitySmoother>();
  smoother->spin();
  rclcpp::shutdown();
  return 0;
}
