// overwatch speed governor node: Ouster range image + IMU + LIO pose + wheels + cmd -> a speed factor that
// pure pursuit multiplies into its target velocity. A dead node means full speed, by design.

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <overwatch/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <regex>
#include <sstream>

#include "overwatch/speed_governor.h"

namespace overwatch {

static_assert(int(State::NONE) == kNone && int(State::TILT) == kTilt && int(State::ROTATION) == kRotation &&
              int(State::PUSH) == kPush && int(State::COVER) == kCover, "limiter ids must match State.msg");
static_assert(int(State::IMU) == kStaleImu && int(State::ODOM) == kStaleOdom &&
              int(State::WHEELS) == kStaleWheels && int(State::CMD) == kStaleCmd &&
              int(State::IMAGE) == kStaleImage, "stale bits must match State.msg");

namespace {
void rollPitch(const geometry_msgs::Quaternion& q, float* roll, float* pitch) {
  *roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  *pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
}

std::vector<float> parseList(const std::string& json, const std::string& key) {
  std::vector<float> out;
  const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
  std::smatch m;
  if (!std::regex_search(json, m, re)) return out;
  std::stringstream ss(m[1].str());
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(std::stof(tok));
  return out;
}

const char* limiterName(uint8_t l) {
  switch (l) {
    case kTilt: return "tilt";
    case kRotation: return "rotation";
    case kPush: return "push";
    case kCover: return "cover";
    default: return "none";
  }
}

Params loadParams(ros::NodeHandle& pnh) {
  Params p;
  pnh.param("release_rate", p.release_rate, p.release_rate);
  pnh.param("odometer_deadband_m", p.odometer_deadband_m, p.odometer_deadband_m);
  pnh.param("speed_window_s", p.speed_window_s, p.speed_window_s);
  pnh.param("speed_min_span_s", p.speed_min_span_s, p.speed_min_span_s);

  pnh.param("proprioceptive/enable", p.proprioceptive, p.proprioceptive);
  auto& t = p.tilt;
  pnh.param("proprioceptive/tilt/enable", t.enable, t.enable);
  pnh.param("proprioceptive/tilt/roll_lo", t.roll_lo, t.roll_lo);
  pnh.param("proprioceptive/tilt/roll_hi", t.roll_hi, t.roll_hi);
  pnh.param("proprioceptive/tilt/roll_floor", t.roll_floor, t.roll_floor);
  pnh.param("proprioceptive/tilt/pitch_lo", t.pitch_lo, t.pitch_lo);
  pnh.param("proprioceptive/tilt/pitch_hi", t.pitch_hi, t.pitch_hi);
  pnh.param("proprioceptive/tilt/pitch_floor", t.pitch_floor, t.pitch_floor);
  auto& r = p.rotation;
  pnh.param("proprioceptive/rotation/enable", r.enable, r.enable);
  pnh.param("proprioceptive/rotation/window_s", r.window_s, r.window_s);
  pnh.param("proprioceptive/rotation/max_dt_s", r.max_dt_s, r.max_dt_s);
  pnh.param("proprioceptive/rotation/lo", r.lo, r.lo);
  pnh.param("proprioceptive/rotation/hi", r.hi, r.hi);
  pnh.param("proprioceptive/rotation/floor", r.floor, r.floor);
  pnh.param("proprioceptive/rotation/min_speed", r.min_speed, r.min_speed);
  pnh.param("proprioceptive/rotation/hold_m", r.hold_m, r.hold_m);
  auto& u = p.push;
  pnh.param("proprioceptive/push/enable", u.enable, u.enable);
  pnh.param("proprioceptive/push/window_s", u.window_s, u.window_s);
  pnh.param("proprioceptive/push/baseline", u.baseline, u.baseline);
  pnh.param("proprioceptive/push/slope_gain", u.slope_gain, u.slope_gain);
  pnh.param("proprioceptive/push/lo", u.lo, u.lo);
  pnh.param("proprioceptive/push/hi", u.hi, u.hi);
  pnh.param("proprioceptive/push/floor", u.floor, u.floor);
  pnh.param("proprioceptive/push/min_cmd_v", u.min_cmd_v, u.min_cmd_v);
  pnh.param("proprioceptive/push/hold_m", u.hold_m, u.hold_m);

  pnh.param("predictive/enable", p.predictive, p.predictive);
  auto& c = p.cover;
  pnh.param("predictive/cover/enable", c.enable, c.enable);
  pnh.param("predictive/cover/half_sector_deg", c.half_sector_deg, c.half_sector_deg);
  pnh.param("predictive/cover/max_ray_z", c.max_ray_z, c.max_ray_z);
  pnh.param("predictive/cover/min_rays", c.min_rays, c.min_rays);
  pnh.param("predictive/cover/window_sweeps", c.window_sweeps, c.window_sweeps);
  pnh.param("predictive/cover/lo", c.lo, c.lo);
  pnh.param("predictive/cover/hi", c.hi, c.hi);
  pnh.param("predictive/cover/floor", c.floor, c.floor);
  return p;
}
}  // namespace

class OverwatchNode {
 public:
  OverwatchNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : gov_(loadParams(pnh)) {
    pnh.param("apply", apply_, apply_);
    pnh.param("imu_yaw_deg", imu_yaw_deg_, imu_yaw_deg_);
    pnh.param("wheel_radius", wheel_radius_, wheel_radius_);
    pnh.param("stale/imu", stale_imu_, stale_imu_);
    pnh.param("stale/odom", stale_odom_, stale_odom_);
    pnh.param("stale/wheels", stale_wheels_, stale_wheels_);
    pnh.param("stale/cmd", stale_cmd_, stale_cmd_);
    pnh.param("stale/image", stale_image_, stale_image_);
    double publish_rate = 20.0;
    pnh.param("publish_rate", publish_rate, publish_rate);

    state_pub_ = nh.advertise<State>("overwatch/state", 10);
    if (apply_) factor_pub_ = nh.advertise<std_msgs::Float32>("overwatch/speed_factor", 10);
    sub_imu_ = nh.subscribe("imu/data", 2000, &OverwatchNode::onImu, this);
    sub_odom_ = nh.subscribe("lio_odom", 50, &OverwatchNode::onOdom, this);
    sub_wheels_ = nh.subscribe("md80/joint_states", 200, &OverwatchNode::onWheels, this);
    sub_cmd_ = nh.subscribe("diff_drive_controller/cmd_vel_out", 200, &OverwatchNode::onCmd, this);
    sub_image_ = nh.subscribe("ouster/range_image", 2, &OverwatchNode::onImage, this);
    sub_meta_ = nh.subscribe("ouster/metadata", 1, &OverwatchNode::onMeta, this);
    timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate), &OverwatchNode::onTick, this);
    ROS_INFO("overwatch: speed governor up, apply=%s", apply_ ? "true" : "false");
  }

 private:
  void onImu(const sensor_msgs::Imu& m) {
    const Eigen::Vector2d g =
        imuToBase(m.angular_velocity.x, m.angular_velocity.y, imu_yaw_deg_ * M_PI / 180.0);
    if (gov_.addImu(m.header.stamp.toSec(), g.x(), g.y())) imu_rx_ = ros::Time::now();
  }

  void onOdom(const nav_msgs::Odometry& m) {
    float roll, pitch;
    rollPitch(m.pose.pose.orientation, &roll, &pitch);
    const Eigen::Vector2d p(m.pose.pose.position.x, m.pose.pose.position.y);
    if (gov_.addOdom(m.header.stamp.toSec(), p, roll, pitch)) odom_rx_ = ros::Time::now();
  }

  void onWheels(const sensor_msgs::JointState& m) {
    double net, speed;
    if (wheelsFromJoints(m.name, m.velocity, m.effort, wheel_radius_, &net, &speed) &&
        gov_.addWheels(m.header.stamp.toSec(), net, speed))
      wheels_rx_ = ros::Time::now();
  }

  void onCmd(const geometry_msgs::TwistStamped& m) {
    if (gov_.addCommand(m.twist.linear.x)) cmd_rx_ = ros::Time::now();
  }

  void onImage(const sensor_msgs::Image& m) {
    if (m.encoding != sensor_msgs::image_encodings::MONO16) {
      ROS_WARN_THROTTLE(5.0, "overwatch: ignoring range image with encoding '%s'", m.encoding.c_str());
      return;
    }
    if (m.data.size() < static_cast<size_t>(m.height) * m.step) {
      ROS_WARN_THROTTLE(5.0, "overwatch: ignoring truncated range image");
      return;
    }
    if (gov_.addRangeImage(m.data.data(), m.height, m.width, m.step)) image_rx_ = ros::Time::now();
  }

  void onMeta(const std_msgs::String& m) {
    std::vector<float> alt;
    try {
      alt = parseList(m.data, "beam_altitude_angles");
    } catch (const std::exception& e) {  // bad metadata falls back to the linear elevations
      ROS_WARN("overwatch: could not parse beam altitudes: %s", e.what());
      return;
    }
    if (gov_.setBeamAltitudes(alt)) ROS_INFO("overwatch: beam altitudes from metadata (%zu beams)", alt.size());
  }

  static bool isStale(const ros::Time& rx, const ros::Time& now, double timeout) {
    return rx.isZero() || (now - rx).toSec() > timeout;
  }

  void onTick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    Stale s;
    s.imu = isStale(imu_rx_, now, stale_imu_);
    s.odom = isStale(odom_rx_, now, stale_odom_);
    s.wheels = isStale(wheels_rx_, now, stale_wheels_);
    s.cmd = isStale(cmd_rx_, now, stale_cmd_);
    s.image = isStale(image_rx_, now, stale_image_);
    const Output o = gov_.update(now.toSec(), s);

    if (apply_) {
      std_msgs::Float32 f;
      f.data = o.speed_factor;
      factor_pub_.publish(f);
    }

    State st;
    st.header.stamp = now;
    st.speed_factor = o.speed_factor;
    st.limiting = o.limiting;
    st.applied = apply_;
    st.tilt_factor = o.tilt_factor;
    st.rotation_factor = o.rotation_factor;
    st.push_factor = o.push_factor;
    st.cover_factor = o.cover_factor;
    st.roll_deg = o.roll_deg;
    st.pitch_deg = o.pitch_deg;
    st.droll_deg = o.droll_deg;
    st.dpitch_deg = o.dpitch_deg;
    st.rotation_deg = o.rotation_deg;
    st.net_effort = o.net_effort;
    st.effort_residual = o.effort_residual;
    st.cover_raw = o.cover_raw;
    st.cover_avg = o.cover_avg;
    st.cover_expected = o.cover_expected;
    st.lio_speed = o.lio_speed;
    st.wheel_speed = o.wheel_speed;
    st.odometer = o.odometer;
    st.cover_compute_ms = o.cover_compute_ms;
    st.stale = o.stale;
    state_pub_.publish(st);

    if (o.limiting != logged_limiter_ && (now - last_log_).toSec() >= 1.0) {
      ROS_INFO("overwatch: limiting=%s factor=%.2f", limiterName(o.limiting), o.speed_factor);
      logged_limiter_ = o.limiting;
      last_log_ = now;
    }
  }

  SpeedGovernor gov_;
  bool apply_ = false;
  double imu_yaw_deg_ = 90.0, wheel_radius_ = 0.13;
  double stale_imu_ = 0.2, stale_odom_ = 0.5, stale_wheels_ = 0.5, stale_cmd_ = 0.5, stale_image_ = 0.5;
  ros::Publisher state_pub_, factor_pub_;
  ros::Subscriber sub_imu_, sub_odom_, sub_wheels_, sub_cmd_, sub_image_, sub_meta_;
  ros::Timer timer_;
  ros::Time imu_rx_, odom_rx_, wheels_rx_, cmd_rx_, image_rx_, last_log_;
  uint8_t logged_limiter_ = kNone;
};

}  // namespace overwatch

int main(int argc, char** argv) {
  ros::init(argc, argv, "overwatch");
  ros::NodeHandle nh, pnh("~");
  overwatch::OverwatchNode node(nh, pnh);
  ros::spin();
  return 0;
}
