#include "overwatch/speed_governor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace overwatch {

namespace {
constexpr double kDeg = M_PI / 180.0;
// OS1-128 elevation span, used when the metadata is missing or does not match the image.
constexpr float kFallbackTopDeg = 21.25f, kFallbackBottomDeg = -21.39f;

bool finite(double a) { return std::isfinite(a); }
}  // namespace

float interp(float x, float x0, float x1, float y0, float y1) {
  if (!(x > x0)) return y0;
  if (x >= x1) return y1;
  return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

Eigen::Vector2d imuToBase(double ix, double iy, double yaw_rad) {
  const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
  return Eigen::Vector2d(c * ix - s * iy, s * ix + c * iy);
}

bool wheelsFromJoints(const std::vector<std::string>& names, const std::vector<double>& velocity,
                      const std::vector<double>& effort, double wheel_radius, double* net_effort,
                      double* wheel_speed) {
  if (velocity.size() < names.size() || effort.size() < names.size()) return false;
  double net = 0.0, speed = 0.0;
  int seen = 0;  // bitmask, so a repeated joint cannot stand in for a missing one
  for (size_t i = 0; i < names.size(); ++i) {
    double sign;
    int bit;
    if (names[i] == "Joint 103") { sign = 1.0; bit = 1; }
    else if (names[i] == "Joint 101") { sign = 1.0; bit = 2; }
    else if (names[i] == "Joint 102") { sign = -1.0; bit = 4; }
    else if (names[i] == "Joint 100") { sign = -1.0; bit = 8; }
    else continue;
    if (!finite(effort[i]) || !finite(velocity[i])) return false;
    net += sign * effort[i];
    speed += sign * velocity[i];
    seen |= bit;
  }
  if (seen != 15) return false;
  *net_effort = net;
  *wheel_speed = speed / 4.0 * wheel_radius;
  return true;
}

float DistanceHold::apply(float f, double odo) {
  if (f < 1.f && f <= held_) {
    held_ = f;
    anchor_ = odo;
  } else if (odo - anchor_ >= hold_m_) {
    held_ = f;
    anchor_ = odo;
  }
  return held_;
}

void DistanceHold::reset(double odo) {
  held_ = 1.f;
  anchor_ = odo;
}

SpeedGovernor::SpeedGovernor(const Params& p)
    : p_(p), rotation_hold_(p.rotation.hold_m), push_hold_(p.push.hold_m) {}

bool SpeedGovernor::addImu(double stamp, double gx, double gy) {
  if (!finite(stamp) || !finite(gx) || !finite(gy)) {
    rot_window_.clear();
    cur_droll_ = cur_dpitch_ = 0.0;
    have_imu_stamp_ = false;
    return false;
  }
  const double dt = have_imu_stamp_ ? stamp - imu_stamp_ : 0.0;
  imu_stamp_ = stamp;
  have_imu_stamp_ = true;
  if (dt <= 0.0 || dt > p_.rotation.max_dt_s) {
    rot_window_.clear();
  } else {
    cum_roll_ += gx * dt / kDeg;
    cum_pitch_ += gy * dt / kDeg;
  }
  rot_window_.push_back({stamp, cum_roll_, cum_pitch_});
  while (rot_window_.front().t < stamp - p_.rotation.window_s) rot_window_.pop_front();

  cur_droll_ = cum_roll_ - rot_window_.front().roll;
  cur_dpitch_ = cum_pitch_ - rot_window_.front().pitch;
  // A ticks-apart window can straddle a short peak; the peak is what the rule must see.
  if (std::fabs(cur_droll_) > std::fabs(peak_droll_)) peak_droll_ = cur_droll_;
  if (std::fabs(cur_dpitch_) > std::fabs(peak_dpitch_)) peak_dpitch_ = cur_dpitch_;
  return true;
}

bool SpeedGovernor::addOdom(double stamp, const Eigen::Vector2d& p, double roll, double pitch) {
  if (!finite(stamp) || !p.allFinite() || !finite(roll) || !finite(pitch)) return false;
  if (!have_odom_) {
    last_counted_ = p;
    have_odom_ = true;
  }
  const double moved = (p - last_counted_).norm();
  if (moved >= p_.odometer_deadband_m) {
    odometer_ += moved;
    last_counted_ = p;
  }
  roll_ = roll;
  pitch_ = pitch;
  attitude_valid_ = true;

  if (!odom_window_.empty() && stamp <= odom_window_.back().t) odom_window_.clear();
  odom_window_.push_back({stamp, p});
  while (odom_window_.front().t < stamp - p_.speed_window_s) odom_window_.pop_front();
  const double span = stamp - odom_window_.front().t;
  speed_valid_ = span >= p_.speed_min_span_s;
  speed_ = speed_valid_ ? (p - odom_window_.front().p).norm() / span : 0.0;
  return true;
}

bool SpeedGovernor::addWheels(double stamp, double net_effort, double wheel_speed) {
  if (!finite(stamp) || !finite(net_effort) || !finite(wheel_speed)) return false;
  if (!wheel_window_.empty() && stamp <= wheel_window_.back().t) wheel_window_.clear();
  wheel_window_.push_back({stamp, net_effort});
  while (wheel_window_.front().t < stamp - p_.push.window_s) wheel_window_.pop_front();
  wheel_speed_ = wheel_speed;
  return true;
}

bool SpeedGovernor::addCommand(double v) {
  if (!finite(v)) return false;
  cmd_v_ = v;
  have_cmd_ = true;
  return true;
}

bool SpeedGovernor::setBeamAltitudes(const std::vector<float>& deg) {
  if (deg.empty() || !std::all_of(deg.begin(), deg.end(), [](float a) { return std::isfinite(a); }))
    return false;
  if (deg != altitudes_deg_) ray_rows_ = 0;  // forces a rebuild on the next image
  altitudes_deg_ = deg;
  return true;
}

void SpeedGovernor::rebuildRays(int rows, int cols) {
  const bool use_metadata = static_cast<int>(altitudes_deg_.size()) == rows;
  rays_.resize(static_cast<size_t>(rows) * cols);
  for (int r = 0; r < rows; ++r) {
    const double frac = rows > 1 ? static_cast<double>(r) / (rows - 1) : 0.0;
    const double el = kDeg * (use_metadata ? altitudes_deg_[r]
                                           : kFallbackTopDeg + (kFallbackBottomDeg - kFallbackTopDeg) * frac);
    for (int c = 0; c < cols; ++c) {
      // The image is destaggered, so beam azimuth offsets must not be added here.
      const double az = kDeg * (180.0 - 360.0 * c / cols);
      rays_[static_cast<size_t>(r) * cols + c] =
          Eigen::Vector3f(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
    }
  }
  ray_rows_ = rows;
  ray_cols_ = cols;
}

bool SpeedGovernor::addRangeImage(const uint8_t* data, int rows, int cols, size_t step_bytes) {
  if (!data || rows <= 0 || cols <= 0 || step_bytes < 2 * static_cast<size_t>(cols)) return false;
  if (!p_.predictive || !p_.cover.enable) return true;
  const auto w0 = std::chrono::steady_clock::now();
  if (rows != ray_rows_ || cols != ray_cols_) rebuildRays(rows, cols);

  const float roll = attitude_valid_ ? roll_ : 0.f, pitch = attitude_valid_ ? pitch_ : 0.f;
  const Eigen::Matrix3f R = (Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
  const float half_sector = p_.cover.half_sector_deg * static_cast<float>(kDeg);
  uint32_t expected = 0, no_return = 0;
  const Eigen::Vector3f* ray = rays_.data();
  for (int r = 0; r < rows; ++r) {
    const uint8_t* row = data + r * step_bytes;
    for (int c = 0; c < cols; ++c, ++ray) {
      const Eigen::Vector3f d = R * *ray;
      if (d.z() >= p_.cover.max_ray_z) continue;
      if (std::fabs(std::atan2(d.y(), d.x())) >= half_sector) continue;
      ++expected;
      if (row[2 * c] == 0 && row[2 * c + 1] == 0) ++no_return;
    }
  }
  cover_expected_ = expected;
  if (static_cast<int>(expected) >= p_.cover.min_rays) {
    cover_raw_ = static_cast<float>(no_return) / expected;
    cover_fractions_.push_back(cover_raw_);
    while (static_cast<int>(cover_fractions_.size()) > p_.cover.window_sweeps) cover_fractions_.pop_front();
  } else {
    cover_raw_ = 0.f;
  }
  cover_compute_ms_ = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - w0).count();
  return true;
}

Output SpeedGovernor::update(double now, const Stale& stale) {
  Output o;
  o.stale = (stale.imu ? kStaleImu : 0) | (stale.odom ? kStaleOdom : 0) | (stale.wheels ? kStaleWheels : 0) |
            (stale.cmd ? kStaleCmd : 0) | (stale.image ? kStaleImage : 0);

  // The odometer cannot advance, so a hold could never release: drop them rather than latch.
  if (stale.odom) {
    attitude_valid_ = false;
    speed_ = 0.0;
    speed_valid_ = false;
    rotation_hold_.reset(odometer_);
    push_hold_.reset(odometer_);
  }
  // Before the rotation gate, which must not see the last wheel speed on the first stale tick.
  if (stale.wheels) {
    wheel_window_.clear();
    wheel_speed_ = 0.0;
  }
  const double roll = attitude_valid_ ? roll_ : 0.0, pitch = attitude_valid_ ? pitch_ : 0.0;
  o.roll_deg = roll / kDeg;
  o.pitch_deg = pitch / kDeg;
  o.lio_speed = speed_;
  o.wheel_speed = wheel_speed_;
  o.odometer = odometer_;

  // Tilt
  if (p_.proprioceptive && p_.tilt.enable && !stale.odom) {
    const auto& t = p_.tilt;
    o.tilt_factor = std::min(interp(std::fabs(o.roll_deg), t.roll_lo, t.roll_hi, 1.f, t.roll_floor),
                             interp(std::fabs(o.pitch_deg), t.pitch_lo, t.pitch_hi, 1.f, t.pitch_floor));
  }

  // Rotation
  if (stale.imu) {
    rot_window_.clear();
    cur_droll_ = cur_dpitch_ = peak_droll_ = peak_dpitch_ = 0.0;
  }
  o.droll_deg = peak_droll_;
  o.dpitch_deg = peak_dpitch_;
  o.rotation_deg = std::max(std::fabs(peak_droll_), std::fabs(peak_dpitch_));
  peak_droll_ = cur_droll_;
  peak_dpitch_ = cur_dpitch_;
  if (p_.proprioceptive && p_.rotation.enable && !stale.odom) {
    const auto& r = p_.rotation;
    // LIO speed collapses exactly when an obstacle stops the body, while the wheels keep turning.
    const float f = std::max(o.lio_speed, std::fabs(o.wheel_speed)) >= r.min_speed
                        ? interp(o.rotation_deg, r.lo, r.hi, 1.f, r.floor)
                        : 1.f;
    o.rotation_factor = rotation_hold_.apply(f, odometer_);
  }

  // Push
  const auto& u = p_.push;
  bool have_residual = false;
  if (!wheel_window_.empty()) {
    double sum = 0.0;
    for (const auto& s : wheel_window_) sum += s.net;
    const double mean = sum / wheel_window_.size();
    o.net_effort = mean;
    o.effort_residual = mean - (u.baseline + u.slope_gain * (-std::sin(pitch)));
    have_residual = true;
  }
  if (p_.proprioceptive && u.enable && !stale.odom) {
    const bool gated_in =
        have_residual && have_cmd_ && !stale.cmd && static_cast<float>(cmd_v_) >= u.min_cmd_v;
    const float f = gated_in ? interp(o.effort_residual, u.lo, u.hi, 1.f, u.floor) : 1.f;
    o.push_factor = push_hold_.apply(f, odometer_);
  }

  // Cover
  if (stale.image) {
    cover_fractions_.clear();
    cover_raw_ = 0.f;
    cover_expected_ = 0;
  }
  o.cover_raw = cover_raw_;
  o.cover_expected = cover_expected_;
  o.cover_compute_ms = cover_compute_ms_;
  if (!cover_fractions_.empty()) {
    float sum = 0.f;
    for (float f : cover_fractions_) sum += f;
    o.cover_avg = sum / cover_fractions_.size();
    if (p_.predictive && p_.cover.enable)
      o.cover_factor = interp(o.cover_avg, p_.cover.lo, p_.cover.hi, 1.f, p_.cover.floor);
  }

  // Stuck. A stale or too-short odometry window reads as speed 0, so it must not count as standing still.
  {
    const auto& k = p_.stuck;
    const bool candidate = k.enable && finite(now) && !stale.odom && speed_valid_ && have_cmd_ && !stale.cmd &&
                           std::fabs(static_cast<float>(cmd_v_)) >= k.min_cmd_v &&
                           static_cast<float>(speed_) < k.max_body_speed;
    if (candidate && !stuck_candidate_) stuck_since_ = now;
    stuck_candidate_ = candidate;
    o.stuck_s = candidate ? static_cast<float>(now - stuck_since_) : 0.f;
    o.stuck = candidate && now - stuck_since_ >= k.dwell_s;
  }

  // Output stage: falls at once, recovers at release_rate.
  float raw = 1.f;
  uint8_t lowest = kNone;
  const std::pair<float, uint8_t> rules[] = {{o.tilt_factor, kTilt},
                                             {o.rotation_factor, kRotation},
                                             {o.push_factor, kPush},
                                             {o.cover_factor, kCover}};
  for (const auto& rule : rules)
    if (rule.first < raw) {
      raw = rule.first;
      lowest = rule.second;
    }

  const double dt = have_tick_ ? std::max(0.0, std::min(0.5, now - last_tick_)) : 0.0;
  if (finite(now)) {
    last_tick_ = now;
    have_tick_ = true;
  }
  const float step = p_.release_rate * static_cast<float>(finite(dt) ? dt : 0.0);
  factor_ = raw < factor_ ? raw : std::min(raw, factor_ + step);
  factor_ = std::max(0.f, std::min(1.f, factor_));

  if (raw < 1.f) last_limiter_ = lowest;
  else if (factor_ >= 1.f) last_limiter_ = kNone;
  o.speed_factor = factor_;
  o.limiting = last_limiter_;
  return o;
}

}  // namespace overwatch
