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
                      double* wheel_speed, double* abs_effort, double* wheel_yaw_speed) {
  if (velocity.size() < names.size() || effort.size() < names.size()) return false;
  double net = 0.0, speed = 0.0, abs_sum = 0.0, yaw = 0.0;
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
    abs_sum += std::fabs(effort[i]);
    // left joints (103, 101) have sign +1: forward left wheels turn the body clockwise
    yaw += (sign > 0 ? -1.0 : 1.0) * sign * velocity[i];
    seen |= bit;
  }
  if (seen != 15) return false;
  *net_effort = net;
  *wheel_speed = speed / 4.0 * wheel_radius;
  if (abs_effort) *abs_effort = abs_sum;
  if (wheel_yaw_speed) *wheel_yaw_speed = yaw / 4.0 * wheel_radius;
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

bool SpeedGovernor::addOdom(double stamp, const Eigen::Vector2d& p, double roll, double pitch, double yaw) {
  if (!finite(stamp) || !p.allFinite() || !finite(roll) || !finite(pitch)) return false;
  if (finite(yaw)) {
    if (!yaw_hist_.empty() && stamp <= yaw_hist_.back().t) yaw_hist_.clear();
    if (yaw_hist_.empty()) have_yaw_ = false;
    if (!have_yaw_) {
      yaw_unwrapped_ = yaw;
      have_yaw_ = true;
    } else {
      const double prev = std::remainder(yaw_unwrapped_, 2.0 * M_PI);
      yaw_unwrapped_ += std::remainder(yaw - prev, 2.0 * M_PI);
    }
    yaw_hist_.push_back({stamp, yaw_unwrapped_});
    const double keep = std::max(p_.rotation_stuck.yaw_window_s, p_.rotation_stuck.progress_window_s) + 1.0;
    while (yaw_hist_.front().t < stamp - keep) yaw_hist_.pop_front();
  }
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

bool SpeedGovernor::addWheels(double stamp, double net_effort, double wheel_speed, double abs_effort,
                              double wheel_yaw_speed) {
  if (!finite(stamp) || !finite(net_effort) || !finite(wheel_speed)) return false;
  wheel_abs_effort_ = abs_effort;
  wheel_yaw_speed_ = wheel_yaw_speed;
  if (!wheel_window_.empty() && stamp <= wheel_window_.back().t) wheel_window_.clear();
  wheel_window_.push_back({stamp, net_effort});
  while (wheel_window_.front().t < stamp - p_.push.window_s) wheel_window_.pop_front();
  wheel_speed_ = wheel_speed;
  return true;
}

bool SpeedGovernor::addAutonomyCommand(double v, double w) {
  if (!finite(v) || !finite(w)) return false;
  auto_v_ = v;
  auto_w_ = w;
  have_auto_ = true;
  return true;
}

bool SpeedGovernor::addCommand(double v, double w) {
  if (!finite(v) || !finite(w)) return false;
  cmd_v_ = v;
  cmd_w_ = w;
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
            (stale.cmd ? kStaleCmd : 0) | (stale.image ? kStaleImage : 0) | (stale.autonomy ? kStaleAutonomy : 0);

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
    wheel_abs_effort_ = wheel_yaw_speed_ = NAN;
  }
  if (stale.odom) yaw_hist_.clear();
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

  // Rotation stuck. Pure pursuit rotates in place (or wiggles, once latched), the base executes pure
  // pursuit's command, and the wheels pivot or strain, but LIO yaw does not change.
  {
    const auto& q = p_.rotation_stuck;
    // yaw change over [t_last - window, t_last] of the yaw history; false without enough span
    auto yawChange = [this](double window, double* change, double* span) {
      if (yaw_hist_.size() < 2) return false;
      const double t1 = yaw_hist_.back().t;
      auto it = yaw_hist_.begin();
      while (it + 1 != yaw_hist_.end() && it->t < t1 - window) ++it;
      *span = t1 - it->t;
      *change = yaw_hist_.back().yaw - it->yaw;
      return *span >= 0.7 * window;
    };
    double dyaw = 0.0, span = 0.0;
    const bool rate_valid = !stale.odom && yawChange(q.yaw_window_s, &dyaw, &span);
    const double yaw_rate = rate_valid ? dyaw / span : 0.0;
    o.yaw_rate = static_cast<float>(yaw_rate);
    o.wheel_yaw_speed = finite(wheel_yaw_speed_) ? static_cast<float>(wheel_yaw_speed_) : 0.f;
    o.abs_effort = finite(wheel_abs_effort_) ? static_cast<float>(wheel_abs_effort_) : 0.f;

    const bool autonomy = have_auto_ && !stale.autonomy && have_cmd_ && !stale.cmd &&
                          std::fabs(cmd_v_ - auto_v_) <= p_.stuck.max_cmd_mismatch &&
                          std::fabs(cmd_w_ - auto_w_) <= q.max_cmd_w_mismatch;
    const bool turning = autonomy && std::fabs(auto_w_) >= q.min_cmd_w;

    if (!q.enable || !finite(now)) {
      rot_candidate_ = rot_latched_ = rot_escalated_ = rot_releasing_ = false;
    } else {
      if (!rot_latched_) {
        const bool pivoting = !stale.wheels && finite(wheel_yaw_speed_) && finite(wheel_abs_effort_) &&
                              (std::fabs(wheel_yaw_speed_) >= q.min_wheel_yaw_speed ||
                               wheel_abs_effort_ >= q.min_abs_effort);
        const bool candidate = turning && std::fabs(auto_v_) <= q.max_cmd_v && pivoting && rate_valid &&
                               std::fabs(yaw_rate) < q.max_yaw_rate;
        if (candidate && !rot_candidate_) rot_since_ = now;
        rot_candidate_ = candidate;
        if (candidate) o.rotation_stuck_s = static_cast<float>(now - rot_since_);
        if (candidate && now - rot_since_ >= q.dwell_s) {
          rot_latched_ = true;
          rot_candidate_ = false;
          rot_releasing_ = false;
          rot_since_ = now;
        }
      }
      if (rot_latched_) {
        // Brief command mismatches at the wiggle's direction changes must not release it.
        if (turning) {
          rot_releasing_ = false;
        } else if (!rot_releasing_) {
          rot_releasing_ = true;
          rot_release_since_ = now;
        }
        if (rot_releasing_ && now - rot_release_since_ >= q.release_s) {
          rot_latched_ = false;
        } else {
          o.rotation_stuck_s = static_cast<float>(now - rot_since_);
          double progress = 0.0, pspan = 0.0;
          if (now - rot_since_ >= q.escalate_after_s && !stale.odom &&
              yawChange(q.progress_window_s, &progress, &pspan) &&
              std::fabs(progress) < q.min_progress_deg * kDeg) {
            rot_latched_ = false;
            rot_escalated_ = true;
          }
        }
      }
      // Escalated: stuck until pure pursuit stops turning (spam cancels it on the rising edge).
      if (rot_escalated_ && !turning) rot_escalated_ = false;
    }
    o.rotation_stuck = rot_latched_;
    o.rotation_escalated = rot_escalated_;
  }

  // Stuck. A stale or too-short odometry window reads as speed 0, so it must not count as standing still.
  // An operator hold is not stuck: the joystick (base command differs from pure pursuit's) or motors off
  // (no torque, wheels still).
  {
    const auto& k = p_.stuck;
    const bool autonomy = have_auto_ && !stale.autonomy &&
                          std::fabs(static_cast<float>(cmd_v_ - auto_v_)) <= k.max_cmd_mismatch;
    const bool driving = !stale.wheels && have_residual &&
                         (o.effort_residual >= k.min_effort_residual ||
                          std::fabs(o.wheel_speed) >= k.min_wheel_speed);
    // The wiggle's forward/back moves would look like a linear stuck: suppressed while it runs.
    const bool candidate = k.enable && finite(now) && !stale.odom && speed_valid_ && have_cmd_ && !stale.cmd &&
                           autonomy && driving && !rot_latched_ && !rot_escalated_ &&
                           std::fabs(static_cast<float>(cmd_v_)) >= k.min_cmd_v &&
                           static_cast<float>(speed_) < k.max_body_speed;
    if (candidate && !stuck_candidate_) stuck_since_ = now;
    stuck_candidate_ = candidate;
    o.stuck_s = candidate ? static_cast<float>(now - stuck_since_) : 0.f;
    o.stuck = (candidate && now - stuck_since_ >= k.dwell_s) || rot_escalated_;
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
