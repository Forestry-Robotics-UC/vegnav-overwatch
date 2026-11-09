#pragma once
// ROS-free core of the overwatch speed governor: IMU, LIO pose, wheels, command and the range image ->
// one speed factor in [floor, 1]. It only ever slows the robot, and stale or missing inputs make a rule
// return 1 (fail open).
//
// Units at this interface: stamps in s, angles and rates in rad, rad/s; rule thresholds are in degrees.
// Pitch positive = nose down.

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace overwatch {

// Linear from (x0, y0) to (x1, y1), clamped at both ends. NaN gives y0, which fails open for every rule.
float interp(float x, float x0, float x1, float y0, float y1);

// Gyro from the IMU frame to base_link, for an IMU mounted yawed by yaw_rad about z.
Eigen::Vector2d imuToBase(double ix, double iy, double yaw_rad);

// Signed sum of the four MD80 efforts (+ = forward) and mean forward wheel speed [m/s]. Joint order is
// not fixed. False unless all four joints are present and finite. Optionally also the sum of |effort|
// and the wheels' turning speed (right minus left over 2, + = CCW) [m/s], both of which a pivot shows
// and the signed sums cancel.
bool wheelsFromJoints(const std::vector<std::string>& names, const std::vector<double>& velocity,
                      const std::vector<double>& effort, double wheel_radius, double* net_effort,
                      double* wheel_speed, double* abs_effort = nullptr, double* wheel_yaw_speed = nullptr);

// Keeps a slowdown until the robot has travelled hold_m past the last trigger.
class DistanceHold {
 public:
  explicit DistanceHold(double hold_m = 0.8) : hold_m_(hold_m) {}
  float apply(float f, double odo);
  void reset(double odo);

 private:
  double hold_m_;
  float held_ = 1.f;
  double anchor_ = 0.0;
};

enum Limiter : uint8_t { kNone = 0, kTilt = 1, kRotation = 2, kPush = 3, kCover = 4 };
enum StaleBit : uint8_t { kStaleImu = 1, kStaleOdom = 2, kStaleWheels = 4, kStaleCmd = 8, kStaleImage = 16,
                         kStaleAutonomy = 32 };

struct Params {
  float release_rate = 0.25f;           // factor per second
  double odometer_deadband_m = 0.05;    // LIO jitter below this does not accumulate
  double speed_window_s = 0.5;
  double speed_min_span_s = 0.3;
  bool proprioceptive = true;
  bool predictive = true;

  struct Tilt {
    bool enable = true;
    float roll_lo = 5.f, roll_hi = 20.f, roll_floor = 0.35f;
    float pitch_lo = 10.f, pitch_hi = 25.f, pitch_floor = 0.5f;
  } tilt;

  struct Rotation {
    bool enable = true;
    double window_s = 0.5;
    double max_dt_s = 0.1;              // a longer IMU gap restarts the window
    float lo = 5.f, hi = 9.f, floor = 0.3f;
    float min_speed = 0.1f;             // m/s, larger of LIO and wheel speed
    double hold_m = 0.8;
  } rotation;

  struct Push {
    bool enable = true;
    double window_s = 0.2;
    float baseline = 11.6f, slope_gain = 59.4f;
    float lo = 18.f, hi = 28.f, floor = 0.3f;
    float min_cmd_v = 0.1f;
    double hold_m = 0.8;
  } push;

  // Rotation stuck: pure pursuit rotates in place, the wheels pivot, the body does not turn. Reported on
  // /overwatch/rotation_stuck, where pure pursuit switches its rotation to a wiggle; latched while pure
  // pursuit keeps turning (rotating or wiggling). If the wiggle makes no progress, escalates to stuck
  // (back up and replan). Never changes the factor.
  struct RotationStuck {
    bool enable = true;
    float min_cmd_w = 0.5f;             // rad/s, |pure pursuit turn rate|
    float max_cmd_v = 0.05f;            // m/s, |pure pursuit speed| to start: an in-place rotation
    double yaw_window_s = 1.0;          // LIO yaw rate over this window
    float max_yaw_rate = 0.02f;         // rad/s; a slow but real pivot on Nov 6 was 0.04-0.10
    double dwell_s = 2.0;
    float max_cmd_w_mismatch = 0.1f;    // rad/s, |base - pure pursuit| (joystick excluded)
    float min_wheel_yaw_speed = 0.05f;  // m/s, |wheel pivot speed| ...
    float min_abs_effort = 25.f;        // ... or sum |effort|: the drive is trying (motors off excluded)
    double release_s = 0.5;             // pure pursuit must stop turning this long to release
    double escalate_after_s = 15.0;     // latched this long and ...
    double progress_window_s = 8.0;     // ... turned less than min_progress_deg over this window:
    float min_progress_deg = 15.f;      //     report stuck instead
  } rotation_stuck;

  // Stuck: the drive is commanded but the body does not move. Reported only; it never changes the factor.
  struct Stuck {
    bool enable = true;
    float min_cmd_v = 0.1f;             // m/s, |commanded linear speed|
    float max_body_speed = 0.05f;       // m/s, LIO speed
    double dwell_s = 1.0;               // the condition must hold this long
    // Not an operator hold: the drive is trying (pushing or spinning), and the command reaching the base is
    // pure pursuit's (not the joystick). Without wheels or pure pursuit's command it never fires.
    float min_effort_residual = 8.f;    // push residual, same units as Push
    float min_wheel_speed = 0.1f;       // m/s, |mean wheel speed|
    float max_cmd_mismatch = 0.05f;     // m/s, |base command - pure pursuit command|
  } stuck;

  struct Cover {
    bool enable = true;
    float half_sector_deg = 45.f;
    float max_ray_z = -0.02f;           // levelled rays above this never had ground to hit
    int min_rays = 100;
    int window_sweeps = 3;
    float lo = 0.03f, hi = 0.2f, floor = 0.6f;
  } cover;
};

// Decided by the caller from arrival times.
struct Stale {
  bool imu = false, odom = false, wheels = false, cmd = false, image = false;
  bool autonomy = false;  // pure pursuit's own command (only the stuck rule reads it)
};

struct Output {
  float speed_factor = 1.f;
  uint8_t limiting = kNone;
  float tilt_factor = 1.f, rotation_factor = 1.f, push_factor = 1.f, cover_factor = 1.f;
  float roll_deg = 0.f, pitch_deg = 0.f;    // the attitude in use: 0 while odom is stale
  float droll_deg = 0.f, dpitch_deg = 0.f, rotation_deg = 0.f;
  float net_effort = 0.f;                   // mean over the push window
  float effort_residual = 0.f;
  float cover_raw = 0.f, cover_avg = 0.f;
  uint32_t cover_expected = 0;
  float lio_speed = 0.f, wheel_speed = 0.f, odometer = 0.f;
  float cover_compute_ms = 0.f;
  uint8_t stale = 0;
  bool stuck = false;
  float stuck_s = 0.f;                      // how long the stuck condition has held, 0 when it does not
  bool rotation_stuck = false;
  float rotation_stuck_s = 0.f;             // candidate time before the latch, then time latched
  bool rotation_escalated = false;          // stuck comes from a wiggle that made no progress
  float yaw_rate = 0.f;                     // LIO, over the rotation-stuck window [rad/s]
  float wheel_yaw_speed = 0.f, abs_effort = 0.f;
};

class SpeedGovernor {
 public:
  explicit SpeedGovernor(const Params& p = Params());

  // Each returns false when the input was rejected, so the caller does not count it as a live topic.
  bool addImu(double stamp, double gx, double gy);  // base-frame gyro
  // yaw NaN: unknown (the rotation-stuck rule then never fires)
  bool addOdom(double stamp, const Eigen::Vector2d& p, double roll, double pitch, double yaw = NAN);
  bool addWheels(double stamp, double net_effort, double wheel_speed, double abs_effort = NAN,
                 double wheel_yaw_speed = NAN);
  bool addCommand(double v, double w = 0.0);
  bool addAutonomyCommand(double v, double w = 0.0);  // pure pursuit's output, before the mux
  // mono16, row-major, rows * step_bytes readable. Only zero / non-zero matters, so byte order does not.
  bool addRangeImage(const uint8_t* data, int rows, int cols, size_t step_bytes);
  bool setBeamAltitudes(const std::vector<float>& deg);

  Output update(double now, const Stale& stale);

 private:
  struct RotSample { double t, roll, pitch; };
  struct OdomSample { double t; Eigen::Vector2d p; };
  struct WheelSample { double t, net; };

  void rebuildRays(int rows, int cols);

  Params p_;

  // Rotation: window of cumulative angles [deg], plus the peak since the last tick.
  std::deque<RotSample> rot_window_;
  double cum_roll_ = 0.0, cum_pitch_ = 0.0;
  double imu_stamp_ = 0.0;
  bool have_imu_stamp_ = false;
  double cur_droll_ = 0.0, cur_dpitch_ = 0.0;
  double peak_droll_ = 0.0, peak_dpitch_ = 0.0;
  DistanceHold rotation_hold_;

  // Odometry
  std::deque<OdomSample> odom_window_;
  Eigen::Vector2d last_counted_ = Eigen::Vector2d::Zero();
  bool have_odom_ = false;
  double odometer_ = 0.0, speed_ = 0.0;
  bool speed_valid_ = false;  // speed_ spans at least speed_min_span_s
  double roll_ = 0.0, pitch_ = 0.0;
  bool attitude_valid_ = false;  // cleared by a stale tick, so the cover rule levels with zero attitude

  // Push
  std::deque<WheelSample> wheel_window_;
  double wheel_speed_ = 0.0;
  double cmd_v_ = 0.0;
  bool have_cmd_ = false;
  double auto_v_ = 0.0;
  bool have_auto_ = false;
  DistanceHold push_hold_;

  // Cover
  std::vector<float> altitudes_deg_;
  std::vector<Eigen::Vector3f> rays_;  // sensor frame, row-major
  int ray_rows_ = 0, ray_cols_ = 0;
  std::deque<float> cover_fractions_;
  float cover_raw_ = 0.f;
  uint32_t cover_expected_ = 0;
  float cover_compute_ms_ = 0.f;

  // Stuck
  double stuck_since_ = 0.0;
  bool stuck_candidate_ = false;

  // Rotation stuck
  struct YawSample { double t, yaw; };
  std::deque<YawSample> yaw_hist_;          // unwrapped
  double yaw_unwrapped_ = 0.0;
  bool have_yaw_ = false;
  double cmd_w_ = 0.0, auto_w_ = 0.0;
  double wheel_abs_effort_ = NAN, wheel_yaw_speed_ = NAN;
  bool rot_candidate_ = false, rot_latched_ = false, rot_escalated_ = false;
  double rot_since_ = 0.0, rot_release_since_ = 0.0;
  bool rot_releasing_ = false;

  // Output stage
  float factor_ = 1.f;
  uint8_t last_limiter_ = kNone;
  double last_tick_ = 0.0;
  bool have_tick_ = false;
};

}  // namespace overwatch
