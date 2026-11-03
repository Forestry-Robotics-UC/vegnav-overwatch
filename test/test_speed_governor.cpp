// Core of the speed governor on synthetic inputs. The numbers checked here are the spec's.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <random>

#include "overwatch/speed_governor.h"

using namespace overwatch;

namespace {
constexpr double kD2R = M_PI / 180.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kTol = 1e-4f;

enum Rule { kOnlyTilt, kOnlyRotation, kOnlyPush, kOnlyCover };

Params only(Rule rule) {
  Params p;
  p.tilt.enable = rule == kOnlyTilt;
  p.rotation.enable = rule == kOnlyRotation;
  p.push.enable = rule == kOnlyPush;
  p.cover.enable = rule == kOnlyCover;
  return p;
}

// Odometry at 10 Hz in a straight line, keeping its own clock.
struct OdomFeed {
  SpeedGovernor& g;
  double t = 0.0, x = 0.0;
  void step(double dx, double roll = 0.0, double pitch = 0.0) {
    x += dx;
    t += 0.1;
    ASSERT_TRUE(g.addOdom(t, Eigen::Vector2d(x, 0.0), roll, pitch));
  }
  void cruise(double speed, double duration) {
    const int n = std::lround(duration * 10.0);
    for (int i = 0; i < n; ++i) step(speed * 0.1);
  }
};

// Gyro at 500 Hz, keeping its own clock. Rates in deg/s.
struct GyroFeed {
  SpeedGovernor& g;
  double t = 100.0;
  void run(double roll_rate, double pitch_rate, double duration) {
    const int n = std::lround(duration * 500.0);
    for (int i = 0; i < n; ++i) {
      ASSERT_TRUE(g.addImu(t, roll_rate * kD2R, pitch_rate * kD2R));
      t += 0.002;
    }
  }
};

Output tiltAt(double roll_deg, double pitch_deg) {
  SpeedGovernor g(only(kOnlyTilt));
  EXPECT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), roll_deg * kD2R, pitch_deg * kD2R));
  return g.update(0.0, Stale());
}

// A governor already driving at 0.5 m/s, with the gyro rule alone.
struct RotationRig {
  SpeedGovernor g{only(kOnlyRotation)};
  OdomFeed odom{g};
  GyroFeed gyro{g};
  RotationRig() { odom.cruise(0.5, 1.0); }
};

// A governor with the push rule alone, with a window full of constant net effort.
struct PushRig {
  SpeedGovernor g{only(kOnlyPush)};
  OdomFeed odom{g};
  double wheel_t = 0.0;
  void wheels(double net, double duration = 0.3) {
    const int n = std::lround(duration * 100.0);
    for (int i = 0; i < n; ++i) {
      ASSERT_TRUE(g.addWheels(wheel_t, net, 0.5));
      wheel_t += 0.01;
    }
  }
};

Output pushAt(double net, double pitch_deg, double v) {
  PushRig r;
  r.odom.step(0.0, 0.0, pitch_deg * kD2R);
  r.wheels(net);
  EXPECT_TRUE(r.g.addCommand(v));
  return r.g.update(0.0, Stale());
}

constexpr float kFallbackTop = 21.25f, kFallbackBottom = -21.39f;

double fallbackElevation(int r, int rows) {
  return kFallbackTop + (kFallbackBottom - kFallbackTop) * static_cast<double>(r) / (rows - 1);
}

double azimuth(int c, int cols) {
  return 180.0 - 360.0 * c / cols;
}

// Wrapped to (-180, 180], for selecting columns by bearing.
double bearing(int c, int cols) {
  double a = azimuth(c, cols);
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// Mono16 image. The pad bytes at the end of each row must never be read as pixels.
struct Image {
  int rows, cols;
  size_t step;
  std::vector<uint8_t> data;
  Image(int rows_in, int cols_in, uint16_t fill = 1000, int pad_bytes = 0, uint8_t pad_value = 0)
      : rows(rows_in), cols(cols_in), step(2 * cols_in + pad_bytes), data(rows_in * step, pad_value) {
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) set(r, c, fill);
  }
  void set(int r, int c, uint16_t v) {
    data[r * step + 2 * c] = v & 0xff;
    data[r * step + 2 * c + 1] = v >> 8;
  }
  void zeroWhere(const std::function<bool(int, int)>& pred) {
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c)
        if (pred(r, c)) set(r, c, 0);
  }
};

// Independent count of the rays the cover rule should expect, with the fallback elevations.
uint32_t bruteExpected(int rows, int cols, double roll_deg, double pitch_deg) {
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(pitch_deg * kD2R, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(roll_deg * kD2R, Eigen::Vector3d::UnitX())).toRotationMatrix();
  uint32_t n = 0;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const double el = fallbackElevation(r, rows) * kD2R, az = azimuth(c, cols) * kD2R;
      const Eigen::Vector3d d =
          R * Eigen::Vector3d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
      if (d.z() < -0.02 && std::fabs(std::atan2(d.y(), d.x())) < 45.0 * kD2R) ++n;
    }
  return n;
}

Output coverOf(const Image& im, double roll_deg = 0.0, double pitch_deg = 0.0) {
  SpeedGovernor g(only(kOnlyCover));
  EXPECT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), roll_deg * kD2R, pitch_deg * kD2R));
  EXPECT_TRUE(g.addRangeImage(im.data.data(), im.rows, im.cols, im.step));
  return g.update(0.0, Stale());
}
}  // namespace

// ---------- interp ----------

TEST(Interp, ClampsAtBothEnds) {
  EXPECT_FLOAT_EQ(interp(-100.f, 5.f, 20.f, 1.f, 0.35f), 1.f);
  EXPECT_FLOAT_EQ(interp(5.f, 5.f, 20.f, 1.f, 0.35f), 1.f);
  EXPECT_FLOAT_EQ(interp(20.f, 5.f, 20.f, 1.f, 0.35f), 0.35f);
  EXPECT_FLOAT_EQ(interp(1000.f, 5.f, 20.f, 1.f, 0.35f), 0.35f);
}

TEST(Interp, LinearInside) {
  EXPECT_FLOAT_EQ(interp(12.5f, 5.f, 20.f, 1.f, 0.35f), 0.675f);
  EXPECT_FLOAT_EQ(interp(1.f, 0.f, 2.f, 0.f, 10.f), 5.f);
}

TEST(Interp, NaNFailsOpen) {
  EXPECT_FLOAT_EQ(interp(std::numeric_limits<float>::quiet_NaN(), 5.f, 20.f, 1.f, 0.35f), 1.f);
}

// ---------- tilt ----------

TEST(Tilt, RollValues) {
  EXPECT_NEAR(tiltAt(0, 0).tilt_factor, 1.f, kTol);
  EXPECT_NEAR(tiltAt(5, 0).tilt_factor, 1.f, kTol);
  EXPECT_NEAR(tiltAt(12.5, 0).tilt_factor, 0.675f, kTol);
  EXPECT_NEAR(tiltAt(20, 0).tilt_factor, 0.35f, kTol);
  EXPECT_NEAR(tiltAt(40, 0).tilt_factor, 0.35f, kTol);
}

TEST(Tilt, PitchValues) {
  EXPECT_NEAR(tiltAt(0, 10).tilt_factor, 1.f, kTol);
  EXPECT_NEAR(tiltAt(0, 17.5).tilt_factor, 0.75f, kTol);
  EXPECT_NEAR(tiltAt(0, 25).tilt_factor, 0.5f, kTol);
  EXPECT_NEAR(tiltAt(0, 40).tilt_factor, 0.5f, kTol);
}

TEST(Tilt, NegativeAnglesAreSymmetric) {
  EXPECT_NEAR(tiltAt(-12.5, 0).tilt_factor, 0.675f, kTol);
  EXPECT_NEAR(tiltAt(-20, 0).tilt_factor, 0.35f, kTol);
  EXPECT_NEAR(tiltAt(0, -17.5).tilt_factor, 0.75f, kTol);
  EXPECT_NEAR(tiltAt(0, -25).tilt_factor, 0.5f, kTol);
}

TEST(Tilt, TakesTheMinOfRollAndPitch) {
  EXPECT_NEAR(tiltAt(12.5, 25).tilt_factor, 0.5f, kTol);    // pitch wins
  EXPECT_NEAR(tiltAt(20, 17.5).tilt_factor, 0.35f, kTol);   // roll wins
  EXPECT_NEAR(tiltAt(-20, -25).tilt_factor, 0.35f, kTol);
}

TEST(Tilt, HasNoHold) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  EXPECT_NEAR(g.update(0.0, Stale()).tilt_factor, 0.35f, kTol);
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 0.0, 0.0));
  EXPECT_NEAR(g.update(0.1, Stale()).tilt_factor, 1.f, kTol);
}

// ---------- rotation ----------

TEST(Rotation, Constant20DegPerSecondGivesFloor) {
  RotationRig r;
  r.gyro.run(20.0, 0.0, 0.5);
  const Output o = r.g.update(1.0, Stale());
  EXPECT_NEAR(o.rotation_deg, 10.f, 0.1f);
  EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
}

TEST(Rotation, ThresholdsAreInterpolated) {
  RotationRig r;
  r.gyro.run(14.0, 0.0, 0.5);  // about 7 deg, half-way
  const Output o = r.g.update(1.0, Stale());
  EXPECT_NEAR(o.rotation_factor, 1.f + (0.3f - 1.f) * (o.rotation_deg - 5.f) / 4.f, kTol);
  EXPECT_GT(o.rotation_factor, 0.3f);
  EXPECT_LT(o.rotation_factor, 1.f);
}

TEST(Rotation, ZeroMeanVibrationIsIgnored) {
  RotationRig r;
  for (int i = 0; i < 1000; ++i) {  // +-30 deg/s square wave at 20 Hz, half-periods 13, 12, 12, 13 samples
    const int k = i % 50;
    const double rate = k < 13 || (k >= 25 && k < 37) ? 30.0 : -30.0;
    r.gyro.run(rate, -rate, 0.002);
  }
  Output o = r.g.update(1.0, Stale());
  EXPECT_LT(o.rotation_deg, 1.f);  // swings 0.75 deg
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
  for (int i = 0; i < 1000; ++i) {  // and as a sine
    const double rate = 30.0 * std::sin(2.0 * M_PI * 20.0 * i * 0.002);
    r.gyro.run(rate, -rate, 0.002);
  }
  o = r.g.update(1.05, Stale());
  EXPECT_LT(o.rotation_deg, 1.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(Rotation, SlowRotationIsIgnored) {
  RotationRig r;
  r.gyro.run(2.0, 2.0, 5.0);
  const Output o = r.g.update(1.0, Stale());
  EXPECT_NEAR(o.rotation_deg, 1.f, 0.02f);  // the window keeps only the last 0.5 s
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(Rotation, BriefPeakBetweenTicksIsLatched) {
  RotationRig r;
  r.gyro.run(500.0, 0.0, 0.024);   // +11 deg in 24 ms
  r.gyro.run(-500.0, 0.0, 0.024);  // and back, so at the tick the window reads about 0
  Output o = r.g.update(1.0, Stale());
  EXPECT_NEAR(o.rotation_deg, 11.f, 0.5f);
  EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
  // The latch is consumed by the tick; only the distance hold keeps the factor down afterwards.
  r.gyro.run(0.0, 0.0, 0.05);
  o = r.g.update(1.05, Stale());
  EXPECT_LT(o.rotation_deg, 1.5f);
  EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
}

TEST(Rotation, BothAxesTrigger) {
  {
    RotationRig r;
    r.gyro.run(20.0, 0.0, 0.5);
    const Output o = r.g.update(1.0, Stale());
    EXPECT_GT(o.droll_deg, 9.f);
    EXPECT_NEAR(o.dpitch_deg, 0.f, kTol);
    EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
  }
  {
    RotationRig r;
    r.gyro.run(0.0, -20.0, 0.5);
    const Output o = r.g.update(1.0, Stale());
    EXPECT_NEAR(o.droll_deg, 0.f, kTol);
    EXPECT_LT(o.dpitch_deg, -9.f);
    EXPECT_NEAR(o.rotation_deg, -o.dpitch_deg, kTol);
    EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
  }
}

TEST(Rotation, BelowMinSpeedGivesOne) {
  SpeedGovernor g(only(kOnlyRotation));
  OdomFeed odom{g};
  GyroFeed gyro{g};
  odom.cruise(0.05, 1.0);
  gyro.run(20.0, 0.0, 0.5);
  const Output o = g.update(1.0, Stale());
  EXPECT_LT(o.lio_speed, 0.1f);
  EXPECT_GT(o.rotation_deg, 9.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(Rotation, WheelSpeedOpensTheGateWhenLioCollapses) {
  // The rock: the body has stopped, so LIO reads near 0 while the wheels still turn at 0.5 m/s.
  SpeedGovernor g(only(kOnlyRotation));
  OdomFeed odom{g};
  GyroFeed gyro{g};
  odom.cruise(0.0, 1.0);
  ASSERT_TRUE(g.addWheels(1.0, 0.0, 0.5));
  gyro.run(20.0, 0.0, 0.5);
  const Output o = g.update(1.0, Stale());
  EXPECT_LT(o.lio_speed, 0.1f);
  EXPECT_FLOAT_EQ(o.wheel_speed, 0.5f);
  EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
}

TEST(Rotation, WheelSpeedOpensTheGateInReverse) {
  SpeedGovernor g(only(kOnlyRotation));
  OdomFeed odom{g};
  GyroFeed gyro{g};
  odom.cruise(0.0, 1.0);
  ASSERT_TRUE(g.addWheels(1.0, 0.0, -0.5));
  gyro.run(20.0, 0.0, 0.5);
  EXPECT_NEAR(g.update(1.0, Stale()).rotation_factor, 0.3f, kTol);
}

TEST(Rotation, PivotKeepsTheGateClosed) {
  // Every joint turning hard, as in place: left forward and right backward net to a small mean speed.
  double net = 0.0, speed = 1.0;
  ASSERT_TRUE(wheelsFromJoints({"Joint 100", "Joint 101", "Joint 102", "Joint 103"}, {6.0, 6.2, 6.0, 6.2},
                               {5.0, 5.0, 5.0, 5.0}, 0.13, &net, &speed));
  ASSERT_LT(std::fabs(speed), 0.1);
  SpeedGovernor g(only(kOnlyRotation));
  OdomFeed odom{g};
  GyroFeed gyro{g};
  odom.cruise(0.0, 1.0);
  ASSERT_TRUE(g.addWheels(1.0, net, speed));
  gyro.run(20.0, 0.0, 0.5);
  const Output o = g.update(1.0, Stale());
  EXPECT_LT(o.lio_speed, 0.1f);
  EXPECT_GT(o.rotation_deg, 9.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(Rotation, StaleWheelsFallBackToLioFromTheFirstTick) {
  Stale s;
  s.wheels = true;
  {
    // LIO under min_speed: the wheel speed fed just before must not open the gate on this first stale tick.
    SpeedGovernor g(only(kOnlyRotation));
    OdomFeed odom{g};
    GyroFeed gyro{g};
    odom.cruise(0.0, 1.0);
    ASSERT_TRUE(g.addWheels(1.0, 0.0, 0.5));
    gyro.run(20.0, 0.0, 0.5);
    const Output o = g.update(1.0, s);
    EXPECT_FLOAT_EQ(o.wheel_speed, 0.f);
    EXPECT_GT(o.rotation_deg, 9.f);
    EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
  }
  {
    // LIO over min_speed still opens it with no wheels at all.
    RotationRig r;
    ASSERT_TRUE(r.g.addWheels(1.0, 0.0, 0.5));
    r.gyro.run(20.0, 0.0, 0.5);
    const Output o = r.g.update(1.0, s);
    EXPECT_FLOAT_EQ(o.wheel_speed, 0.f);
    EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
  }
}

TEST(Rotation, TimeGapRestartsWindow) {
  RotationRig r;
  r.gyro.run(19.0, 0.0, 0.25);  // under 5 deg on each side of the gap, near 9.5 if joined
  r.gyro.t += 0.2;
  r.gyro.run(19.0, 0.0, 0.25);
  const Output o = r.g.update(1.0, Stale());
  EXPECT_LT(o.rotation_deg, 5.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(Rotation, BackwardStampRestartsWindow) {
  RotationRig r;
  r.gyro.run(19.0, 0.0, 0.25);
  r.gyro.t -= 1.0;
  r.gyro.run(19.0, 0.0, 0.25);
  const Output o = r.g.update(1.0, Stale());
  EXPECT_LT(o.rotation_deg, 5.f);
}

TEST(Rotation, GapJustUnderLimitDoesNotRestart) {
  RotationRig r;
  r.gyro.run(20.0, 0.0, 0.2);
  r.gyro.t += 0.05;  // dt = 0.052 <= 0.1, integrated at the sample's own rate
  r.gyro.run(20.0, 0.0, 0.2);
  const Output o = r.g.update(1.0, Stale());
  EXPECT_NEAR(o.rotation_deg, 20.0 * 0.45, 0.1);
}

TEST(Rotation, NaNGyroIsIgnoredAndRestartsWindow) {
  RotationRig r;
  r.gyro.run(19.0, 0.0, 0.25);
  EXPECT_FALSE(r.g.addImu(r.gyro.t, kNaN, 0.0));
  EXPECT_FALSE(r.g.addImu(r.gyro.t, 0.0, kInf));
  EXPECT_FALSE(r.g.addImu(kNaN, 0.0, 0.0));
  r.gyro.run(19.0, 0.0, 0.25);
  const Output o = r.g.update(1.0, Stale());
  EXPECT_TRUE(std::isfinite(o.rotation_deg));
  EXPECT_LT(o.rotation_deg, 5.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(Rotation, ImuYawMapping) {
  const Eigen::Vector2d x = imuToBase(1.0, 0.0, 90 * kD2R);  // bx = cos(y)*ix - sin(y)*iy
  EXPECT_NEAR(x.x(), 0.0, 1e-12);
  EXPECT_NEAR(x.y(), 1.0, 1e-12);
  const Eigen::Vector2d y = imuToBase(0.0, 1.0, 90 * kD2R);
  EXPECT_NEAR(y.x(), -1.0, 1e-12);
  EXPECT_NEAR(y.y(), 0.0, 1e-12);
  const Eigen::Vector2d same = imuToBase(0.3, -0.7, 0.0);
  EXPECT_NEAR(same.x(), 0.3, 1e-12);
  EXPECT_NEAR(same.y(), -0.7, 1e-12);
}

// ---------- distance hold ----------

TEST(DistanceHoldTest, StaysDownWithNoTravel) {
  DistanceHold h(0.8);
  EXPECT_FLOAT_EQ(h.apply(0.3f, 10.0), 0.3f);
  for (int i = 0; i < 10000; ++i) EXPECT_FLOAT_EQ(h.apply(1.f, 10.0), 0.3f);
}

TEST(DistanceHoldTest, ReleasesOnlyAfterHoldDistance) {
  DistanceHold h(0.8);
  h.apply(0.3f, 10.0);
  EXPECT_FLOAT_EQ(h.apply(1.f, 10.4), 0.3f);
  EXPECT_FLOAT_EQ(h.apply(1.f, 10.79), 0.3f);
  EXPECT_FLOAT_EQ(h.apply(1.f, 10.81), 1.f);
}

TEST(DistanceHoldTest, EqualRetriggerRefreshesAnchor) {
  DistanceHold h(0.8);
  h.apply(0.3f, 10.0);
  EXPECT_FLOAT_EQ(h.apply(0.3f, 10.5), 0.3f);
  EXPECT_FLOAT_EQ(h.apply(1.f, 11.0), 0.3f);   // 0.5 m past the refreshed anchor, 1.0 m past the old
  EXPECT_FLOAT_EQ(h.apply(1.f, 11.31), 1.f);
}

TEST(DistanceHoldTest, LowerFactorTakesOverAtOnce) {
  DistanceHold h(0.8);
  h.apply(0.6f, 10.0);
  EXPECT_FLOAT_EQ(h.apply(0.3f, 10.1), 0.3f);
}

TEST(DistanceHoldTest, WeakerTriggerDoesNotRefreshAndExpiryTakesCurrentValue) {
  DistanceHold h(0.8);
  h.apply(0.3f, 10.0);
  EXPECT_FLOAT_EQ(h.apply(0.7f, 10.5), 0.3f);
  EXPECT_FLOAT_EQ(h.apply(0.7f, 10.81), 0.7f);  // anchor stayed at 10.0, expiry adopts the current f
  EXPECT_FLOAT_EQ(h.apply(1.f, 11.0), 0.7f);    // and it holds again from there
  EXPECT_FLOAT_EQ(h.apply(1.f, 11.62), 1.f);
}

TEST(DistanceHoldTest, ResetReleases) {
  DistanceHold h(0.8);
  h.apply(0.3f, 10.0);
  h.reset(10.0);
  EXPECT_FLOAT_EQ(h.apply(1.f, 10.0), 1.f);
}

TEST(Rotation, HoldStaysWithTimeAndReleasesOnOdometer) {
  RotationRig r;
  r.gyro.run(20.0, 0.0, 0.5);
  double now = 2.0;
  EXPECT_NEAR(r.g.update(now, Stale()).rotation_factor, 0.3f, kTol);
  r.gyro.run(0.0, 0.0, 0.6);  // flush the gyro window so the rule's own input is back to 1
  r.odom.cruise(0.0, 1.0);    // and the speed window
  r.g.update(now, Stale());   // consumes the latch

  // Time passes, the robot stands: stays down.
  for (int i = 0; i < 400; ++i) {
    r.odom.step(0.0);
    r.gyro.run(0.0, 0.0, 0.05);
    now += 0.05;
    const Output o = r.g.update(now, Stale());
    EXPECT_NEAR(o.rotation_factor, 0.3f, kTol);
    EXPECT_FLOAT_EQ(o.lio_speed, 0.f);
    EXPECT_LT(o.rotation_deg, 0.01f);
  }

  // Now it drives. 0.7 m past the anchor is still held, 0.9 m is not.
  for (int i = 0; i < 7; ++i) r.odom.step(0.1);
  EXPECT_NEAR(r.g.update(now += 0.05, Stale()).rotation_factor, 0.3f, kTol);
  for (int i = 0; i < 2; ++i) r.odom.step(0.1);
  EXPECT_FLOAT_EQ(r.g.update(now += 0.05, Stale()).rotation_factor, 1.f);
}

// ---------- odometer and speed ----------

TEST(Odometer, JitterDoesNotAccumulate) {
  SpeedGovernor g(only(kOnlyTilt));
  double t = 0.0;
  for (int i = 0; i < 1000; ++i) {  // +-2 cm along one axis
    const double jitter = i % 2 == 0 ? 0.02 : -0.02;
    ASSERT_TRUE(g.addOdom(t += 0.1, Eigen::Vector2d(jitter, 0.0), 0.0, 0.0));
  }
  EXPECT_FLOAT_EQ(g.update(t, Stale()).odometer, 0.f);
  for (int i = 0; i < 1000; ++i) {  // and within 2 cm of the point in any direction
    const double a = i * 2.4;
    ASSERT_TRUE(g.addOdom(t += 0.1, Eigen::Vector2d(0.02 * std::cos(a), 0.02 * std::sin(a)), 0.0, 0.0));
  }
  EXPECT_FLOAT_EQ(g.update(t, Stale()).odometer, 0.f);
}

TEST(Odometer, CountsRealTravel) {
  SpeedGovernor g(only(kOnlyTilt));
  double t = 0.0;
  for (int i = 0; i <= 100; ++i)  // 1 cm steps
    ASSERT_TRUE(g.addOdom(t += 0.1, Eigen::Vector2d(0.01 * i, 0.0), 0.0, 0.0));
  const float odo = g.update(t, Stale()).odometer;
  EXPECT_GT(odo, 0.94f);
  EXPECT_LE(odo, 1.0f + kTol);
}

TEST(Odometer, UsesXYOnlyAndAddsTheWholeStep) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d(1.0, 1.0), 0.0, 0.0));  // first sample is not travel
  ASSERT_TRUE(g.addOdom(0.2, Eigen::Vector2d(1.3, 1.4), 0.0, 0.0));
  EXPECT_NEAR(g.update(0.2, Stale()).odometer, 0.5f, kTol);
}

TEST(LioSpeed, Computation) {
  SpeedGovernor g(only(kOnlyTilt));
  OdomFeed odom{g};
  odom.cruise(0.5, 2.0);
  EXPECT_NEAR(g.update(0.0, Stale()).lio_speed, 0.5f, 1e-3f);
}

TEST(LioSpeed, UsesOldestSampleInWindowNotInstantaneous) {
  SpeedGovernor g(only(kOnlyTilt));
  OdomFeed odom{g};
  odom.cruise(0.0, 2.0);
  odom.step(0.1);  // one 1 m/s step after standing: 0.1 m over a window of 0.4 to 0.5 s
  const float v = g.update(0.0, Stale()).lio_speed;
  EXPECT_GE(v, 0.2f - kTol);
  EXPECT_LE(v, 0.25f + kTol);
}

TEST(LioSpeed, ZeroBelowMinimumSpan) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(1.0, Eigen::Vector2d(0.0, 0.0), 0.0, 0.0));
  EXPECT_FLOAT_EQ(g.update(1.0, Stale()).lio_speed, 0.f);
  ASSERT_TRUE(g.addOdom(1.2, Eigen::Vector2d(1.0, 0.0), 0.0, 0.0));
  EXPECT_FLOAT_EQ(g.update(1.2, Stale()).lio_speed, 0.f);   // 0.2 s span
  ASSERT_TRUE(g.addOdom(1.4, Eigen::Vector2d(2.0, 0.0), 0.0, 0.0));
  EXPECT_NEAR(g.update(1.4, Stale()).lio_speed, 5.f, 1e-3f);  // 0.4 s span
}

TEST(LioSpeed, GapGivesZero) {
  SpeedGovernor g(only(kOnlyTilt));
  OdomFeed odom{g};
  odom.cruise(0.5, 1.0);
  ASSERT_TRUE(g.addOdom(odom.t + 2.0, Eigen::Vector2d(odom.x + 1.0, 0.0), 0.0, 0.0));
  EXPECT_FLOAT_EQ(g.update(0.0, Stale()).lio_speed, 0.f);
}

// Stuck: odometry at 10 Hz, the update clock following it.
struct StuckRig {
  SpeedGovernor g{only(kOnlyTilt)};
  OdomFeed odom{g};
  Output run(double speed, double duration) {
    Output o;
    const int n = std::lround(duration * 10.0);
    for (int i = 0; i < n; ++i) {
      odom.step(speed * 0.1);
      o = g.update(odom.t, Stale());
    }
    return o;
  }
};

TEST(Stuck, CommandedButStillFiresAfterDwell) {
  StuckRig r;
  r.run(0.5, 1.0);
  ASSERT_TRUE(r.g.addCommand(0.5));
  EXPECT_FALSE(r.run(0.0, 0.9).stuck);  // the speed window still holds motion, then the dwell starts
  const Output o = r.run(0.0, 1.0);
  EXPECT_TRUE(o.stuck);
  EXPECT_GE(o.stuck_s, 1.0f - kTol);
}

TEST(Stuck, NotWhileMovingOrWithoutCommand) {
  StuckRig r;
  ASSERT_TRUE(r.g.addCommand(0.5));
  EXPECT_FALSE(r.run(0.2, 3.0).stuck);
  ASSERT_TRUE(r.g.addCommand(0.05));    // below min_cmd_v: a deliberate stop
  EXPECT_FALSE(r.run(0.0, 3.0).stuck);
  StuckRig n;                            // no command ever received
  EXPECT_FALSE(n.run(0.0, 3.0).stuck);
}

TEST(Stuck, ReverseCommandCounts) {
  StuckRig r;
  ASSERT_TRUE(r.g.addCommand(-0.3));
  EXPECT_TRUE(r.run(0.0, 2.0).stuck);
}

TEST(Stuck, ClearsWhenTheBodyMoves) {
  StuckRig r;
  ASSERT_TRUE(r.g.addCommand(0.5));
  ASSERT_TRUE(r.run(0.0, 2.0).stuck);
  const Output o = r.run(0.3, 1.0);
  EXPECT_FALSE(o.stuck);
  EXPECT_FLOAT_EQ(o.stuck_s, 0.f);
}

TEST(Stuck, StaleOrGappedOdometryIsNotStandingStill) {
  StuckRig r;
  ASSERT_TRUE(r.g.addCommand(0.5));
  r.run(0.0, 2.0);
  Stale s;
  s.odom = true;
  EXPECT_FALSE(r.g.update(r.odom.t + 0.1, s).stuck);
  // After a gap the window restarts below its minimum span: speed reads 0 but is not valid.
  StuckRig q;
  ASSERT_TRUE(q.g.addCommand(0.5));
  q.run(0.5, 1.0);
  ASSERT_TRUE(q.g.addOdom(q.odom.t + 2.0, Eigen::Vector2d(q.odom.x, 0.0), 0.0, 0.0));
  EXPECT_FALSE(q.g.update(q.odom.t + 2.0, Stale()).stuck);
  EXPECT_FALSE(q.g.update(q.odom.t + 3.5, Stale()).stuck);
}

TEST(Stuck, StaleCommandDoesNotCount) {
  StuckRig r;
  ASSERT_TRUE(r.g.addCommand(0.5));
  r.run(0.0, 2.0);
  Stale s;
  s.cmd = true;
  EXPECT_FALSE(r.g.update(r.odom.t, s).stuck);
}

TEST(Stuck, NeverChangesTheFactor) {
  StuckRig r;
  ASSERT_TRUE(r.g.addCommand(0.5));
  const Output o = r.run(0.0, 3.0);
  ASSERT_TRUE(o.stuck);
  EXPECT_FLOAT_EQ(o.speed_factor, 1.f);
}

TEST(Odom, NonFiniteIsRejectedAndKeepsAttitude) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  EXPECT_FALSE(g.addOdom(0.1, Eigen::Vector2d(kNaN, 0.0), 0.0, 0.0));
  EXPECT_FALSE(g.addOdom(0.1, Eigen::Vector2d(0.0, kInf), 0.0, 0.0));
  EXPECT_FALSE(g.addOdom(0.1, Eigen::Vector2d::Zero(), kNaN, 0.0));
  EXPECT_FALSE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 0.0, kNaN));
  EXPECT_FALSE(g.addOdom(kNaN, Eigen::Vector2d::Zero(), 0.0, 0.0));
  const Output o = g.update(0.1, Stale());
  EXPECT_NEAR(o.tilt_factor, 0.35f, kTol);
  EXPECT_FLOAT_EQ(o.odometer, 0.f);
}

// ---------- wheels from joints ----------

TEST(Joints, SignedSumAndAnyOrder) {
  double net = 0.0, speed = 0.0;
  // Left 103, 101 are + forward; right 102, 100 are - forward.
  ASSERT_TRUE(wheelsFromJoints({"Joint 102", "Joint 100", "Joint 103", "Joint 101"}, {-4.0, -4.0, 4.0, 4.0},
                               {-3.0, -4.0, 1.0, 2.0}, 0.13, &net, &speed));
  EXPECT_DOUBLE_EQ(net, 10.0);
  EXPECT_NEAR(speed, 4.0 * 0.13, 1e-12);
  double net2 = 0.0, speed2 = 0.0;
  ASSERT_TRUE(wheelsFromJoints({"Joint 100", "Joint 101", "Joint 102", "Joint 103"}, {-4.0, 4.0, -4.0, 4.0},
                               {-4.0, 2.0, -3.0, 1.0}, 0.13, &net2, &speed2));
  EXPECT_DOUBLE_EQ(net2, net);
  EXPECT_DOUBLE_EQ(speed2, speed);
}

TEST(Joints, SpinningInPlaceNetsToZero) {
  double net = 1.0, speed = 1.0;
  ASSERT_TRUE(wheelsFromJoints({"Joint 100", "Joint 101", "Joint 102", "Joint 103"}, {3.0, 3.0, 3.0, 3.0},
                               {5.0, 5.0, 5.0, 5.0}, 0.13, &net, &speed));
  EXPECT_DOUBLE_EQ(net, 0.0);
  EXPECT_DOUBLE_EQ(speed, 0.0);
}

TEST(Joints, AllFourRequired) {
  double net = 0.0, speed = 0.0;
  const std::vector<std::string> all = {"Joint 100", "Joint 101", "Joint 102", "Joint 103"};
  const std::vector<double> zeros(4, 0.0), three(3, 0.0);
  EXPECT_FALSE(wheelsFromJoints({"Joint 100", "Joint 101", "Joint 102"}, three, three, 0.13, &net, &speed));
  EXPECT_FALSE(wheelsFromJoints({"Joint 100", "Joint 100", "Joint 102", "Joint 103"}, zeros, zeros, 0.13, &net,
                                &speed));
  EXPECT_FALSE(wheelsFromJoints(all, zeros, {}, 0.13, &net, &speed));
  EXPECT_FALSE(wheelsFromJoints(all, {}, zeros, 0.13, &net, &speed));
}

TEST(Joints, NonFiniteRejected) {
  double net = 0.0, speed = 0.0;
  EXPECT_FALSE(wheelsFromJoints({"Joint 100", "Joint 101", "Joint 102", "Joint 103"}, {0, 0, 0, 0},
                                {0, kNaN, 0, 0}, 0.13, &net, &speed));
  EXPECT_FALSE(wheelsFromJoints({"Joint 100", "Joint 101", "Joint 102", "Joint 103"}, {kInf, 0, 0, 0},
                                {0, 0, 0, 0}, 0.13, &net, &speed));
}

// ---------- push ----------

TEST(Push, ResidualAtLoAndHi) {
  Output o = pushAt(11.6 + 18.0, 0.0, 0.5);
  EXPECT_NEAR(o.effort_residual, 18.f, kTol);
  EXPECT_NEAR(o.push_factor, 1.f, 1e-3f);
  o = pushAt(11.6 + 28.0, 0.0, 0.5);
  EXPECT_NEAR(o.effort_residual, 28.f, kTol);
  EXPECT_NEAR(o.push_factor, 0.3f, 1e-3f);
  o = pushAt(11.6 + 23.0, 0.0, 0.5);
  EXPECT_NEAR(o.push_factor, 0.65f, 1e-3f);
  o = pushAt(100.0, 0.0, 0.5);
  EXPECT_NEAR(o.push_factor, 0.3f, 1e-3f);
}

TEST(Push, SlopeIsSubtracted) {
  // The mound: climbing nose-up at 21 deg with net 33 is only gravity.
  Output o = pushAt(33.0, -21.0, 0.5);
  EXPECT_NEAR(o.effort_residual, 33.0 - (11.6 + 59.4 * std::sin(21 * kD2R)), 1e-3);
  EXPECT_LT(std::fabs(o.effort_residual), 0.5f);
  EXPECT_FLOAT_EQ(o.push_factor, 1.f);
  // The same effort on the flat is a push.
  o = pushAt(33.0, 0.0, 0.5);
  EXPECT_NEAR(o.effort_residual, 21.4f, kTol);
  EXPECT_LT(o.push_factor, 1.f);
  // Nose down expects less, so the residual grows.
  o = pushAt(33.0, 10.0, 0.5);
  EXPECT_NEAR(o.effort_residual, 33.0 - (11.6 - 59.4 * std::sin(10 * kD2R)), 1e-3);
  EXPECT_LT(o.push_factor, 0.5f);
}

TEST(Push, GatedByCommandSpeed) {
  EXPECT_FLOAT_EQ(pushAt(100.0, 0.0, 0.05).push_factor, 1.f);
  EXPECT_FLOAT_EQ(pushAt(100.0, 0.0, -0.5).push_factor, 1.f);
  EXPECT_FLOAT_EQ(pushAt(100.0, 0.0, 0.0).push_factor, 1.f);  // an in-place pivot commands v = 0
  // Inclusive at the limit.
  EXPECT_NEAR(pushAt(100.0, 0.0, 0.1).push_factor, 0.3f, 1e-3f);
}

TEST(Push, HardTurnDoesNotBlockTheRule) {
  // A hard turn at |w| = 2 rad/s reaches the core only as its v: addCommand no longer takes a turn rate.
  const double v_in_hard_turn = 0.3;
  const Output o = pushAt(11.6 + 28.0, 0.0, v_in_hard_turn);
  EXPECT_NEAR(o.effort_residual, 28.f, kTol);
  EXPECT_NEAR(o.push_factor, 0.3f, 1e-3f);
  EXPECT_EQ(o.limiting, kPush);
  EXPECT_FLOAT_EQ(pushAt(11.6 + 28.0, 0.0, 0.05).push_factor, 1.f);  // v below min_cmd_v still gives 1
}

TEST(Push, NoCommandEverGivesOne) {
  PushRig r;
  r.wheels(100.0);
  EXPECT_FLOAT_EQ(r.g.update(0.0, Stale()).push_factor, 1.f);
}

TEST(Push, AveragesOverTheWindow) {
  PushRig r;
  r.wheels(0.0, 1.0);
  r.wheels(11.6 + 40.0, 0.1);  // half of the 0.2 s window
  ASSERT_TRUE(r.g.addCommand(0.5));
  const Output o = r.g.update(0.0, Stale());
  EXPECT_NEAR(o.net_effort, (11.6 + 40.0) * 0.5, 2.0);
  EXPECT_NEAR(o.effort_residual, o.net_effort - 11.6f, 1e-3);  // the rule sees the mean, not the last sample
  r.wheels(11.6 + 40.0, 0.3);  // older samples have left the window
  EXPECT_NEAR(r.g.update(0.1, Stale()).net_effort, 11.6 + 40.0, 1e-3);
}

TEST(Push, HoldsForDistance) {
  PushRig r;
  r.odom.step(0.0);
  r.wheels(100.0);
  ASSERT_TRUE(r.g.addCommand(0.5));
  EXPECT_NEAR(r.g.update(0.0, Stale()).push_factor, 0.3f, 1e-3f);
  r.wheels(0.0);
  for (int i = 0; i < 7; ++i) r.odom.step(0.1);
  EXPECT_NEAR(r.g.update(0.1, Stale()).push_factor, 0.3f, 1e-3f);
  for (int i = 0; i < 2; ++i) r.odom.step(0.1);
  EXPECT_FLOAT_EQ(r.g.update(0.2, Stale()).push_factor, 1.f);
}

TEST(Push, StaleWheelsOrCommandGivesInputOne) {
  for (const bool wheels_stale : {true, false}) {
    PushRig r;
    r.wheels(100.0);
    ASSERT_TRUE(r.g.addCommand(0.5));
    Stale s;
    s.wheels = wheels_stale;
    s.cmd = !wheels_stale;
    EXPECT_FLOAT_EQ(r.g.update(0.0, s).push_factor, 1.f);
  }
}

TEST(Push, StaleInputDoesNotCutAnActiveHoldShort) {
  PushRig r;
  r.odom.step(0.0);
  r.wheels(100.0);
  ASSERT_TRUE(r.g.addCommand(0.5));
  EXPECT_NEAR(r.g.update(0.0, Stale()).push_factor, 0.3f, 1e-3f);
  Stale s;
  s.wheels = true;
  EXPECT_NEAR(r.g.update(0.1, s).push_factor, 0.3f, 1e-3f);
  for (int i = 0; i < 9; ++i) r.odom.step(0.1);
  EXPECT_FLOAT_EQ(r.g.update(0.2, s).push_factor, 1.f);
}

TEST(Push, NonFiniteIsRejected) {
  PushRig r;
  r.wheels(11.6 + 28.0);
  EXPECT_FALSE(r.g.addWheels(10.0, kNaN, 0.0));
  EXPECT_FALSE(r.g.addWheels(10.0, 1.0, kInf));
  EXPECT_FALSE(r.g.addWheels(kNaN, 1.0, 0.0));
  EXPECT_TRUE(r.g.addCommand(0.5));
  EXPECT_FALSE(r.g.addCommand(kNaN));
  EXPECT_FALSE(r.g.addCommand(kInf));
  const Output o = r.g.update(0.0, Stale());
  EXPECT_NEAR(o.net_effort, 11.6 + 28.0, 1e-3);
  EXPECT_NEAR(o.push_factor, 0.3f, 1e-3f);
}

// ---------- cover ----------

constexpr int kRows = 128, kCols = 1024;

TEST(Cover, FullImageOfReturnsGivesZero) {
  const Output o = coverOf(Image(kRows, kCols));
  EXPECT_EQ(o.cover_expected, bruteExpected(kRows, kCols, 0, 0));
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
  EXPECT_FLOAT_EQ(o.cover_avg, 0.f);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);
}

TEST(Cover, AllZerosInSectorGivesOneAndFloor) {
  Image im(kRows, kCols);
  im.zeroWhere([&](int, int c) { return std::fabs(bearing(c, kCols)) < 50.0; });
  const Output o = coverOf(im);
  EXPECT_GT(o.cover_expected, 1000u);
  EXPECT_FLOAT_EQ(o.cover_raw, 1.f);
  EXPECT_FLOAT_EQ(o.cover_avg, 1.f);
  EXPECT_NEAR(o.cover_factor, 0.6f, kTol);
}

TEST(Cover, ThresholdsAreInterpolated) {
  // Zero one column in every five of the sector: fraction near 0.2, the rule's far end.
  Image im(kRows, kCols);
  im.zeroWhere([&](int, int c) { return std::fabs(bearing(c, kCols)) < 50.0 && c % 5 == 0; });
  Output o = coverOf(im);
  EXPECT_NEAR(o.cover_raw, 0.2f, 0.01f);
  EXPECT_NEAR(o.cover_factor, interp(o.cover_avg, 0.03f, 0.2f, 1.f, 0.6f), kTol);
  EXPECT_NEAR(o.cover_factor, 0.6f, 0.02f);
}

TEST(Cover, ZerosBehindAreIgnored) {
  Image im(kRows, kCols);
  im.zeroWhere([&](int, int c) { return std::fabs(bearing(c, kCols)) > 90.0; });
  const Output o = coverOf(im);
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);
}

TEST(Cover, ZerosBeyondSectorAreIgnored) {
  Image im(kRows, kCols);
  im.zeroWhere([&](int, int c) { return std::fabs(bearing(c, kCols)) >= 45.5; });
  const Output o = coverOf(im);
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);

  // And just inside the sector they count, on both sides.
  for (const double sign : {1.0, -1.0}) {
    Image in(kRows, kCols);
    in.zeroWhere([&](int, int c) {
      const double b = sign * bearing(c, kCols);
      return b > 43.0 && b < 44.5;
    });
    EXPECT_GT(coverOf(in).cover_raw, 0.f);
  }
}

TEST(Cover, UpwardBeamsAreIgnored) {
  Image im(kRows, kCols);
  im.zeroWhere([&](int r, int) { return fallbackElevation(r, kRows) > 0.0; });
  const Output o = coverOf(im);
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
}

TEST(Cover, CentreColumnLooksForward) {
  auto oneColumn = [&](int col) {
    Image im(kRows, kCols);
    im.zeroWhere([&](int, int c) { return c == col; });
    return coverOf(im);
  };
  const Output centre = oneColumn(kCols / 2);
  const uint32_t down = std::round(centre.cover_raw * centre.cover_expected);
  EXPECT_GT(down, 50u);
  EXPECT_LT(down, 70u);  // about the lower half of 128 beams
  EXPECT_FLOAT_EQ(oneColumn(0).cover_raw, 0.f);                // c = 0 looks straight back
  EXPECT_FLOAT_EQ(oneColumn(kCols / 4).cover_raw, 0.f);        // +90 deg
  EXPECT_FLOAT_EQ(oneColumn(3 * kCols / 4).cover_raw, 0.f);    // -90 deg
}

TEST(Cover, ColumnIncreasesClockwise) {
  // az = 180 - 360*c/cols: columns below the centre are on the left (+y), so roll left-up removes them.
  Image left(kRows, kCols);
  left.zeroWhere([&](int, int c) { return c < kCols / 2; });
  Image right(kRows, kCols);
  right.zeroWhere([&](int, int c) { return c > kCols / 2; });
  const Output l = coverOf(left, 10.0, 0.0), r = coverOf(right, 10.0, 0.0);
  EXPECT_LT(l.cover_raw, 0.5f);
  EXPECT_GT(r.cover_raw, 0.5f);
}

TEST(Cover, NoseUpReducesExpected) {
  const uint32_t level = coverOf(Image(kRows, kCols)).cover_expected;
  const uint32_t up = coverOf(Image(kRows, kCols), 0.0, -10.0).cover_expected;
  const uint32_t down = coverOf(Image(kRows, kCols), 0.0, 10.0).cover_expected;
  EXPECT_LT(up, level);
  EXPECT_GT(down, level);
  EXPECT_EQ(up, bruteExpected(kRows, kCols, 0.0, -10.0));
  EXPECT_EQ(down, bruteExpected(kRows, kCols, 0.0, 10.0));
}

TEST(Cover, PitchLevellingSelectsTheRightBeams) {
  // Beams 5 deg below the sensor's horizon are above the true horizon at 10 deg nose-up.
  Image im(kRows, kCols);
  im.zeroWhere([&](int r, int) { const double e = fallbackElevation(r, kRows); return e > -7.0 && e < -3.0; });
  EXPECT_GT(coverOf(im, 0.0, 0.0).cover_raw, 0.f);
  EXPECT_FLOAT_EQ(coverOf(im, 0.0, -10.0).cover_raw, 0.f);
}

TEST(Cover, RollLevellingMatchesBruteForce) {
  for (const double roll : {-15.0, -5.0, 5.0, 15.0})
    EXPECT_EQ(coverOf(Image(kRows, kCols), roll, 3.0).cover_expected, bruteExpected(kRows, kCols, roll, 3.0));
}

TEST(Cover, RollShiftsTheExpectedSide) {
  // Roll positive lifts the left (+y) side: fewer left rays reach the ground, more on the right.
  Image left(kRows, kCols);
  left.zeroWhere([&](int, int c) { return bearing(c, kCols) > 0.0; });
  const float level = coverOf(left).cover_raw;
  const float rolled = coverOf(left, 10.0, 0.0).cover_raw;
  const float opposite = coverOf(left, -10.0, 0.0).cover_raw;
  EXPECT_NEAR(level, 0.5f, 0.02f);
  EXPECT_LT(rolled, level);
  EXPECT_GT(opposite, level);
}

TEST(Cover, AveragesOverWindowSweeps) {
  SpeedGovernor g(only(kOnlyCover));
  Image returns(kRows, kCols), empty(kRows, kCols, 0);
  auto add = [&](const Image& im) { ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step)); };
  add(empty);
  Output o = g.update(0.0, Stale());
  EXPECT_FLOAT_EQ(o.cover_raw, 1.f);
  EXPECT_FLOAT_EQ(o.cover_avg, 1.f);
  add(returns);
  o = g.update(0.1, Stale());
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
  EXPECT_FLOAT_EQ(o.cover_avg, 0.5f);
  add(returns);
  o = g.update(0.2, Stale());
  EXPECT_NEAR(o.cover_avg, 1.f / 3.f, 1e-5f);
  EXPECT_NEAR(o.cover_factor, 0.6f, kTol);
  add(returns);  // the empty sweep leaves the window
  o = g.update(0.3, Stale());
  EXPECT_FLOAT_EQ(o.cover_avg, 0.f);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);
}

TEST(Cover, TooFewExpectedRaysIsSkipped) {
  SpeedGovernor g(only(kOnlyCover));
  Image empty(kRows, kCols, 0);
  ASSERT_TRUE(g.addRangeImage(empty.data.data(), kRows, kCols, empty.step));
  ASSERT_FLOAT_EQ(g.update(0.0, Stale()).cover_avg, 1.f);

  Image tiny(4, 16);  // far under min_rays, and a real sweep would pull the average to 0.5
  ASSERT_TRUE(g.addRangeImage(tiny.data.data(), 4, 16, tiny.step));
  const Output o = g.update(0.1, Stale());
  EXPECT_LT(o.cover_expected, 100u);
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
  EXPECT_FLOAT_EQ(o.cover_avg, 1.f);
}

TEST(Cover, TooFewExpectedRaysFromStartGivesOne) {
  Image tiny(4, 16, 0);
  const Output o = coverOf(tiny);
  EXPECT_LT(o.cover_expected, 100u);
  EXPECT_FLOAT_EQ(o.cover_avg, 0.f);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);
}

TEST(Cover, ImageSizeChangeRebuilds) {
  SpeedGovernor g(only(kOnlyCover));
  Image a(kRows, 1024), b(kRows, 2048), c(64, 2048);
  ASSERT_TRUE(g.addRangeImage(a.data.data(), a.rows, a.cols, a.step));
  EXPECT_EQ(g.update(0.0, Stale()).cover_expected, bruteExpected(kRows, 1024, 0, 0));
  ASSERT_TRUE(g.addRangeImage(b.data.data(), b.rows, b.cols, b.step));
  EXPECT_EQ(g.update(0.1, Stale()).cover_expected, bruteExpected(kRows, 2048, 0, 0));
  ASSERT_TRUE(g.addRangeImage(c.data.data(), c.rows, c.cols, c.step));
  EXPECT_EQ(g.update(0.2, Stale()).cover_expected, bruteExpected(64, 2048, 0, 0));
  ASSERT_TRUE(g.addRangeImage(a.data.data(), a.rows, a.cols, a.step));
  EXPECT_EQ(g.update(0.3, Stale()).cover_expected, bruteExpected(kRows, 1024, 0, 0));
}

TEST(Cover, RowStepPaddingIsHonoured) {
  // Padding full of non-zero bytes must not hide empty pixels...
  Image empty(kRows, kCols, 0, 6, 0xff);
  EXPECT_FLOAT_EQ(coverOf(empty).cover_raw, 1.f);
  // ...and padding full of zeros must not read as no-return pixels.
  Image full(kRows, kCols, 1000, 6, 0);
  EXPECT_FLOAT_EQ(coverOf(full).cover_raw, 0.f);
  // A marked row lands where it should once padded.
  Image marked(kRows, kCols, 1000, 6, 0xff);
  marked.zeroWhere([&](int r, int) { return fallbackElevation(r, kRows) < -3.0; });
  Image plain(kRows, kCols);
  plain.zeroWhere([&](int r, int) { return fallbackElevation(r, kRows) < -3.0; });
  EXPECT_FLOAT_EQ(coverOf(marked).cover_raw, coverOf(plain).cover_raw);
  EXPECT_GT(coverOf(marked).cover_raw, 0.5f);
  // Columns too: a wrong row start shifts pixels sideways, out of a sector-shaped hole.
  const auto in_sector = [&](int, int c) { return std::abs(c - kCols / 2) < kCols / 8; };
  Image hole_padded(kRows, kCols, 1000, 6, 0xff), hole_plain(kRows, kCols);
  hole_padded.zeroWhere(in_sector);
  hole_plain.zeroWhere(in_sector);
  EXPECT_FLOAT_EQ(coverOf(hole_padded).cover_raw, coverOf(hole_plain).cover_raw);
  EXPECT_FLOAT_EQ(coverOf(hole_plain).cover_raw, 1.f);
}

TEST(Cover, EitherByteNonZeroIsAReturn) {
  // Byte order must not matter: 0x0100 has a zero low byte and is still a return.
  EXPECT_FLOAT_EQ(coverOf(Image(kRows, kCols, 0x0100)).cover_raw, 0.f);
  EXPECT_FLOAT_EQ(coverOf(Image(kRows, kCols, 0x0001)).cover_raw, 0.f);
}

TEST(Cover, RejectsBadImages) {
  SpeedGovernor g(only(kOnlyCover));
  Image im(kRows, kCols);
  EXPECT_FALSE(g.addRangeImage(nullptr, kRows, kCols, im.step));
  EXPECT_FALSE(g.addRangeImage(im.data.data(), 0, kCols, im.step));
  EXPECT_FALSE(g.addRangeImage(im.data.data(), kRows, 0, im.step));
  EXPECT_FALSE(g.addRangeImage(im.data.data(), kRows, kCols, 2 * kCols - 1));
}

TEST(Cover, MetadataAltitudesAreUsed) {
  SpeedGovernor g(only(kOnlyCover));
  ASSERT_TRUE(g.setBeamAltitudes(std::vector<float>(kRows, 10.f)));  // every beam upward
  Image im(kRows, kCols);
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  EXPECT_EQ(g.update(0.0, Stale()).cover_expected, 0u);
  // New altitudes at the same image size rebuild the rays.
  ASSERT_TRUE(g.setBeamAltitudes(std::vector<float>(kRows, -10.f)));
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  EXPECT_GT(g.update(0.1, Stale()).cover_expected, bruteExpected(kRows, kCols, 0, 0));
}

TEST(Cover, MetadataRowMismatchUsesFallback) {
  SpeedGovernor g(only(kOnlyCover));
  ASSERT_TRUE(g.setBeamAltitudes(std::vector<float>(64, 10.f)));
  Image im(kRows, kCols);
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  EXPECT_EQ(g.update(0.0, Stale()).cover_expected, bruteExpected(kRows, kCols, 0, 0));
}

TEST(Cover, BadMetadataIsRejectedAndFallbackKept) {
  SpeedGovernor g(only(kOnlyCover));
  EXPECT_FALSE(g.setBeamAltitudes({}));
  std::vector<float> bad(kRows, 10.f);
  bad[3] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(g.setBeamAltitudes(bad));
  Image im(kRows, kCols);
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  EXPECT_EQ(g.update(0.0, Stale()).cover_expected, bruteExpected(kRows, kCols, 0, 0));
}

// ---------- combined ----------

TEST(Combined, MinAndLimiting) {
  SpeedGovernor g;
  Image empty(kRows, kCols, 0);
  ASSERT_TRUE(g.addRangeImage(empty.data.data(), kRows, kCols, empty.step));
  Output o = g.update(0.0, Stale());
  EXPECT_NEAR(o.speed_factor, 0.6f, kTol);
  EXPECT_EQ(o.limiting, kCover);

  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  o = g.update(0.05, Stale());
  EXPECT_NEAR(o.tilt_factor, 0.35f, kTol);
  EXPECT_NEAR(o.cover_factor, 0.6f, kTol);
  EXPECT_NEAR(o.speed_factor, 0.35f, kTol);
  EXPECT_EQ(o.limiting, kTilt);
}

TEST(Combined, EachRuleCanBeTheLimiter) {
  {
    RotationRig r;
    r.gyro.run(20.0, 0.0, 0.5);
    EXPECT_EQ(r.g.update(0.0, Stale()).limiting, kRotation);
  }
  EXPECT_EQ(pushAt(100.0, 0.0, 0.5).limiting, kPush);
  EXPECT_EQ(tiltAt(20, 0).limiting, kTilt);
  EXPECT_EQ(coverOf(Image(kRows, kCols, 0)).limiting, kCover);
}

TEST(Combined, FallsImmediately) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 0.0, 0.0));
  EXPECT_FLOAT_EQ(g.update(0.0, Stale()).speed_factor, 1.f);
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  EXPECT_NEAR(g.update(0.05, Stale()).speed_factor, 0.35f, kTol);
}

TEST(Combined, RecoversExactlyAtReleaseRate) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  float f = g.update(0.0, Stale()).speed_factor;
  ASSERT_NEAR(f, 0.35f, kTol);
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 0.0, 0.0));
  double now = 0.0;
  for (int i = 0; i < 20; ++i) {
    now += 0.05;
    const Output o = g.update(now, Stale());
    EXPECT_FLOAT_EQ(o.tilt_factor, 1.f);
    EXPECT_NEAR(o.speed_factor, f + 0.25f * 0.05f, 1e-5f);
    f = o.speed_factor;
  }
  EXPECT_NEAR(f, 0.35f + 0.25f, 1e-4f);  // after one second
}

TEST(Combined, ReleaseStopsAtRaw) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  g.update(0.0, Stale());
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 12.5 * kD2R, 0.0));  // raw 0.675
  double now = 0.0;
  for (int i = 0; i < 100; ++i) g.update(now += 0.05, Stale());
  EXPECT_NEAR(g.update(now += 0.05, Stale()).speed_factor, 0.675f, kTol);
}

TEST(Combined, DtIsClamped) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  g.update(0.0, Stale());
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 0.0, 0.0));
  // A long gap is worth at most 0.5 s of release.
  EXPECT_NEAR(g.update(100.0, Stale()).speed_factor, 0.35f + 0.25f * 0.5f, 1e-5f);
  // Time going backwards releases nothing.
  EXPECT_NEAR(g.update(50.0, Stale()).speed_factor, 0.35f + 0.25f * 0.5f, 1e-5f);
}

TEST(Combined, LastLimiterKeptWhileRecoveringThenNone) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 0.0));
  EXPECT_EQ(g.update(0.0, Stale()).limiting, kTilt);
  ASSERT_TRUE(g.addOdom(0.1, Eigen::Vector2d::Zero(), 0.0, 0.0));
  double now = 0.0;
  Output o;
  for (int i = 0; i < 200; ++i) {
    o = g.update(now += 0.05, Stale());
    if (o.speed_factor < 1.f) {
      EXPECT_EQ(o.limiting, kTilt);
    } else {
      break;
    }
  }
  EXPECT_FLOAT_EQ(o.speed_factor, 1.f);
  EXPECT_EQ(o.limiting, kNone);
}

TEST(Combined, NoneWhenNothingLimits) {
  SpeedGovernor g;
  const Output o = g.update(0.0, Stale());
  EXPECT_FLOAT_EQ(o.speed_factor, 1.f);
  EXPECT_EQ(o.limiting, kNone);
}

TEST(Combined, DisablingABranchOrRuleGivesOne) {
  auto hit = [](const Params& p) {
    SpeedGovernor g(p);
    OdomFeed odom{g};
    GyroFeed gyro{g};
    double t = 0.0;
    for (int i = 0; i < 10; ++i)
      odom.step(0.05, 20 * kD2R, 25 * kD2R);
    gyro.run(20.0, 20.0, 0.5);
    for (int i = 0; i < 30; ++i) {
      EXPECT_TRUE(g.addWheels(t, 200.0, 0.5));
      t += 0.01;
    }
    EXPECT_TRUE(g.addCommand(0.5));
    Image empty(kRows, kCols, 0);
    EXPECT_TRUE(g.addRangeImage(empty.data.data(), kRows, kCols, empty.step));
    return g.update(1.0, Stale());
  };

  // Everything on: all four rules fire, so the switches below are what makes the difference.
  const Output on = hit(Params());
  EXPECT_LT(on.tilt_factor, 1.f);
  EXPECT_LT(on.rotation_factor, 1.f);
  EXPECT_LT(on.push_factor, 1.f);
  EXPECT_LT(on.cover_factor, 1.f);

  Params p;
  p.proprioceptive = false;
  Output o = hit(p);
  EXPECT_FLOAT_EQ(o.tilt_factor, 1.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
  EXPECT_FLOAT_EQ(o.push_factor, 1.f);
  EXPECT_LT(o.cover_factor, 1.f);

  p = Params();
  p.predictive = false;
  o = hit(p);
  EXPECT_LT(o.tilt_factor, 1.f);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);
  EXPECT_EQ(o.cover_expected, 0u);  // not computed at all

  p = Params();
  p.proprioceptive = p.predictive = false;
  o = hit(p);
  EXPECT_FLOAT_EQ(o.speed_factor, 1.f);
  EXPECT_EQ(o.limiting, kNone);

  p = Params();
  p.tilt.enable = false;
  EXPECT_FLOAT_EQ(hit(p).tilt_factor, 1.f);
  p = Params();
  p.rotation.enable = false;
  EXPECT_FLOAT_EQ(hit(p).rotation_factor, 1.f);
  p = Params();
  p.push.enable = false;
  EXPECT_FLOAT_EQ(hit(p).push_factor, 1.f);
  p = Params();
  p.cover.enable = false;
  EXPECT_FLOAT_EQ(hit(p).cover_factor, 1.f);

  p = Params();
  p.tilt.enable = p.rotation.enable = p.push.enable = p.cover.enable = false;
  EXPECT_FLOAT_EQ(hit(p).speed_factor, 1.f);
}

// ---------- staleness ----------

TEST(StaleRules, BitmaskMatchesFlags) {
  SpeedGovernor g;
  Stale s;
  EXPECT_EQ(g.update(0.0, s).stale, 0);
  s.imu = true;
  EXPECT_EQ(g.update(0.0, s).stale, kStaleImu);
  s = Stale();
  s.odom = true;
  EXPECT_EQ(g.update(0.0, s).stale, kStaleOdom);
  s = Stale();
  s.wheels = true;
  EXPECT_EQ(g.update(0.0, s).stale, kStaleWheels);
  s = Stale();
  s.cmd = true;
  EXPECT_EQ(g.update(0.0, s).stale, kStaleCmd);
  s = Stale();
  s.image = true;
  EXPECT_EQ(g.update(0.0, s).stale, kStaleImage);
  s.imu = s.odom = s.wheels = s.cmd = s.image = true;
  EXPECT_EQ(g.update(0.0, s).stale, 31);
  EXPECT_EQ(kStaleImu, 1);
  EXPECT_EQ(kStaleOdom, 2);
  EXPECT_EQ(kStaleWheels, 4);
  EXPECT_EQ(kStaleCmd, 8);
  EXPECT_EQ(kStaleImage, 16);
}

TEST(StaleRules, StaleImuIgnoresRotationInput) {
  RotationRig r;
  r.gyro.run(20.0, 0.0, 0.5);
  Stale s;
  s.imu = true;
  const Output o = r.g.update(1.0, s);
  EXPECT_FLOAT_EQ(o.rotation_deg, 0.f);
  EXPECT_FLOAT_EQ(o.rotation_factor, 1.f);
}

TEST(StaleRules, StaleImuKeepsHoldRunningOnOdometry) {
  RotationRig r;
  r.gyro.run(20.0, 0.0, 0.5);
  ASSERT_NEAR(r.g.update(1.0, Stale()).rotation_factor, 0.3f, kTol);
  Stale s;
  s.imu = true;
  EXPECT_NEAR(r.g.update(1.05, s).rotation_factor, 0.3f, kTol);
  for (int i = 0; i < 7; ++i) r.odom.step(0.1);
  EXPECT_NEAR(r.g.update(1.1, s).rotation_factor, 0.3f, kTol);
  for (int i = 0; i < 2; ++i) r.odom.step(0.1);
  EXPECT_FLOAT_EQ(r.g.update(1.15, s).rotation_factor, 1.f);
}

TEST(StaleRules, StaleOdomGivesTiltOne) {
  SpeedGovernor g(only(kOnlyTilt));
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 25 * kD2R));
  ASSERT_LT(g.update(0.0, Stale()).tilt_factor, 1.f);
  Stale s;
  s.odom = true;
  const Output o = g.update(0.05, s);
  EXPECT_FLOAT_EQ(o.tilt_factor, 1.f);
  EXPECT_FLOAT_EQ(o.roll_deg, 0.f);
  EXPECT_FLOAT_EQ(o.pitch_deg, 0.f);
  EXPECT_FLOAT_EQ(o.lio_speed, 0.f);
}

TEST(StaleRules, StaleOdomResetsHolds) {
  RotationRig r;
  r.gyro.run(20.0, 0.0, 0.5);
  ASSERT_NEAR(r.g.update(1.0, Stale()).rotation_factor, 0.3f, kTol);
  r.gyro.run(0.0, 0.0, 0.5);
  Stale s;
  s.odom = true;
  EXPECT_FLOAT_EQ(r.g.update(1.05, s).rotation_factor, 1.f);
  // Still released once odometry is back, with no further travel.
  r.odom.step(0.0);
  EXPECT_FLOAT_EQ(r.g.update(1.1, Stale()).rotation_factor, 1.f);

  PushRig p;
  p.odom.step(0.0);
  p.wheels(100.0);
  ASSERT_TRUE(p.g.addCommand(0.5));
  ASSERT_NEAR(p.g.update(0.0, Stale()).push_factor, 0.3f, 1e-3f);
  EXPECT_FLOAT_EQ(p.g.update(0.05, s).push_factor, 1.f);
}

TEST(StaleRules, StaleOdomDoesNotLatchAPushThatCannotRelease) {
  PushRig r;
  r.wheels(100.0);
  ASSERT_TRUE(r.g.addCommand(0.5));
  Stale s;
  s.odom = true;
  for (int i = 0; i < 10; ++i) EXPECT_FLOAT_EQ(r.g.update(0.05 * i, s).push_factor, 1.f);
}

TEST(StaleRules, StaleOdomLevelsCover) {
  SpeedGovernor g(only(kOnlyCover));
  Image im(kRows, kCols);
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 0.0, -10 * kD2R));
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  EXPECT_EQ(g.update(0.0, Stale()).cover_expected, bruteExpected(kRows, kCols, 0.0, -10.0));

  Stale s;
  s.odom = true;
  g.update(0.05, s);
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  const Output o = g.update(0.1, s);
  EXPECT_EQ(o.cover_expected, bruteExpected(kRows, kCols, 0.0, 0.0));

  // A fresh pose restores the levelling.
  ASSERT_TRUE(g.addOdom(0.2, Eigen::Vector2d::Zero(), 0.0, -10 * kD2R));
  ASSERT_TRUE(g.addRangeImage(im.data.data(), kRows, kCols, im.step));
  EXPECT_EQ(g.update(0.25, Stale()).cover_expected, bruteExpected(kRows, kCols, 0.0, -10.0));
}

TEST(StaleRules, StaleImageClearsCover) {
  SpeedGovernor g(only(kOnlyCover));
  Image empty(kRows, kCols, 0), returns(kRows, kCols);
  ASSERT_TRUE(g.addRangeImage(empty.data.data(), kRows, kCols, empty.step));
  ASSERT_NEAR(g.update(0.0, Stale()).cover_factor, 0.6f, kTol);
  Stale s;
  s.image = true;
  const Output o = g.update(0.05, s);
  EXPECT_FLOAT_EQ(o.cover_factor, 1.f);
  EXPECT_FLOAT_EQ(o.cover_avg, 0.f);
  EXPECT_FLOAT_EQ(o.cover_raw, 0.f);
  // The average does not resume from before the gap.
  ASSERT_TRUE(g.addRangeImage(returns.data.data(), kRows, kCols, returns.step));
  EXPECT_FLOAT_EQ(g.update(0.1, Stale()).cover_avg, 0.f);
}

TEST(StaleRules, EverythingStaleGivesFullSpeed) {
  SpeedGovernor g;
  ASSERT_TRUE(g.addOdom(0.0, Eigen::Vector2d::Zero(), 20 * kD2R, 25 * kD2R));
  Stale s;
  s.imu = s.odom = s.wheels = s.cmd = s.image = true;
  double now = 0.0;
  Output o;
  for (int i = 0; i < 60; ++i) o = g.update(now += 0.05, s);
  EXPECT_FLOAT_EQ(o.speed_factor, 1.f);
  EXPECT_EQ(o.limiting, kNone);
}

// ---------- robustness ----------

TEST(Robustness, OutputAlwaysFiniteAndInRange) {
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  auto wild = [&](double scale) {
    const double pick = u(rng);
    if (pick > 0.95) return kNaN;
    if (pick > 0.9) return kInf;
    if (pick > 0.85) return -kInf;
    return u(rng) * scale;
  };
  SpeedGovernor g;
  Image im(kRows, kCols);
  im.zeroWhere([&](int r, int c) { return (r * 7 + c * 13) % 11 == 0; });
  double t = 0.0;
  for (int i = 0; i < 2000; ++i) {
    t += 0.05;
    g.addImu(t + wild(0.5), wild(20.0), wild(20.0));
    g.addOdom(t + wild(0.5), Eigen::Vector2d(wild(100.0), wild(100.0)), wild(3.2), wild(1.6));
    g.addWheels(t + wild(0.5), wild(200.0), wild(5.0));
    g.addCommand(wild(1.0));
    if (i % 20 == 0) g.addRangeImage(im.data.data(), kRows, kCols, im.step);
    Stale s;
    s.imu = u(rng) > 0.8;
    s.odom = u(rng) > 0.8;
    s.wheels = u(rng) > 0.8;
    s.cmd = u(rng) > 0.8;
    s.image = u(rng) > 0.8;
    const Output o = g.update(i % 100 == 0 ? kNaN : t, s);
    ASSERT_TRUE(std::isfinite(o.speed_factor));
    ASSERT_GE(o.speed_factor, 0.f);
    ASSERT_LE(o.speed_factor, 1.f);
    for (const float f : {o.tilt_factor, o.rotation_factor, o.push_factor, o.cover_factor}) {
      ASSERT_TRUE(std::isfinite(f));
      ASSERT_GE(f, 0.f);
      ASSERT_LE(f, 1.f);
    }
    for (const float f : {o.roll_deg, o.pitch_deg, o.droll_deg, o.dpitch_deg, o.rotation_deg, o.net_effort,
                          o.effort_residual, o.cover_raw, o.cover_avg, o.lio_speed, o.wheel_speed, o.odometer})
      ASSERT_TRUE(std::isfinite(f));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
