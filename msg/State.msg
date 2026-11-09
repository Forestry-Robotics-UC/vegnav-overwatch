# Speed governor state, published at publish_rate whether or not the factor is applied.
Header header

float32 speed_factor

uint8 NONE = 0
uint8 TILT = 1
uint8 ROTATION = 2
uint8 PUSH = 3
uint8 COVER = 4
uint8 limiting

bool applied                     # false = shadow mode, /overwatch/speed_factor is not published

float32 tilt_factor
float32 rotation_factor          # after its distance hold
float32 push_factor              # after its distance hold
float32 cover_factor

float32 roll_deg
float32 pitch_deg                # positive = nose down
float32 droll_deg                # peak since the last tick, over the rotation window
float32 dpitch_deg
float32 rotation_deg             # max(|droll|, |dpitch|)

float32 net_effort               # mean over the push window, + = forward
float32 effort_residual          # net_effort minus the flat-ground and slope terms

float32 cover_raw                # no-return fraction of the latest sweep
float32 cover_avg
uint32 cover_expected            # rays in the forward sector aimed at the ground, latest sweep

float32 lio_speed
float32 wheel_speed
float32 odometer

float32 cover_compute_ms         # wall time of the latest sweep

uint8 IMU = 1
uint8 ODOM = 2
uint8 WHEELS = 4
uint8 CMD = 8
uint8 IMAGE = 16
uint8 AUTONOMY = 32               # pure pursuit's command, read only by the stuck rule
uint8 stale

bool stuck                       # pure pursuit drives, the wheels push or spin, the body does not move,
                                 # for stuck/dwell_s; published on /overwatch/stuck
float32 stuck_s                  # how long the stuck condition has held, 0 when it does not

bool rotation_stuck              # pure pursuit rotates in place, the wheels pivot, LIO yaw does not change,
                                 # for rotation_stuck/dwell_s; latched while pure pursuit keeps turning;
                                 # published on /overwatch/rotation_stuck (pure pursuit then wiggles)
float32 rotation_stuck_s         # candidate time before the latch, then time latched
bool rotation_escalated          # the wiggle made no progress: stuck is set (back up and replan)
float32 yaw_rate                 # LIO, over rotation_stuck/yaw_window_s [rad/s]
float32 wheel_yaw_speed          # (right - left) / 2 wheel speed [m/s], + = CCW
float32 abs_effort               # sum of |effort| of the four wheels
