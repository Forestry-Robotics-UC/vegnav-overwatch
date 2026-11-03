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
uint8 stale
