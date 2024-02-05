std_msgs/Header header # Standard ROS message header

int8 ALT_REF_GEOID=0
int8 ALT_REF_ELLIPSOID=1

# Position
float64     latitude     # In degrees, positive north, [-90,90]
float64     longitude    # In degrees, increasgin toward east, [0,360]
int8        altitude_ref # 0: Geoid / 1: Ellipsoid, see constant above.
float32     altitude     # In meters, positive up

# Position Deviation
float64[9] position_covariance # In square meters, ENU in row-major order
                                # Null matrix if unknown

# Attitude
float32 heading # In degrees, [0,360]
float32 roll    # In degrees, positive port up, [-180,180]
float32 pitch   # In degrees, positive bow down, [-90,90]

# Attitude Deviation
float64[9] attitude_covariance # In rad2/sec2
                               # Null matrix if unknown

# Speed Vessel Frame
geometry_msgs/Vector3 speed_vessel_frame # In m/s, x: forward, y: left, z: up

# Speed Vessel Frame Deviation
float64[9] speed_vessel_frame_covariance # In m2/s4, ENU in row-major order
                                         # Null matrix if unknown

# Status
# TODO...
