uint8 lr

# Motor status
uint8[6]  status
float32[6] m_pos
float32[6] m_spd
float32[6] m_cur
float32[6] m_force

# Joint angles (radians)
float32[11] j_ang

# Base pose
float64 x_base
float64 y_base
float64 z_base
float64 roll_base
float64 pitch_base
float64 yaw_base

# Fingertip poses (world frame)
float64[5] x
float64[5] y
float64[5] z
float64[5] roll
float64[5] pitch
float64[5] yaw

# Quaternion (optional)
float64[5] w
float64[5] i
float64[5] j
float64[5] k