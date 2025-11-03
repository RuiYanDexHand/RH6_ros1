# RH6 CAN communication library (Python)
# High-level API re-exports. Prefer real hardware wrapper when available.
try:
    from .ryhandlib_wrapper import (
        init_can_servo_lib,
        servo_move_mix,
        get_servo_data,
        cleanup_can_servo_lib,
    )
except Exception:
    from .can_interface import (
        init_can_servo_lib,
        servo_move_mix,
        get_servo_data,
        cleanup_can_servo_lib,
    )
