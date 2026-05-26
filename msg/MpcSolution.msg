Header header
float64[] q_start
float64[] dq_start
float64[] ddq_start
float64[] jerk
float64 dt_mpc          
float64 solve_time_ms
float64 position_error_norm    # ||q_current - q_target||
float64 velocity_error_norm    # ||dq_current - dq_target||
float64[] position_error_per_joint  # errore per ogni giunto
float64[] velocity_error_per_joint  # errore per ogni giunto