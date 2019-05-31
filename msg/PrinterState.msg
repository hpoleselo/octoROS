# -- ACTUAL DATA --

# timestamp for logging
int64 timestamp
string date_time

# data from the 3D Printer
float32 temp_tool1_actual
float32 temp_tool2_actual
float32 temp_bed_actual

# data from the file to be printed
int32 file_size
string file_name
string printer3d_state
float32 progress
int64 time_left
float32 time_elapsed

# -- GOAL DATA --

# data from the 3D Printer
float32 temp_tool1_goal
float32 temp_tool2_goal
float32 temp_bed_goal