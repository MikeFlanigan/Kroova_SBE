import numpy as np 
params = np.genfromtxt('SBE_control_params.csv',delimiter=",")
target_RH, P_gain, I_gain, D_gain, servo_control_offset, US_rolling_avg_window, US_max_thresh, US_min_thresh, servo_max, servo_min = params[1]
# target RH in mm
# P gain
# I gain
# D gain 
# servo control angle offset in degrees
# Length of rolling average window on US sensor
# max valid mm -- based on blade rider foil and mounting setup as of 6/11/18
# min valid mm -- based on blade rider foil and mounting setup as of 6/11/18
# max servo angle based on mechanical limits as of 6/11/18
# min servo angle based on mechanical limits as of 6/11/18
