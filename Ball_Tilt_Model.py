ball_diam = 0 # meters
ball_radius = ball_diam / 2 # meters
ball_mass = 0 # kg
ball_inertia = 2/3*ball_mass*ball_radius**2 # using inertia for a hollow sphere

ball_pos = 2 # meters from the leftmost end of the track (sensor at the right end of the track)
ball_omega = 0 # rad/s

ball_linear_speed = 0 # m/s might not use

ramp_angle = 0 # radians

g = 9.82 # m/s^2


def init_ball_physics_sim(ball_pos_init, servo_angle):
	global ball_pos
	ball_pos = ball_pos_init


def ball_physics_update(servo_angle, time_delta):
	global ball_pos

	# calculate how far the ball moved based on last constant speed time step

	# calculate how much the ball accelerated based on last constant acceleration time step

	# update the torque on the ball based on the new servo/ramp angle 


	return ball_pos