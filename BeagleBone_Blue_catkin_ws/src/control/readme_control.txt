/*
 * @file readme_control.txt
 * 
 * @description
 * 	implicit
 * 
 * @author | initials	
 * 	Raymond Turrisi, Graduate School of Oceanography, University of Rhode Island | rt
 * @contributors | initials
 * 	- 
 * 	- 
 * @log
 * 	9/4/2020: MiniMAUV control read me conceived - rt
 * 
 * 
 * 
*/

9/4/2020: 
The control package will contain actuator nodes, the MAUV heartbeat, IOs which must operate at a certain frequency (PWM), and primative maneuvers which must be used in series to move the vehicle between waypoints. 

	> MAUV_Heartbeat
		- Tics in milliseconds
		- Stored in int32, (will hit ceiling limit, 2,147,483,647, at 6000 hours)
		- Publishes to heartbeat/<int32>beat
		- Governs the timing for the vehicle

	> PWM_out
		- Reads from DTT, VBS, and MM topics and outputs the desired PWM at a standard frequency of 50hz
		-This is done due to the Robot Control Library's (on the BeagleBoneBlue) limitations of sending a single pulsewidth instead of a continuous PWM while other calculations must operate at a higher frequency

	> DTT
		- Receives desired angles from /dtt/<float32>hfleft_ang and /dtt/<float32>hfright_ang
		- Receives desired PWM from /dtt/<float32>escleft and /dtt/<float32>escright
		- Reads current angle and publishes them to /dtt/<float32>hfleft_ang_read and /dtt/<float32>hfright_ang_read
		- Performs the PID calculations to control angular velocity and maintain the supplied power to the servomotors
			- From MBED benchtop tests:
				- Pconst = 1, Iconst = 2.5, Dconst = 0.5
				- Max gain = 30 (pwm above below center at 1475 pwm)

	> VBS
		- Upon startup calibrates to 0 distance with button, and then goes to desired distance
		- Reads desired state of buoyancy from /vbs/<float32>vbs_desired_dist
		- Publishes state of buoyancy to /vbs/<float32>vbs_dist
		- Uses empirical coefficient which is multiplied by number of revolutions to predict the change in buoyancy until VBS has reached desired distance
		- Changes buoyancy to reach desired yaw with /pose/<float32>y and /pose/<float32>desired_y within a tolerance to ensure it is centered around desired buoyancy

	> Battery
		- Reads angle of moving mass from /mm/<float32>mm_ang to sustain desired orientation
		- Adjusts angle within a certain tolerance around center in order to roll the vehicle from desired objective with /pose/<float32>r and /pose/<float32>desired_r (only in horizontal orientation, used to be self righting if /commands/<std::string>orientation == V to stay parallel to gravity (9.81 m/s^2 in the X direction of the vehicle

	> 0_ChangeMode
		- If /commands/<std::string>orientation == H, pushes forward VBS and rolls batteries to bottom of vehicle
		- If /commands/<std::string>orientation == V, retracts VBS and rolls batteries to top of vehicle

	> 1H_Heading
		- If /commands/<std::string>orientation == H && /commands/<std::string>command == 1, uses PID control to change heading about the Z axis while also driving forward in a straight line

	> 2H_LongTurn_FromAngle
		- If /commands/<std::string>orientation == H && /commands/<std::string>command == 2, takes a long turn from angle to desired angle about Z axis from current x to current y
		- Distinguished from Sharp turn in Brain node
		- Used for surveying
		- Powers on thruster greater than the other but at the same angle

	> 3H_SharpTurn_FromAngle
		- If /commands/<std::string>orientation == H && /commands/<std::string>command == 3, takes a sharp turn from angle to desired angle about Z axis from current x to current y
		- Distinguished from Sharp turn in Brain node
		- Used for surveying
		- Rolls foils so vehicle is on its side, orients foils perpendicular to vehicle, drivers thrusters to spin in place about what was it's Z axis, but on it's side the Y axis

	> 4H_LongDecsension
		- If /commands/<std::string>orientation == H && /commands/<std::string>command == 4
		- Maintains horizontal orientation while decreasing buoyancy and pushing slightly downward

	> 5H_Hover
		- If /commands/<std::string>orientation == H && /commands/<std::string>command == 5
		- Hovers vehicle in place at a position [x y z] and at the best roll and yaw [r p 0] in order to best observe object
		- Can drive vehicle in reverse with a very slow drive forward, and a burst reverse?

	> 6H_FollowPath
		- If /commands/<std::string>orientation == H && /commands/<std::string>command == 6
		- Takes a series of waypoints (next waypoint, and calculates best orientation to enter the following waypoint with a spline interpolation algorithm)
		- Makes vehicle follow the specified path

	> 1V_DiveToDepth
		- If /commands/<std::string>orientation == V && /commands/<std::string>command == 1
		- Dives to certain depth (external pressure) with /data/<float32>/epressure, and maintains depth

	> 2V_Heading
		- If /commands/<std::string>orientation == V && /commands/<std::string>command == 2
		- In Ghost mode drives forward and maintains a particular heading (heading? check ahrs)

	> 3V_Drift
		- If /commands/<std::string>orientation == V && /commands/<std::string>command == 3
		- Disables V regs and enters low power mode

	> 4V_Hover
		- If /commands/<std::string>orientation == V && /commands/<std::string>command == 4
		- Maintains a particular pose and self rights itself to best observe in vertical orientation
		- Panorams with bottom camera of surrounding structures
	
	> 5V_FollowPath
		- If /commands/<std::string>orientation == V && /commands/<std::string>command == 5
		- Takes a series of waypoints (next waypoint, and calculates best orientation to enter the following waypoint with a spline interpolation algorithm)
		- Makes vehicle follow the specified path in Ghost mode


File system
/src
	> /Navigation
	> /Control
		> /include
		> /src	
			> readme_actuators.txt
			> MAUV_Heartbeat
			> PWM Outs (and VBS button)
			> DTT
			> VBS
			> Battery
			> readme_commands.txt
			> 0_ChangeMode
			> 1H_Heading
			> 2H_LongTurn_FromAngle
			> 3H_SharpTurn_FromAngle
			> 4H_LongDecsension
			> 5H_Hover
			> 6H_FollowPath
			> 1V_DiveToDepth
			> 2V_Heading
			> 3V_Drift
			> 4V_Hover
			> 5V_FollowPath
	> /Data
	> CMakeList.txt