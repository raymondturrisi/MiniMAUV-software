/*
 * @file readme_sensors.txt
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
 * 	9/4/2020: MiniMAUV sensors read me conceived - rt
 * 
 * 
 * 
*/
9/4/2020:
The sensor package will contain sensor nodes to individual TYPES of sensors with appropriate error handling in the absense of a component. These nodes report to their respective topics and messages, before being processed under the filtering node, and written to dense and minimalistic data files in the bagging node which must be compatible with MATLAB & Python in post processing. 

	> VISensor1.1
		- Reads the current and voltage from Arduinos current and voltage sensor, while also reading the BeagleBoneBlue's integrated voltage sensor at the barrel jack. 
		- Publishes these readings to their respective messages of /sensors/<float32>current, /sensors/<float32>voltage1, and /sensors/<float32>voltage2 respectively

	> PTSensors
		- Reads the external pressure and temperature from the Bar30 and the internal pressure and temperature which is built into the BeagleBoneBlue, publishing to /sensors/<float32>epressure, /sensors/<float32>etemp, /sensors/<float32>ipressure, and /sensors/<float32>itemp messages respectively. 
		- Depth can be derived by any node which needs it from the /data/<float32>epressure message
		- Safe measures can be implemented by the brain node with the /data/<float32>ipressure and /data/<float32>itemp messages

	> imu_mpu
		- Reads the Lord Microstrain AHRS and the BeagleBoneBlue's mpu for comparative measurements and integrating the local pose
		- Cycles at the maximum frequency of the Microstrain AHRS in order to most accurately integrate and determine it's current pose
		- Publishes to:
		/imus
			/<float32>ahrs_ax
			/<float32>ahrs_ay
			/<float32>ahrs_az
			/<float32>bbbl_ax
			/<float32>bbbl_ay
			/<float32>bbbl_az
			/<float32>ahrs_wx
			/<float32>ahrs_wy
			/<float32>ahrs_wz
			/<float32>bbbl_wx
			/<float32>bbbl_wy
			/<float32>bbbl_wz
			/<float32>ahrs_ax
			/<float32>ahrs_ay
			/<float32>ahrs_az
	> filtering
		- Filters all data under /sensors topic with various algorithms before publishing to:
		/data
			/<float32>epressure
			/<float32>ipressure
			/<float32>etemp
			/<float32>itemp
			/<float32>voltage
			/<float32>current
			/<float32>power
		- Performs dead reckoning algorithm and publishes to: 
		/pose
			/<float32>x
			/<float32>y
			/<float32>z
			/<float32>r
			/<float32>p
			/<float32>y
			/<float32>h
		- Cycles at equivalent frequency to imus

	> bagging
		- Records all data from /data at an appropriate frequency for best insight on post mission performance evaluation
		- Uses standard format and delimeters understood by MATLAB and Python