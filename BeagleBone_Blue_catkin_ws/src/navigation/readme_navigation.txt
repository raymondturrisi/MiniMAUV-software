/*
 * @file readme_navigation.txt
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
 * 	9/4/2020: MiniMAUV navigation read me conceived - rt
 * 
 * 
 * 
*/

9/4/2020: 
The navigation package will contain mission "programs" and a "brain" node. 

Mission programs set objectives and waypoints until conditions are met before fulfilling a desired process before calling. These objectives and waypoints are published to the goals topic, and solely interpreted by the brain node. The brain node takes third level precedence in controlling the MAUV. This node makes decisions for which "commands" to call to take the vehicle from its current state, to its desired state which is published on the goal topic. The "brain" node, is the fundamental driver of the vehicle, which will always be running as it is the decision maker for the vehicle, however mission nodes are per mission which must work with the brain node. In order to change the mission, you must modify the launch file - do not have more than one mission node run at launch time. 

First level precedence are bailing protocols stored within the heartbeat node - for example if a leak sensor is tripped, the battery is low, etc. the mission node is shutdown by the heartbeat node which sends procedural commands to recover the vehicle depending on the severity of the condition. 

Second level precedence are compensations made by closed loop control in order to meet desired objectives. 

These programs should be linear and not loop themselves, rather have a clear start and finish with loops which repeatedly publish to topics in between. i.e.

start
.
.
initialization procedure with MAUV_heartbeat
.
.

while(stepN) {
	publish loc1
	if(con1 && con2 && con3){
		stepN++
		call hover
		do {

		} while(con4);
	}
	if(null) {
		stepN++
	}
}

while(stepN+1) {
	publish loc1
	if(con5 && con6 && con7){
		stepN++
		call hover
		do {

		} while(con8);
	}
}

.
.
.

stop

File system
/src
	> readme_master.txt
	> /Navigation
		> /include
		> /src
			> mission1
			> MAUV_brain
	> /Control
	> /Data