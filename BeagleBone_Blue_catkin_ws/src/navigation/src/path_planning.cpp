/*
* @file path_planner.cpp
* @Node name: path_planner
* @Publishing to:
*
* @Subscribing to:
*
*
* @description
* 	Takes first message published by the mission node, adds to queue, and then
* observes *obstacles* to determine best path to meet next objectives. Computes
* next step and pushes them to an positional objectives queue and the desired position to
* the back of the queue.
*
* @author | initials
* 	Raymond Turrisi, raymond.turrisi@gmail.com | rt
* @contributors | initials
* 	-
* 	-
* @log
* 	9/--/2020:
*
*
*
*/
