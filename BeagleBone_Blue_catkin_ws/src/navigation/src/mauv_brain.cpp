/*
* @file mauv_brain.cpp
* @Node name: mauv_brain
* @Publishing to:
*
* @Subscribing to:
*
*
* @description
* 	Compares desired pose, position, and conditions published by mission nodes,
* with the current pose, position, and conditions of the MAUV, and makes
* decisions for which commands to call in order to meet desired objectives.
* Also houses bailing conditions to skip all of the mission sequences and call
* bailing node for mission recovery.
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
