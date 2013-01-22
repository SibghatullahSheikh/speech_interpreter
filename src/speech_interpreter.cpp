/*
 * speech_interpreter.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jelfring
 */

#include "Interpreter.h"

int main(int argc, char **argv) {

	ros::init (argc, argv, "speech_interpreter");
	ros::NodeHandle nh;

	ROS_INFO("Started speech_interpreter");

	SpeechInterpreter::Interpreter in;

	ros::spin();

	return 0;

}
