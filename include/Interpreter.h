/*
 * Interpreter.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jelfring
 */

#ifndef INTERPRETER_H_
#define INTERPRETER_H_

//ROS
#include <ros/ros.h>

//STL
#include <string>
#include <map>
#include <vector>

// Message/service types
#include "speech_interpreter/GetInfo.h"
#include "speech_interpreter/GetAction.h"
#include "std_msgs/String.h"

namespace SpeechInterpreter {

typedef std::map<std::string, std::vector<std::pair<std::string, int> > > CategoryMap;

class Interpreter {
public:
	Interpreter();
	virtual ~Interpreter();

	/**
	 * Initialize mappings, services
	 */
	void initializeMappings(ros::NodeHandle& nh);
	void initializeSpeechServicesTopics(ros::NodeHandle& nh);

	/**
	 * Services offered
	 */
	bool getInfo(speech_interpreter::GetInfo::Request  &req, speech_interpreter::GetInfo::Response &res);
	bool getAction(speech_interpreter::GetAction::Request  &req, speech_interpreter::GetAction::Response &res);

	/**
	 * Callbacks
	 */
	void speechCallback(std_msgs::String res);

	/**
	 * Wait for user response over the topic given by the category parameter
	 */
	bool waitForAnswer(std::string category, double t_max);

	/**
	 * Ask user for input
	 */
	std::string askUser(std::string type, const unsigned int n_tries_max, const double time_out);

	/**
	 * Let AMIGO speak
	 */
	void amigoSpeak(std::string txt);

	// Services
	ros::ServiceServer info_service_;
	ros::ServiceServer action_service_;

	// Knowledge maps
	std::map<std::string, std::string> action_map_;
	CategoryMap category_map_;
	std::map<std::string, std::vector<std::string> > action_category_map_;

	// Communication maps
	std::map<std::string, std::pair<ros::ServiceClient, ros::ServiceClient> > category_srv_clients_map_;
	std::map<std::string, ros::Subscriber> category_sub_map_;

	// Reponse user
	std::string answer_;
};

}

#endif /* INTERPRETER_H_ */
