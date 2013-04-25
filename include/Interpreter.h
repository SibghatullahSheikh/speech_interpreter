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
#include "speech_interpreter/GetContinue.h"
#include "speech_interpreter/GetYesNo.h"
#include "speech_interpreter/GetCleanup.h"
#include "speech_interpreter/GetOpenChallenge.h"
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
    bool getContinue(speech_interpreter::GetContinue::Request  &req, speech_interpreter::GetContinue::Response &res);
    bool getYesNo(speech_interpreter::GetYesNo::Request  &req, speech_interpreter::GetYesNo::Response &res);
    bool getCleanup(speech_interpreter::GetCleanup::Request  &req, speech_interpreter::GetCleanup::Response &res);
    bool getOpenChallenge(speech_interpreter::GetOpenChallenge::Request  &req, speech_interpreter::GetOpenChallenge::Response &res);

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

    /**
     * Get line number of spoken text without spaces in gpsr_without_spaces.txt
     * And get the text from gpsr_with_spaces.txt
     */
    int getLineNumber (std::string text_at_line, std::string category);
    std::string getTextWithSpaces(int number, std::string category);

    void setColor(int r, int g, int b);

	// Services
	ros::ServiceServer info_service_;
	ros::ServiceServer action_service_;
    ros::ServiceServer continue_service_;
    ros::ServiceServer yes_no_service_;
    ros::ServiceServer cleanup_service_;
    ros::ServiceServer open_challenge_service_;

	// Knowledge maps
	std::map<std::string, std::string> action_map_;
	CategoryMap category_map_;
	std::map<std::string, std::vector<std::string> > action_category_map_;

	// Communication maps
	std::map<std::string, std::pair<ros::ServiceClient, ros::ServiceClient> > category_srv_clients_map_;
	std::map<std::string, ros::Subscriber> category_sub_map_;

	// Reponse user
	std::string answer_;

    ros::Publisher set_rgb_lights_;

    //std::string getSentence(std::string possible_sentences[]);
    std::string getSentence(std::vector<std::string> possible_text);
private:
    bool iExplainedLights;
};

}

#endif /* INTERPRETER_H_ */
