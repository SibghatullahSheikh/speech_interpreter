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

#include "speech_interpreter/AskUser.h"

#include <tue_pocketsphinx/Switch.h>

#include "std_msgs/String.h"

#include <psi/Client.h>

namespace SpeechInterpreter {

typedef std::map<std::string, std::vector<std::pair<std::string, int> > > CategoryMap;

class Interpreter {
public:
	Interpreter();
	virtual ~Interpreter();

	/**
	 * Initialize mappings, services
	 */
    void initialize(ros::NodeHandle& nh);
	void initializeSpeechServicesTopics(ros::NodeHandle& nh);

	/**
	 * Services offered
	 */
    bool getInfoSrv(speech_interpreter::GetInfo::Request  &req, speech_interpreter::GetInfo::Response &res);
    bool getActionSrv(speech_interpreter::GetAction::Request  &req, speech_interpreter::GetAction::Response &res);
    bool getContinueSrv(speech_interpreter::GetContinue::Request  &req, speech_interpreter::GetContinue::Response &res);
    bool getYesNoSrv(speech_interpreter::GetYesNo::Request  &req, speech_interpreter::GetYesNo::Response &res);
    bool getCleanupSrv(speech_interpreter::GetCleanup::Request  &req, speech_interpreter::GetCleanup::Response &res);
    bool getOpenChallengeSrv(speech_interpreter::GetOpenChallenge::Request  &req, speech_interpreter::GetOpenChallenge::Response &res);

    bool askUser(speech_interpreter::AskUser::Request  &req, speech_interpreter::AskUser::Response &res);

    bool getAction(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer);
    bool getContinue(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer);
    bool getYesNo(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer);
    bool getCleanup(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer);
    bool getOpenChallenge(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer);
    bool getInfo(const std::string& type, const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer);

	/**
	 * Callbacks
	 */
	void speechCallback(std_msgs::String res);

	/**
	 * Wait for user response over the topic given by the category parameter
	 */
	bool waitForAnswer(std::string category, double t_max);

    /**
     * Wait for user response over the topic given by the category parameter
     */
    bool waitForAction(std::string category, double t_max);

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

    int getPosString(std::string input_text, std::string found_text);

    /**
     * askActionInSteps, gets action in steps, in case action is not understanded in 2 tries
     */
    std::vector<std::string> askActionInSteps(const double time_out_action);

    /**
     * getYesNoFunc, gets action in steps, in case action is not understanded in 2 tries
     */
    std::string getYesNoFunc(bool confirmation, const double n_tries_max, const double time_out);

	// Services
	ros::ServiceServer info_service_;
	ros::ServiceServer action_service_;
    ros::ServiceServer continue_service_;
    ros::ServiceServer yes_no_service_;
    ros::ServiceServer cleanup_service_;
    ros::ServiceServer open_challenge_service_;

    ros::ServiceServer ask_user_service_;

	// Knowledge maps
	std::map<std::string, std::string> action_map_;
    CategoryMap category_map_;
	std::map<std::string, std::vector<std::string> > action_category_map_;

    // pocketsphinx communication
    ros::ServiceClient client_speech_recognition_;
    ros::Subscriber sub_speech_recognition_;

    // Knowledge client
    psi::Client* client_reasoner_;

	// Reponse user
    bool answer_received_;
	std::string answer_;

    ros::Publisher set_rgb_lights_;

    ros::ServiceClient client_speech_;

    //std::string getSentence(std::string possible_sentences[]);
    std::string getSentence(std::vector<std::string> possible_text);
private:
    bool iExplainedLights;
    bool find_me;
    bool find_from;
    bool find_to;
    bool find_exit;
    bool find_at;
    std::vector<std::string> action_heard;
    std::vector<bool> action_heard_keywords;
    std::vector<std::string> action_steps;

    ros::Publisher pub_amigo_speech_sim_; // For using amigo's speech in simulation
};

}

#endif /* INTERPRETER_H_ */
