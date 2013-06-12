/*
 * Interpreter.cpp
 *
 *  Created on: Nov 20, 2012
 *      Authors: Jos Elfring
 *               Erik Geerts
 */

#include "Interpreter.h"

#include "std_srvs/Empty.h"

#include <text_to_speech_philips/Speak.h>

#include <iostream>
#include <fstream>
using std::ifstream;
using std::cerr;
using std::cout;
#include <ros/package.h>

#include <std_msgs/ColorRGBA.h>
#include <amigo_msgs/RGBLightCommand.h>

#include <vector>

namespace SpeechInterpreter {

Interpreter::Interpreter() : answer_("") {

	// Nodehandle
	ros::NodeHandle nh("~");

	// Initialize mappings
    initialize(nh);

	// Initialize speech recognition services
	initializeSpeechServicesTopics(nh);

	// Offer services

    // these will become obsolete
    info_service_ = nh.advertiseService("get_info_user", &Interpreter::getInfoSrv, this);
    action_service_ = nh.advertiseService("get_action_user", &Interpreter::getActionSrv, this);
    continue_service_ = nh.advertiseService("get_continue", &Interpreter::getContinueSrv, this);
    yes_no_service_ = nh.advertiseService("get_yes_no", &Interpreter::getYesNoSrv, this);
    cleanup_service_ = nh.advertiseService("get_cleanup", &Interpreter::getCleanupSrv, this);
    open_challenge_service_ = nh.advertiseService("get_open_challenge", &Interpreter::getOpenChallengeSrv, this);

    // new generic service
    ask_user_service_  = nh.advertiseService("ask_user", &Interpreter::askUser, this);

    client_speech_ =  nh.serviceClient<text_to_speech_philips::Speak>("/text_to_speech/speak");

    pub_amigo_speech_sim_ = nh.advertise<std_msgs::String>("/amigo_speech_sim", 10); // For using amigo's speech in simulation

    set_rgb_lights_ = nh.advertise<amigo_msgs::RGBLightCommand>("/user_set_rgb_lights", 100);

    // Initialize explained lights for get_action
    iExplainedLights = false;

    find_me = false;
    find_from = false;
    find_to = false;
    find_exit = false;
    find_at = false;
}

Interpreter::~Interpreter() {
    delete client_reasoner_;
}


void Interpreter::initialize(ros::NodeHandle& nh) {

    // connect to reasoner
    client_reasoner_ = new psi::Client("reasoner");

    // load speech interpreter knowledge in reasoner
    client_reasoner_->query(psi::Compound("load_database", psi::Constant("tue_knowledge"), psi::Constant("prolog/speech_interpreter.pl")));

    // load actions synonyms
    std::vector<psi::BindingSet> resp_actions = client_reasoner_->query(psi::Compound("action_map", psi::Variable("A"), psi::Variable("B")));
    for(std::vector<psi::BindingSet>::iterator it = resp_actions.begin(); it != resp_actions.end(); ++it) {
        const psi::BindingSet& set = *it;
        action_map_[set.get("A").toString()] = set.get("B").toString();
        //std::cout << set.get("A").toString() << ", " << set.get("B").toString() << std::endl;
    }

    // load categories
    std::vector<psi::BindingSet> resp_categories = client_reasoner_->query(psi::Compound("category_map", psi::Variable("A"), psi::Variable("B"), psi::Variable("N")));
    for(std::vector<psi::BindingSet>::iterator it = resp_categories.begin(); it != resp_categories.end(); ++it) {
        const psi::BindingSet& set = *it;
        category_map_[set.get("A").toString()].push_back(std::make_pair(set.get("B").toString(), set.get("N").getNumber()));
        //std::cout << set.get("A").toString() << ", " << set.get("B").toString() << ", " << set.get("N").getNumber() << std::endl;
    }

    std::string path = ros::package::getPath("tue_knowledge") + "/speech_recognition";

    // connect to pocketsphinx
    client_speech_recognition_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    client_speech_recognition_.waitForExistence();

    sub_speech_recognition_ = nh.subscribe("/pocketsphinx/output", 1, &Interpreter::speechCallback, this);
}

void Interpreter::initializeSpeechServicesTopics(ros::NodeHandle& nh) {
    // OBSOLETE: already done in initializeMappings
}

bool Interpreter::askUser(speech_interpreter::AskUser::Request  &req, speech_interpreter::AskUser::Response &res) {

    std::map<std::string, std::string> answer;

    if (req.info_type == "action") {
        getAction(req.time_out, req.num_tries, answer);
    } else if (req.info_type == "continue") {
        getContinue(req.time_out, req.num_tries, answer);
    } else if (req.info_type == "yesno") {
        getYesNo(req.time_out, req.num_tries, answer);
    } else if (req.info_type == "cleanup") {
        getCleanup(req.time_out, req.num_tries, answer);
    } else if (req.info_type == "open_challenge") {
        getOpenChallenge(req.time_out, req.num_tries, answer);
    } else {
        getInfo(req.info_type, req.time_out, req.num_tries, answer);
    }

    for(std::map<std::string, std::string>::iterator it = answer.begin(); it != answer.end(); ++it) {
        res.keys.push_back(it->first);
        res.values.push_back(it->second);
    }

    return true;
}

bool Interpreter::getInfoSrv(speech_interpreter::GetInfo::Request  &req, speech_interpreter::GetInfo::Response &res) {
    std::map<std::string, std::string> answer;
    getInfo(req.type, ros::Duration(req.time_out), req.n_tries, answer);
    res.answer = answer["answer"];
    return true;
}

bool Interpreter::getActionSrv(speech_interpreter::GetAction::Request  &req, speech_interpreter::GetAction::Response &res) {
    std::map<std::string, std::string> answer;
    getAction(ros::Duration(req.time_out), 1, answer);
    
    res.action = answer["action"];
    res.end_location = answer["end_location"];
    res.object = answer["object"];
    res.object_location = answer["object_location"];
    res.object_room = answer["object_room"];
    res.start_location = answer["start_location"];
    return true;
}

bool Interpreter::getContinueSrv(speech_interpreter::GetContinue::Request  &req, speech_interpreter::GetContinue::Response &res) {
    std::map<std::string, std::string> answer;
    getContinue(ros::Duration(req.time_out), req.n_tries_max, answer);
    res.answer = answer["answer"];
    return true;
}

bool Interpreter::getYesNoSrv(speech_interpreter::GetYesNo::Request  &req, speech_interpreter::GetYesNo::Response &res) {
    std::map<std::string, std::string> answer;
    getYesNo(ros::Duration(req.time_out), req.n_tries_max, answer);
    res.answer = answer["answer"];
    return true;
}

bool Interpreter::getCleanupSrv(speech_interpreter::GetCleanup::Request  &req, speech_interpreter::GetCleanup::Response &res) {
    std::map<std::string, std::string> answer;
    getCleanup(ros::Duration(req.time_out), req.n_tries_max, answer);
    res.answer = answer["answer"];
    return true;
}

bool Interpreter::getOpenChallengeSrv(speech_interpreter::GetOpenChallenge::Request  &req, speech_interpreter::GetOpenChallenge::Response &res) {
    std::map<std::string, std::string> answer;
    getOpenChallenge(ros::Duration(req.time_out), req.n_tries_max, answer);
    res.answer = answer["answer"];
    return true;
}

/**
 * Service that asks info from user
 */
bool Interpreter::getInfo(const std::string& type, const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer) {

	// Convert to lower case
    std::string type_lower = type;
    std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);
	std::string response = "";

	// Validate input
    if (type_lower == "name") {
        ROS_INFO("I will get you a name, %d tries and time out of %f", max_num_tries, max_duration.toSec());

    }
    else if (type_lower == "drink_cocktail") {
        ROS_INFO("I will get you a drink in %d tries and time out of %f", max_num_tries, max_duration.toSec());

    }
    else if (type_lower == "room_cleanup") {
        // USED IN CLEANUP!!!
        ROS_INFO("I will get you a room in %d tries and time out of %f", max_num_tries, max_duration.toSec());

    }
    else if (type_lower == "demo_challenge") {
        ROS_INFO("I will get you what you want on your sandwich in %d tries and time out of %f", max_num_tries, max_duration.toSec());

    }
    else {

		// Iterate over categories and determine if it is present
		bool return_value = false;
		for (CategoryMap::const_iterator itc = category_map_.begin(); itc != category_map_.end(); ++itc) {
			// Iterate over vector of pairs
			for (std::vector<std::pair<std::string, int> >::const_iterator itp = itc->second.begin(); itp != itc->second.end(); ++itp) {
				// Check if the requested type equals the current category
                if (itp->first == type_lower) {
                    ROS_INFO("I will get you a %s, %d tries and time out of %f", type_lower.c_str(), max_num_tries, max_duration.toSec());
					return_value = true;
				}
			}
		}
        if (iExplainedLights == false) {
            // Explain lights during questioning:
            // Red: Amigo talks
            // Green: Question        // Check action type that is requesteder talks
            setColor(1,0,0); // color red
            std::string explaining_txt = "Before I ask you where I should go, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
            amigoSpeak(explaining_txt);

            iExplainedLights = true;
        }
		// Feedback in case of invalid request
		if (!return_value) {
            ROS_WARN("Speech interpreter received request for %s, which is unknown", type_lower.c_str());
			return false;
		}
	}

    response = askUser(type_lower, max_num_tries, max_duration.toSec());
	ROS_INFO("Answer is %s", response.c_str());

    answer["answer"] = response;

    setColor(0,0,1); // color blue
	return (!response.empty());
}


/**
 * Function that gets an action from a user and asks for missing information
 */
bool Interpreter::getAction(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer) {
        // Check action type that is requested
	// Initial response should be empty
    answer["action"] = "empty";
    answer["start_location"] = "meeting_point";
    answer["end_location"] = "meeting_point";
    answer["object"] = "empty";
    answer["object_room"] = "empty";
    answer["object_location"] = "empty";

    action_category_map_.clear();
    action_steps.clear();

	// Get the action from the user
    ROS_INFO("I will get you an action within %f seconds", max_duration.toSec());
    double t_start = ros::Time::now().toSec();

    if (iExplainedLights == false) {
        // Explain lights during questioning:
        // Red: Amigo talks
        // Green: Questioner talks
        setColor(1,0,0); // color red
        std::string explaining_txt = "Before I ask you what I can do for you, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
        amigoSpeak(explaining_txt);
        
        iExplainedLights = true;
    }

    // Get action description
    max_num_tries = 1e6;
    std::string action = askUser("action", max_num_tries, max_duration.toSec());
    ROS_DEBUG("Received action %s, %f seconds left for refining action", action.c_str(), ros::Time::now().toSec() - t_start);

    // Check if action is heard correctly, otherwise get action in steps
    if (action == "wrong action heard") {
        action_steps = askActionInSteps(max_duration.toSec());

        answer["action"]          = action_steps[0];
        answer["end_location"]    = action_steps[1];
        answer["object"]          = action_steps[2];
        answer["object_location"] = action_steps[3];
        if (answer["object"] != "empty") {
            if (answer["object_room"] == "empty" ) {
                answer["object_room"] = "room_not_known";
            }
            if (answer["object_location"] == "empty") {
                answer["object_location"] = "location_not_known";
            }
        }
        if (answer["end_location"] == "empty") {
            answer["end_location"] = "meeting_point";
        }
        setColor(0,0,1); // color blue
        return true;
    }



    // Check action type that is requested, first determine if action is bring/navigate/go and which category the action then is.
    if (getPosString(action,"bring ") != -1){
        if (find_me) {
            answer["action"] = "get";
        }
        else {
            answer["action"] = "transport";
        }
    }
    if (answer["action"] == "empty") {
        for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
            if (action.find(it->first) != std::string::npos) {
                //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                answer["action"] = it->second;
                break;
            }
        }
    }

	// Check if action is known
    if (answer["action"].empty()) {
        ROS_ERROR("Code error: Action %s received over speech_sentences/output topic does not contain a known action", action.c_str());
        setColor(0,0,1); // color blue
		return false;
	}

    ROS_DEBUG("Action category is %s", answer["action"].c_str());
    bool action_cat_found = false;

    if (answer["action"] == "transport") {
        action_category_map_["transport"].push_back("object_category");    // transport always needs an object
        action_category_map_["transport"].push_back("location_category");  // transport always needs at least one location
    }
    else if (answer["action"] == "get") {
        action_category_map_["get"].push_back("object_category");    // get always needs an object
        if (find_from) {
            action_category_map_["get"].push_back("location_category");  // for asking from which location to grab object.
        }
    }
    else if (answer["action"] == "point") {
        if (find_exit) {
            answer["end_location"] = "exit";
        }
        else if (find_from) {
            action_category_map_["find"].push_back("object_category");
            action_category_map_["find"].push_back("location_category");
            answer["action"] = "find";
            action_cat_found = true;
        }
        else { // determine if object or location should be pointed at.
            for (std::vector<std::pair<std::string, int> >::const_iterator it_cat = category_map_["object_category"].begin();
                    it_cat != category_map_["object_category"].end(); ++it_cat) {
                // Check if action contains current object/location category
                if (action.find(it_cat->first) != std::string::npos) {
                    action_category_map_["find"].push_back("object_category");
                    // finding object -> action 'find' is used
                    answer["action"] = "find";
                    action_cat_found = true;
                }
            }
            if (action_cat_found == false) {
                for (std::vector<std::pair<std::string, int> >::const_iterator it_cat = category_map_["location_category"].begin();
                        it_cat != category_map_["location_category"].end(); ++it_cat) {
                    // Check if action contains current object/location category
                    if (action.find(it_cat->first) != std::string::npos) {
                        action_category_map_["point"].push_back("location_category");
                    }
                }
            }
        }
    }
    else if ((answer["action"] == "find") && (action_cat_found == false)) {
        if (find_exit) {
            answer["end_location"] = "exit";
            // finding location -> action 'point' is used
            answer["action"] = "point";
        }
        else if (find_from) {
            action_category_map_["find"].push_back("object_category");
            action_category_map_["find"].push_back("location_category");
            answer["action"] = "find";
        }
        else { // determine if object or location should be pointed at.
            for (std::vector<std::pair<std::string, int> >::const_iterator it_cat = category_map_["object_category"].begin();
                    it_cat != category_map_["object_category"].end(); ++it_cat) {
                // Check if action contains current object/location category
                if (action.find(it_cat->first) != std::string::npos) {
                    action_category_map_["find"].push_back("object_category");
                    answer["action"] = "find";
                }
            }
            if (action_cat_found == false) {
                for (std::vector<std::pair<std::string, int> >::const_iterator it_cat = category_map_["location_category"].begin();
                        it_cat != category_map_["location_category"].end(); ++it_cat) {
                    // Check if action contains current object/location category
                    if (action.find(it_cat->first) != std::string::npos) {
                        // finding location -> action 'point' is used
                        action_category_map_["point"].push_back("location_category");
                    }

                }
            }
        }
    }
    else if (answer["action"] == "navigate") {
        if (find_exit) {
            answer["end_location"] = "exit";
            answer["action"] = "leave";
        }
        else {
            action_category_map_["navigate"].push_back("location_category");
        }
    }

    ROS_DEBUG("action_category_map determined");

    // All actions in action_map_ must be defined in action_category_map_ as well
    std::map<std::string, std::vector<std::string> >::const_iterator it_action = action_category_map_.find(answer["action"]);

    if (it_action == action_category_map_.end()) {
		ROS_DEBUG("All information is available, no need to ask questions");
        amigoSpeak("I know everything I need to know, I will go to an exit now");
        find_me = false;
        find_from = false;
        find_to = false;
        find_exit = false;
        find_at = false;
        setColor(0,0,1); // color blue
		return true;
	}

	// Ask for additional information
	for (std::vector<std::string>::const_iterator it = it_action->second.begin(); it != it_action->second.end(); ++it) {
        ROS_DEBUG("%s is needed", it->c_str());

		// Check which instance of the object/location is already given
		std::string response = "empty";
		for (std::vector<std::pair<std::string, int> >::const_iterator it_cat = category_map_[(*it)].begin();
				it_cat != category_map_[(*it)].end(); ++it_cat) {

			// Check if action contains current object/location category
			if (action.find(it_cat->first) != std::string::npos) {

				// Found category, check if category has multiple possible values
				if (it_cat->second > 1) {

					// TODO Now fixed maximum number of attempts, change?
                    ROS_DEBUG("%s has %d possible values, need to ask user for more information", it_cat->first.c_str(), it_cat->second);
                    //amigoSpeak("I am sorry, but I need more information."); //" Can you be more specific?"); REMOVED THIS SENTENCE, DOES NOT FEEL NATURAL

                    if ((*it) == "location_category") {
                        if (find_from) {
                            if (answer["action"] == "transport" || answer["action"] == "get") {
                                amigoSpeak("I wonder where you wanted me to get the object from.");
                            }
                        }
                        else if (find_to) {
                            if (answer["action"] == "transport") {
                                amigoSpeak("About the dropoff location,");
                            }
                        }
                    }

                    response = askUser(it_cat->first, 10, max_duration.toSec() - (ros::Time::now().toSec() - t_start));

				} else {
                    ROS_DEBUG("%s has only one possible value", it_cat->first.c_str());
					amigoSpeak("I only know one " + it_cat->first);
					if (it_cat->first == "bathroomstuff") {
						response = "cif";
                    }
					else {
						response = it_cat->first;
					}
				}

                if ((*it) == "object_category") {
                    answer["object"] = response;
                }

                if (!(find_from) && ((*it) == "object_category")) {
                    // If category is object, location is queried. First which room -> if yes -> which specific location?
                    unsigned int n_tries = 0;
                    bool heard_one_answer = false;
                    bool object_room_known = false;
                    std::string response_object_room = "empty";
                    if (response == "no_answer") {
                        answer["object_room"] = "room_not_known";
                        answer["object_location"] = "location_not_known";

                        std::vector<std::string> possible_text;
                        possible_text.push_back("We should start all over.");
                        possible_text.push_back("Let's start at the beginning");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                        ++n_tries;

                    }
                    else if (response == "wrong_answer") {
                        answer["object_room"] = "room_not_known";
                        answer["object_location"] = "location_not_known";
                        // Check action type that is requested, first determine if action is bring/navigate/go and which category the action then is.
                        if (getPosString(action,"bring ") != -1){
                            if (find_me) {
                                answer["action"] = "get";
                            }
                            else {
                                answer["action"] = "transport";
                            }
                        }
                        if (answer["action"] == "empty") {
                            for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
                                if (action.find(it->first) != std::string::npos) {
                                    //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                    ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                    answer["action"] = it->second;
                                    break;
                                }
                            }
                        }
                        std::vector<std::string> possible_text;
                        possible_text.push_back("We should start all over.");
                        possible_text.push_back("Let's start at the beginning");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                        ++n_tries;
                    }
                    else {
                        while ((ros::Time::now().toSec() - t_start < (max_duration.toSec() - (ros::Time::now().toSec() - t_start))) && ros::ok()) {

                            // Check if the response starts with a vowel    // Check action type that is requested, first determine if action is bring/navigate/go and which category the action then is.
                            if (getPosString(action,"bring ") != -1){
                                if (find_me) {
                                    answer["action"] = "get";
                                }
                                else {
                                    answer["action"] = "transport";
                                }
                            }
                            if (answer["action"] == "empty") {
                                for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
                                    if (action.find(it->first) != std::string::npos) {
                                        //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                        ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                        answer["action"] = it->second;
                                        break;
                                    }
                                }
                            }
                            std::string vowels = "auioe";
                            bool start_with_vowel = (vowels.find(response[0])< vowels.length());

                            std::string art = (start_with_vowel)?"an ":"a ";
                            std::string starting_txt = "Are you aware of the room in which I can find " + art + response;
                            amigoSpeak(starting_txt);

                            while (n_tries < 5 && ros::ok()) {
                                if (waitForAnswer("yesno", 10.0)) {
                                    setColor(1,0,0); // color red
                                    // Check if answer is confirmed
                                    if (answer_ == "yes" || answer_ == "y") {
                                        std::vector<std::string> possible_text;
                                        possible_text.push_back("Alright then");
                                        possible_text.push_back("Very well!");
                                        possible_text.push_back("Fine!");
                                        possible_text.push_back("All right!");
                                        possible_text.push_back("Okay!");
                                        std::string sentence;
                                        sentence = getSentence(possible_text);
                                        amigoSpeak(sentence);
                                        object_room_known = true;
                                        response_object_room = askUser("room", 5, max_duration.toSec() - (ros::Time::now().toSec() - t_start));
                                        if (response_object_room == "no_answer") {
                                            object_room_known = false;
                                            amigoSpeak("I will go look for it myself, no problem");
                                        }
                                        else if (response_object_room == "wrong_answer") {
                                            object_room_known = false;
                                            // If category is object, location is queried. First which room -> if yes -> which specific location?
                                            unsigned int n_tries = 0;
                                            bool heard_one_answer = false;
                                            bool object_room_known = false;
                                            std::string response_object_room = "empty";
                                            if ((*it) == "object_category") {
                                                if (response == "no_answer") {
                                                    answer["object_room"] = "room_not_known";
                                                    answer["object_location"] = "location_not_known";

                                                    std::vector<std::string> possible_text;
                                                    possible_text.push_back("We should start all over.");
                                                    possible_text.push_back("Let's start at the beginning");
                                                    std::string sentence;
                                                    sentence = getSentence(possible_text);
                                                    amigoSpeak(sentence);
                                                    ++n_tries;

                                                }
                                                else if (response == "wrong_answer") {
                                                    answer["object_room"] = "room_not_known";
                                                    answer["object_location"] = "location_not_known";

                                                    std::vector<std::string> possible_text;
                                                    possible_text.push_back("We should start all over.");
                                                    possible_text.push_back("Let's start at the beginning");
                                                    std::string sentence;
                                                    sentence = getSentence(possible_text);
                                                    amigoSpeak(sentence);
                                                    ++n_tries;
                                                }
                                                else {
                                                    while ((ros::Time::now().toSec() - t_start < (max_duration.toSec() - (ros::Time::now().toSec() - t_start)))  && ros::ok()) {

                                                        // Check if the response starts with a vowel
                                                        std::string vowels = "auioe";
                                                        bool start_with_vowel = (vowels.find(response[0])< vowels.length());

                                                        std::string art = (start_with_vowel)?"an ":"a ";
                                                        std::string starting_txt = "Are you aware of the room in which I can find " + art + response;
                                                        amigoSpeak(starting_txt);

                                                        while (n_tries < 5 && ros::ok()) {
                                                            if (waitForAnswer("yesno", 10.0)) {
                                                                setColor(1,0,0); // color red
                                                                // Check if answer is confirmed
                                                                if (answer_ == "yes" || answer_ == "y") {
                                                                    std::vector<std::string> possible_text;
                                                                    possible_text.push_back("Alright then");
                                                                    possible_text.push_back("Very well!");
                                                                    possible_text.push_back("Fine!");
                                                                    possible_text.push_back("All right!");
                                                                    possible_text.push_back("Okay!");
                                                                    std::string sentence;
                                                                    sentence = getSentence(possible_text);
                                                                    amigoSpeak(sentence);
                                                                    object_room_known = true;
                                                                    response_object_room = askUser("room", 5, max_duration.toSec() - (ros::Time::now().toSec() - t_start));
                                                                    if (response_object_room == "no_answer") {
                                                                        object_room_known = false;
                                                                        amigoSpeak("I will go look for myself, no problem");
                                                                    }
                                                                    else if (response_object_room == "wrong_answer") {
                                                                        object_room_known = false;
                                                                        amigoSpeak("I will go look for myself, no problem");
                                                                    }
                                                                    break;
                                                                } else {
                                                                    if (!heard_one_answer && !(answer_ == "no")) {
                                                                        std::vector<std::string> possible_text;
                                                                        possible_text.push_back("Could you please answer with yes or no?");
                                                                        possible_text.push_back("Would you answer with yes or no?");
                                                                        std::string sentence;
                                                                        sentence = getSentence(possible_text);
                                                                        amigoSpeak(sentence);
                                                                        heard_one_answer = true;
                                                                        ++n_tries;
                                                                    }
                                                                    else {
                                                                        std::vector<std::string> possible_text;
                                                                        possible_text.push_back("Alright, you do not know where it is, I will try to find it myself.");
                                                                        possible_text.push_back("No problem, I will try to find it myself.");
                                                                        possible_text.push_back("You said that you don't know where it is, therefore I will try to find it myself.");
                                                                        std::string sentence;
                                                                        sentence = getSentence(possible_text);
                                                                        amigoSpeak(sentence);
                                                                        break;
                                                                    }
                                                                }
                                                            }
                                                            else {
                                                                std::vector<std::string> possible_text;
                                                                if ((n_tries + 1) == 5) {
                                                                    possible_text.push_back("I did not hear you for a longer time.");
                                                                    possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                                                }
                                                                else {
                                                                    possible_text.push_back("I did not hear you, I'm sorry. Please say something.");
                                                                    possible_text.push_back("Did you say something? In any case, I was not able to hear you.");
                                                                    possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                                                                }
                                                                std::string sentence;
                                                                sentence = getSentence(possible_text);
                                                                amigoSpeak(sentence);
                                                                ++n_tries;
                                                            }
                                                        }

                                                        // If room is not known, room and exact locations will be unknown. Else continue asking specific location
                                                        if (!object_room_known) {
                                                            answer["object_room"] = "room_not_known";
                                                            answer["object_location"] = "location_not_known";
                                                            break;
                                                        }
                                                        else {
                                                            answer["object_room"] = response_object_room;
                                                            std::string starting_txt = "Since you know which room, are you also aware of the specific location to find " + art + response;
                                                            amigoSpeak(starting_txt);
                                                            n_tries = 0;    // Check action type that is requested, first determine if action is bring/navigate/go and which category the action then is.
                                                            if (getPosString(action,"bring ") != -1){
                                                                if (find_me) {
                                                                    answer["action"] = "get";
                                                                }
                                                                else {
                                                                    answer["action"] = "transport";
                                                                }
                                                            }
                                                            if (answer["action"] == "empty") {
                                                                for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
                                                                    if (action.find(it->first) != std::string::npos) {
                                                                        //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                                                        ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                                                        answer["action"] = it->second;
                                                                        break;
                                                                    }
                                                                }
                                                            }
                                                            bool object_location_known = false;
                                                            std::string response_object_location = "empty";

                                                            while (n_tries < 5 && ros::ok()) {
                                                                if (waitForAnswer("yesno", 10.0)) {
                                                                    setColor(1,0,0); // color red
                                                                    // Check if answer is confirmed
                                                                    if (answer_ == "yes" || answer_ == "y") {
                                                                        std::vector<std::string> possible_text;
                                                                        possible_text.push_back("Alright then");
                                                                        possible_text.push_back("Very well!");
                                                                        possible_text.push_back("Fine!");
                                                                        possible_text.push_back("All right!");
                                                                        possible_text.push_back("Okay!");
                                                                        std::string sentence;
                                                                        sentence = getSentence(possible_text);
                                                                        amigoSpeak(sentence);
                                                                        object_location_known = true;
                                                                        response_object_location = askUser(response_object_room, 5, max_duration.toSec() - (ros::Time::now().toSec() - t_start));
                                                                        if (response_object_location == "no_answer") {
                                                                            // Check action type that is requested, first determine if action is bring/navigate/go and which category the action then is.
                                                                            if (getPosString(action,"bring ") != -1){
                                                                                if (find_me) {
                                                                                    answer["action"] = "get";
                                                                                }
                                                                                else {
                                                                                    answer["action"] = "transport";
                                                                                }
                                                                            }
                                                                            if (answer["action"] == "empty") {
                                                                                for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
                                                                                    if (action.find(it->first) != std::string::npos) {
                                                                                        //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                                                                        ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                                                                                        answer["action"] = it->second;
                                                                                        break;
                                                                                    }
                                                                                }
                                                                            }  object_location_known = false;
                                                                            std::string txt_room = "I will go look for myself in the " + response_object_room + ". No problem!";
                                                                            amigoSpeak(txt_room);
                                                                        }
                                                                        else if (response_object_location == "wrong_answer") {
                                                                            object_location_known = false;
                                                                            std::string txt_room = "I will go look for myself in the " + response_object_room + ". No problem!";
                                                                            amigoSpeak(txt_room);
                                                                        }
                                                                        break;
                                                                    }
                                                                    else {
                                                                        if (!heard_one_answer && !(answer_ == "no")) {
                                                                            std::vector<std::string> possible_text;
                                                                            possible_text.push_back("Could you please answer with yes or no?");
                                                                            possible_text.push_back("Would you answer with yes or no?");
                                                                            std::string sentence;
                                                                            sentence = getSentence(possible_text);
                                                                            amigoSpeak(sentence);
                                                                            heard_one_answer = true;
                                                                            ++n_tries;
                                                                        }
                                                                        else {
                                                                            std::vector<std::string> possible_text;
                                                                            possible_text.push_back("Alright, you do not know where it is exactly. I will look in the " + response_object_room);
                                                                            possible_text.push_back("No problem, I will try to find it myself. I will look in the " + response_object_room);
                                                                            possible_text.push_back("You said that you don't know where it is, therefore I will look in the " + response_object_room);
                                                                            std::string sentence;
                                                                            sentence = getSentence(possible_text);
                                                                            amigoSpeak(sentence);
                                                                            break;
                                                                        }
                                                                    }
                                                                }
                                                                else {
                                                                    std::vector<std::string> possible_text;
                                                                    if ((n_tries + 1) == 5) {
                                                                        possible_text.push_back("I did not hear you for a longer time.");
                                                                        possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                                                    }
                                                                    else {
                                                                        possible_text.push_back("I did not hear you, I'm sorry. Please say something.");
                                                                        possible_text.push_back("Did you say something? In any case, I was not able to hear you.");
                                                                        possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                                                                    }
                                                                    std::string sentence;
                                                                    sentence = getSentence(possible_text);
                                                                    amigoSpeak(sentence);
                                                                    ++n_tries;
                                                                }
                                                            }

                                                            // Check if location object is known.
                                                            if (!object_location_known) {
                                                                answer["object_location"] = "location_not_known";
                                                                break;
                                                            }
                                                            else {
                                                                answer["object_location"] = response_object_location;
                                                                break;
                                                            }
                                                        }
                                                    } // end while loop
                                                }
                                            }
                                            break;     amigoSpeak("I will go look for myself, no problem");
                                        }
                                        break;
                                    } else {
                                        if (!heard_one_answer && !(answer_ == "no")) {
                                            std::vector<std::string> possible_text;
                                            possible_text.push_back("Could you please answer with yes or no?");
                                            possible_text.push_back("Would you answer with yes or no?");
                                            std::string sentence;
                                            sentence = getSentence(possible_text);
                                            amigoSpeak(sentence);
                                            heard_one_answer = true;
                                            ++n_tries;
                                        }
                                        else {
                                            std::vector<std::string> possible_text;
                                            possible_text.push_back("Alright, you do not know where it is, I will try to find it myself.");
                                            possible_text.push_back("No problem, I will try to find it myself.");
                                            possible_text.push_back("You said that you don't know where it is, therefore I will try to find it myself.");
                                            std::string sentence;
                                            sentence = getSentence(possible_text);
                                            amigoSpeak(sentence);
                                            break;
                                        }
                                    }
                                }
                                else {
                                    std::vector<std::string> possible_text;
                                    if ((n_tries + 1) == 5) {
                                        possible_text.push_back("I did not hear you for a longer time.");
                                        possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                    }
                                    else {
                                        possible_text.push_back("I did not hear you, I'm sorry. Please say something.");
                                        possible_text.push_back("Did you say something? In any case, I was not able to hear you.");
                                        possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                                    }
                                    std::string sentence;
                                    sentence = getSentence(possible_text);
                                    amigoSpeak(sentence);
                                    ++n_tries;
                                }
                            }

                            // If room is not known, room and exact locations will be unknown. Else continue asking specific location
                            if (!object_room_known) {
                                answer["object_room"] = "room_not_known";
                                answer["object_location"] = "location_not_known";
                                break;
                            }
                            else {
                                answer["object_room"] = response_object_room;
                                std::string starting_txt = "Since you know which room, are you also aware of the specific location to find " + art + response;
                                amigoSpeak(starting_txt);
                                n_tries = 0;
                                bool object_location_known = false;
                                std::string response_object_location = "empty";

                                while (n_tries < 5 && ros::ok()) {
                                    if (waitForAnswer("yesno", 10.0)) {
                                        setColor(1,0,0); // color red
                                        // Check if answer is confirmed
                                        if (answer_ == "yes" || answer_ == "y") {
                                            std::vector<std::string> possible_text;
                                            possible_text.push_back("Alright then");
                                            possible_text.push_back("Very well!");
                                            possible_text.push_back("Fine!");
                                            possible_text.push_back("All right!");
                                            possible_text.push_back("Okay!");
                                            std::string sentence;
                                            sentence = getSentence(possible_text);
                                            amigoSpeak(sentence);
                                            object_location_known = true;
                                            response_object_location = askUser(response_object_room, 5, max_duration.toSec() - (ros::Time::now().toSec() - t_start));
                                            if (response_object_location == "no_answer") {
                                                object_location_known = false;
                                                std::string txt_room = "I will go look for myself in the " + response_object_room + ". No problem!";
                                                amigoSpeak(txt_room);
                                            }
                                            else if (response_object_location == "wrong_answer") {
                                                object_location_known = false;
                                                std::string txt_room = "I will go look for myself in the " + response_object_room + ". No problem!";
                                                amigoSpeak(txt_room);
                                            }
                                            break;
                                        }
                                        else {
                                            if (!heard_one_answer && !(answer_ == "no")) {
                                                std::vector<std::string> possible_text;
                                                possible_text.push_back("Could you please answer with yes or no?");
                                                possible_text.push_back("Would you answer with yes or no?");
                                                std::string sentence;
                                                sentence = getSentence(possible_text);
                                                amigoSpeak(sentence);
                                                heard_one_answer = true;
                                                ++n_tries;
                                            }
                                            else {
                                                std::vector<std::string> possible_text;
                                                possible_text.push_back("Alright, you do not know where it is exactly. I will look in the " + response_object_room);
                                                possible_text.push_back("No problem, I will try to find it myself. I will look in the " + response_object_room);
                                                possible_text.push_back("You said that you don't know where it is, therefore I will look in the " + response_object_room);
                                                std::string sentence;
                                                sentence = getSentence(possible_text);
                                                amigoSpeak(sentence);
                                                break;
                                            }
                                        }
                                    }
                                    else {
                                        std::vector<std::string> possible_text;
                                        if ((n_tries + 1) == 5) {
                                            possible_text.push_back("I did not hear you for a longer time.");
                                            possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                        }
                                        else {
                                            possible_text.push_back("I did not hear you, I'm sorry. Please say something.");
                                            possible_text.push_back("Did you say something? In any case, I was not able to hear you.");
                                            possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                                        }
                                        std::string sentence;
                                        sentence = getSentence(possible_text);
                                        amigoSpeak(sentence);
                                        ++n_tries;
                                    }
                                }

                                // Check if location object is known.
                                if (!object_location_known) {
                                    answer["object_location"] = "location_not_known";
                                    break;
                                }
                                else {
                                    answer["object_location"] = response_object_location;
                                    break;
                                }
                            }
                        } // end while loop
                    }
                    break;
                }
                // Response to user
                if ((*it) == "location_category") {
                    if (find_from) {
                        answer["object_location"] = response;
                        find_from=false;
                    }
                    else {
                        answer["end_location"] = response;
                    }
                }
            }

		}
	}
    find_me = false;
    find_from = false;
    find_to = false;
    find_exit = false;
    find_at = false;
    setColor(0,0,1); // color blue
	return true;
}


/**
 * Callback for speech
 */
void Interpreter::speechCallback(std_msgs::String res) {

	if (res.data != "empty") {
		answer_ = res.data;
		std::transform(answer_.begin(), answer_.end(), answer_.begin(), ::tolower);
        answer_received_ = true;
		ROS_DEBUG("Received response: %s", res.data.c_str());
	}
}

/**
 * Function that switches on the requested speech recognition module and waits for an answer over the appropriate topic
 */
bool Interpreter::waitForAnswer(std::string category, double t_max) {

    // Check for key words
    if (category == "sentences") {
        find_me = false;
        find_from = false;
        find_to = false;
        find_exit = false;
        find_at = false;
    }

    // query reasoner for pocketsphinx configuration
    std::vector<psi::BindingSet> resp_config = client_reasoner_->query(psi::Compound("pocketsphinx_config",
                                                                                        psi::Constant(category),
                                                                                        psi::Variable("HMM"),
                                                                                        psi::Variable("LM"),
                                                                                        psi::Variable("DIC"),
                                                                                        psi::Variable("FSG")));

    if (resp_config.empty()) {
        ROS_WARN("Could not find pocketsphinx configuration for category %s", category.c_str());
        return false;
    }

    psi::BindingSet& resp_config_set = resp_config[0];

    // create service request
    tue_pocketsphinx::Switch::Request req_start;
    req_start.action = tue_pocketsphinx::Switch::Request::START;
    req_start.dictionary = resp_config_set.get("DIC").toString();
    req_start.finite_state_grammer = resp_config_set.get("FSG").toString();
    req_start.hidden_markov_model = resp_config_set.get("HMM").toString();
    req_start.language_model = resp_config_set.get("LM").toString();

    if (req_start.finite_state_grammer == "NONE") {
        req_start.finite_state_grammer = "";
    }

    tue_pocketsphinx::Switch::Response resp_start;

	answer_.clear();

	ros::Rate r(10);

    // loop until we have a valid answer
    while ((answer_.empty() || (answer_== " ")|| (answer_== "  ") || (answer_== "   ")) && ros::ok()) {

        // start speech recognition
        if (client_speech_recognition_.call(req_start, resp_start)) {
            if (resp_start.error_msg == "") {
                ROS_INFO("Switched on speech recognition for: %s\n hmm = %s\nlm = %s\ndic = %s\nfsg = %s",
                         category.c_str(), req_start.hidden_markov_model.c_str(), req_start.dictionary.c_str(),
                         req_start.dictionary.c_str(), req_start.finite_state_grammer.c_str());
                setColor(0,1,0); // color green
            } else {
                ROS_WARN("Unable to turn on speech recognition for %s: %s", category.c_str(), resp_start.error_msg.c_str());
            }
        } else {
            ROS_WARN("Service call for turning on speech recognition for %s failed", category.c_str());
        }

        answer_received_ = false;
        double start_time = ros::Time::now().toSec();

        // wait until we received an answer from speech recognition
        while(!answer_received_ && ros::ok()) {
            if (ros::Time::now().toSec() - start_time > t_max) {
                ROS_WARN("Timeout: No input over speech topic %s for %f seconds", category.c_str(), t_max);
                break;
            }
            ros::spinOnce();
            r.sleep();
        }

        r.sleep();
    }
    setColor(1,0,0); // color red

    // If input is more than one word -> only pick the first word:
    if (answer_.size() > 2 && ! (category == "sentences")) {
        unsigned found=answer_.find(" ");
        std::string part = answer_.substr(0,found);
        answer_= part;
    }

    // Check for key words
    if (category == "sentences") {

        if (getPosString(answer_," me ") != -1){
            find_me = true;
            answer_.replace(answer_.find(" me "), 4, " you ");
        }
        if (getPosString(answer_," from ") != -1){
            find_from = true;
        }
        if (getPosString(answer_," to ") != -1){
            find_to = true;
        }
        if (getPosString(answer_," exit") != -1){
            find_exit = true;
        }
        if (getPosString(answer_," at ") != -1){
            find_at = true;
        }
    }

    // Turn off speech recognition
    tue_pocketsphinx::Switch::Request req_stop;
    req_stop.action = tue_pocketsphinx::Switch::Request::STOP;

    tue_pocketsphinx::Switch::Response resp_stop;
    if (client_speech_recognition_.call(req_stop, resp_stop)) {
        if (resp_stop.error_msg == "") {
            ROS_INFO("Switched off speech recognition for: %s", category.c_str());
        } else {
            ROS_WARN("Unable to turn off speech recognition: %s", resp_stop.error_msg.c_str());
        }
    } else {
        ROS_WARN("Unable to turn off speech recognition for: %s", category.c_str());
    }

	return (!answer_.empty());
}


/**
 * Function that does the actual human robot interaction, i.e., ask the right questions
 */
std::string Interpreter::askUser(std::string type, const unsigned int n_tries_max, const double time_out) {

	// Initialize
	double t_start = ros::Time::now().toSec();
	unsigned int n_tries = 0;
    unsigned int n_tries_before_deeper_questioning = 0;

	// Maximum waiting time per question
    double t_max_question = 10.0;   // before = std::max(9.0, time_out/(2.0*n_tries_max));

	// Check if the type starts with a vowel
	std::string vowels = "auioe";
	bool start_with_vowel = (vowels.find(type[0])< vowels.length());

	// Determine first question
    std::string txt = "", starting_txt, result = "";
    if (type == "name") {
        if (iExplainedLights == false) {
            // Explain lights during questioning:
            // Red: Amigo talks
            // Green: Questioner talks
            setColor(1,0,0); // color red
            std::string explaining_txt = "Before I ask you where I should go, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
            amigoSpeak(explaining_txt);

            iExplainedLights = true;
        }
        std::vector<std::string> possible_text;
        possible_text.push_back("Could you please tell me your name?");
        possible_text.push_back("Can you give me your name?");
        possible_text.push_back("What is your name?");
        starting_txt = getSentence(possible_text);
	} else if (type == "action") {
        // Remap type such that it matches the topic name
        type = "sentences";
        starting_txt = "What can I do for you?";
        t_max_question = 300; // = 5 minutes, in e-gpsr 2013 amigo should handle waiting long time for input.
    } else if (type == "bedroom" || type =="livingroom" || type == "kitchen" || type == "lobby") {
        starting_txt = "Can you give me the exact location?";
    } else if (type == "cleanup") {
        starting_txt = "What do you want me to do";
    } else if (type == "open_challenge") {
        starting_txt = "Where do you want me to go";
    } else if (type == "demo_challenge") {
        starting_txt = "";
    } else if (type == "drink_cocktail") {
        if (iExplainedLights == false) {
            // Explain lights during questioning:
            // Red: Amigo talks
            // Green: Questioner talks
            setColor(1,0,0); // color red
            std::string explaining_txt = "Before I ask you what drink you would like, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
            amigoSpeak(explaining_txt);

            iExplainedLights = true;
        }
        std::vector<std::string> possible_text;
        possible_text.push_back("Could you please tell me what drink you want?");
        possible_text.push_back("What drink would you like to have?");
        possible_text.push_back("Which drink can I serve you?");
        starting_txt = getSentence(possible_text);
    } else if (type == "room_cleanup") {
        if (iExplainedLights == false) {
            // Explain lights during questioning:
            // Red: Amigo talks
            // Green: Questioner talks
            setColor(1,0,0); // color reds
            /*std::string explaining_txt = "Before I ask you which room you would like me to clean, I just want to tell you that if my"
						"lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
			*/		
			std::string explaining_txt = "Would you please answer the question when I become green."
            amigoSpeak(explaining_txt);

            iExplainedLights = true;
        }
        type = "cleanup";
        std::vector<std::string> possible_text;
        possible_text.push_back("What can I do for you?");
        starting_txt = getSentence(possible_text);

    } else if (type == "location_classes"){
        starting_txt = "Can you specify which location class you mean?";

    } else if (type == "object_classes"){
        starting_txt = "Can you specify which object class you mean?";

    } else if (type == "object_or_location"){
        ROS_DEBUG("No starting text for object_or_location");

    } else {
		std::string art = (start_with_vowel)?"an ":"a ";
        starting_txt = "Can you specify which " + type + " you mean?";
    }    

    // Ask
    amigoSpeak(starting_txt);
    ROS_DEBUG("Max time to wait for answer = %f", t_max_question);
    while ((ros::Time::now().toSec() - t_start < time_out && n_tries < n_tries_max)  && ros::ok()) {

        if ((type == "sentences") && (n_tries_before_deeper_questioning == 2)) {
            result = "wrong action heard";
            break;
        }


        // If an answer was heared, verify
        if (waitForAnswer(type, t_max_question)) {
            setColor(1,0,0); // color red
            result = answer_;
            int line_number;
            std::string result2;

            if ( type == "sentences") {
                amigoSpeak("Did you want me to " + result);
                std::vector<std::string> possible_text;
                possible_text.push_back("Is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                possible_text.push_back("Am I right?");
                possible_text.push_back("Is that okay?");
                possible_text.push_back("Is that alright?");
                std::string sentence;
                sentence = getSentence(possible_text);
                amigoSpeak(sentence);
            }
            else if ( type == "cleanup") {

                unsigned found1=result.find("cleanupthe");
                unsigned found2=result.find(" ");
                std::string part = result.substr((found1+10),found2);
                result = part;

                amigoSpeak("I heard clean up the " + result);
                std::vector<std::string> possible_text;
                possible_text.push_back("Is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                possible_text.push_back("Am I right?");
                possible_text.push_back("Is that okay?");
                possible_text.push_back("Is that alright?");
                std::string sentence;
                sentence = getSentence(possible_text);
                amigoSpeak(sentence);
            }
            else if ( type == "open_challenge") {
                line_number = getLineNumber(answer_, "open_challenge");
                result2 = getTextWithSpaces(line_number, "open_challenge");

                amigoSpeak("I heard " + result2);
                std::vector<std::string> possible_text;
                possible_text.push_back("Is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                possible_text.push_back("Am I right?");
                possible_text.push_back("Is that okay?");
                possible_text.push_back("Is that alright?");
                std::string sentence;
                sentence = getSentence(possible_text);
                amigoSpeak(sentence);
            }
            else if ( type == "demo_challenge") {
                line_number = getLineNumber(answer_, "demo_challenge");
                result2 = getTextWithSpaces(line_number, "demo_challenge");

                amigoSpeak("I heard " + result2);
                std::vector<std::string> possible_text;
                possible_text.push_back("Is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                possible_text.push_back("Am I right?");
                possible_text.push_back("Is that okay?");
                possible_text.push_back("Is that alright?");
                std::string sentence;
                sentence = getSentence(possible_text);
                amigoSpeak(sentence);
            }
            else {
                amigoSpeak("I heard " + result);
                std::vector<std::string> possible_text;
                possible_text.push_back("Is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                possible_text.push_back("Am I right?");
                possible_text.push_back("Is that okay?");
                possible_text.push_back("Is that alright?");
                std::string sentence;
                sentence = getSentence(possible_text);
                amigoSpeak(sentence);
            }

            // If answer received, ask for confirmation
            if (waitForAnswer("yesno", 10.0)) {
                setColor(1,0,0); // color red
				// Check if answer is confirmed
				if (answer_ == "yes" || answer_ == "y") {
                    std::vector<std::string> possible_text;
                    possible_text.push_back("Alright then");
                    possible_text.push_back("Very well!");
                    possible_text.push_back("Fine!");
                    possible_text.push_back("All right!");
                    possible_text.push_back("Okay!");
                    std::string sentence;
                    sentence = getSentence(possible_text);
                    amigoSpeak(sentence);
                    break;
                } else {
                    if (type == "sentences") {
                        ++n_tries_before_deeper_questioning;
                        // If input is more than one word -> only pick the first word:
                        if (result.size() > 2) {
                            unsigned found=result.find(" ");
                            std::string part = result.substr(0,found);
                            action_heard.push_back(part);
                            action_heard_keywords.push_back(find_me);
                            action_heard_keywords.push_back(find_from);
                            action_heard_keywords.push_back(find_to);
                            action_heard_keywords.push_back(find_exit);
                            action_heard_keywords.push_back(find_at);
                        }
                    }
                    result = "wrong_answer";

                    if (n_tries_before_deeper_questioning != 2) {
                        std::vector<std::string> possible_text;
                        //ROS_DEBUG("Continue to smaller questions to get to the right action");
                        if ((n_tries + 1) == n_tries_max) {
                            possible_text.push_back("I was not able to understand what you were saying for a longer time, I'm sorry.");
                            possible_text.push_back("I'm sorry, I was not able to understand what you were saying for a longer time.");
                        }
                        else {
                            possible_text.push_back("I understood that it is not correct, lets try it again. Could you please repeat it");
                            possible_text.push_back("Apparently I mis understood, I'm sorry, could you please repeat it");
                            possible_text.push_back("I understood that it is not correct, lets try it again. Could you please repeat it");
                            possible_text.push_back("I was wrong, I'm sorry, could you please repeat it");
                            possible_text.push_back("Maybe there is something wrong with my ears, I'm sorry, could you please repeat it");
                        }
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                    }
                    ++n_tries;
				}
			}

			// If no answer heard to confirmation question, ask for confirmation again
			else {
                setColor(1,0,0); // color red
                if (type == "cleanup" || type == "open_challenge" || type == "demo_challenge") {
                    std::vector<std::string> possible_text;
                    possible_text.push_back("I did not hear you, did you say "+ result2);
                    possible_text.push_back("Did you say "+ result2 + ". Could you confirm that?");
                    std::string sentence;
                    sentence = getSentence(possible_text);
                    amigoSpeak(sentence);
                }
                else {
                    std::vector<std::string> possible_text;
                    possible_text.push_back("I did not hear you, did you say "+ result);
                    possible_text.push_back("Did you say "+ result + ". Could you confirm that?");
                    std::string sentence;
                    sentence = getSentence(possible_text);
                    amigoSpeak(sentence);
                }

                // Check if the answer is confirmed after the secServiceond confirmation question
                if (waitForAnswer("yesno", 10.0)) {
                    setColor(1,0,0); // color red
					// Check if answer is confirmed (second time)
					if (answer_ == "yes" || answer_ == "y") {
	                    std::vector<std::string> possible_text;
	                    possible_text.push_back("Alright then");
	                    possible_text.push_back("Very well!");
	                    possible_text.push_back("Fine!");
	                    possible_text.push_back("All right!");
	                    possible_text.push_back("Okay!");
	                    std::string sentence;
	                    sentence = getSentence(possible_text);
	                    amigoSpeak(sentence);
						break;
					} else {
                        if (type == "sentences") {
                            ++n_tries_before_deeper_questioning;
                            // If input is more than one word -> only pick the first word:
                            if (result.size() > 2) {
                                unsigned found=result.find(" ");
                                std::string part = result.substr(0,found);
                                action_heard.push_back(part);
                                action_heard_keywords.push_back(find_me);
                                action_heard_keywords.push_back(find_from);
                                action_heard_keywords.push_back(find_to);
                                action_heard_keywords.push_back(find_exit);
                                action_heard_keywords.push_back(find_at);
                            }
                        }
						result = "wrong_answer";
                        if (n_tries_before_deeper_questioning != 2) {
                            std::vector<std::string> possible_text;
                            //ROS_DEBUG("Continue to smaller questions to get to the right action");
                            if ((n_tries + 1) == n_tries_max) {
                                possible_text.push_back("I was not able to understand what you were saying for a longer time, I'm sorry.");
                                possible_text.push_back("I'm sorry, I was not able to understand what you were saying for a longer time.");
                            }
                            else {
                                possible_text.push_back("I understood that it is not correct, lets try it again. Could you please repeat it");
                                possible_text.push_back("Apparently I mis understood, I'm sorry, could you please repeat it");
                                possible_text.push_back("I understood that it is not correct, lets try it again. Could you please repeat it");
                                possible_text.push_back("I was wrong, I'm sorry, could you please repeat it");
                                possible_text.push_back("Maybe there is something wrong with my ears, I'm sorry, could you please repeat it");
                            }
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);
                        }
                        ++n_tries;
					}
				}

				// Second confirmation question again did not lead to a yes or no, start all over again
				else {
                    setColor(1,0,0); // color red
                    result = "no_answer";
                    if (type == "sentences" || type == "cleanup" || type == "open_challenge" || type == "demo_challenge") {
                        std::vector<std::string> possible_text;
                        possible_text.push_back("I'm sorry, I didn't hear a confirmation. Could you please repeat what you want me to do?");
                        possible_text.push_back("I didn't hear a confirmation, sorry. Could you please repeat what you want me to do?");
                        possible_text.push_back("I didn't hear a confirmation, sorry. What did you want me to do?");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                    }
                    else {
                        std::string art = (start_with_vowel)?"an ":"a ";
                        std::vector<std::string> possible_text;
                        possible_text.push_back("I'm sorry, I didn't hear a confirmation. Could you please specify which " + type + " you mean?");
                        possible_text.push_back("I didn't hear a confirmation, sorry. Could you please specify which " + type + " you mean?");
                        possible_text.push_back("I didn't hear a confirmation, sorry. Which " + type + " did you mean?");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                    }
					++n_tries;
				}
			}
		}

		// If no answer, ask again

		else {
            setColor(1,0,0); // color red
            result = "no_answer";
            std::vector<std::string> possible_text;
            if ((n_tries + 1) == n_tries_max) {
                possible_text.push_back("I did not hear you for a longer time.");
                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
            }
            else {
                possible_text.push_back("I did not hear you, I'm sorry. Please say something.");
                possible_text.push_back("Did you say something? In any case, I was not able to hear you.");
                possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
            }
            std::string sentence;
            sentence = getSentence(possible_text);
            amigoSpeak(sentence);

            ROS_DEBUG("n_tries before = %i", n_tries);
            ++n_tries;
            ROS_DEBUG("n_tries now = %i", n_tries);
            ROS_DEBUG("n_tries_max = %i", n_tries_max);
            double time_check = ros::Time::now().toSec() - t_start;
            ROS_DEBUG("rostime: now - t_start = %f", time_check);
            ROS_DEBUG("time_out = %f", time_out);
		}
	} 
    return result;
}

/**
 * Function that gets action in steps in case action is not understood corectly via askUser()
 */
std::vector<std::string> Interpreter::askActionInSteps(const double time_out) {

    // Initialize
    double t_start = ros::Time::now().toSec();
    double t_max_question = 10.0;
    bool confirmation = false;
    bool no_answer = false;
    int n_tries_yesno = 2;
    std::vector<std::string> action_steps;
    action_steps.clear();
    std::string answer_yes_no = " ";

    std::string answer_object_class;
    std::string answer_object_exact = "empty";
    std::string answer_location_class;
    std::string answer_location_exact_from = "empty";
    std::string answer_location_exact_to = "empty";
    std::string answer_location_or_object;

    ROS_INFO("First action heard was = '%s'", action_heard[0].c_str());
    ROS_INFO("Second action heard was = '%s'", action_heard[1].c_str());

    amigoSpeak("I was still not able to understand you correctly. I will try to get the action with smaller steps.");

    ROS_DEBUG("Max time to wait for answer = %f", t_max_question);
    while ((ros::Time::now().toSec() - t_start < time_out) && ros::ok()) {


    //            action_heard_keywords.push_back(find_me);
    //            action_heard_keywords.push_back(find_from);
    //            action_heard_keywords.push_back(find_to);
    //            action_heard_keywords.push_back(find_exit);
    //            action_heard_keywords.push_back(find_at);

        /////////////////////////////////////////////////////////
        // Step 1: determine action class of each heard action //
        /////////////////////////////////////////////////////////

        std::string action_1 = "empty";
        std::string action_2 = "empty";

        // Action 1:
        if (getPosString(action_heard[0],"bring") != -1){
            if (action_heard_keywords[0]) {  // =  find_me of action1
                action_1 = "get";
            }
            else {
                action_1 = "transport";
            }
        }

       if (action_1 == "empty") {
            for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
                if (action_heard[0].find(it->first) != std::string::npos) {
                    //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                    ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                    action_1 = it->second;
                    break;
                }
            }
        }
        ROS_INFO("first action class = %s",action_1.c_str());

        // Action 2:

        if (getPosString(action_heard[1],"bring") != -1){
            if (action_heard_keywords[5]) { // =  find_me of action2
                action_2 = "get";
            }
            else {
                action_2 = "transport";
            }
        }
        if (action_2 == "empty") {
            for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
                if (action_heard[1].find(it->first) != std::string::npos) {
                    //ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                    ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
                    action_2 = it->second;
                    break;
                }
            }
        }
        ROS_INFO("second action class = %s",action_2.c_str());

        /////////////////////////////////////////////////////////
        //           Step 2: determine correct action          //
        /////////////////////////////////////////////////////////

        while (action_steps.empty() && ros::ok()) {
            no_answer = false;
            // If action_1 and action_2 are the same:
            if (action_1 == action_2) {
                if (action_1 == "transport") {
                    amigoSpeak("My guess is that you want me to transport something to a location, that is not yours, is that corect?");
                }
                else if (action_1 == "get") {
                    amigoSpeak("My guess is that you want me to get you something, is that corect?");
                }
                else if (action_1 == "point") {
                    amigoSpeak("My guess is that you want me to point at a location or object, is that corect?");
                }
                else if (action_1 == "find") {
                    amigoSpeak("My guess is that you want me to find a location or object, is that corect?");
                }
                else if (action_1 == "navigate") {
                    amigoSpeak("My guess is that you want me to navigate to a location, is that corect?");
                }
                answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                setColor(1,0,0);
                if (answer_yes_no == "yes") {
                    action_steps.push_back(action_1);
                    break;
                }
                else if (answer_yes_no == "no_answer") {
                    amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                    no_answer = true;
                }
            }
        // If two actions are not the same.
            else if (!(action_1 == action_2)) {
                ROS_INFO("Actions are of a different class.");
                if (action_2 == "transport") {
                    amigoSpeak("Do you want me to transport something to a location, that is not yours, is that corect?");
                }
                else if (action_2 == "get") {
                    amigoSpeak("Do you want me to get you something, is that corect?");
                }
                else if (action_2 == "point") {
                    amigoSpeak("Do you want me to point at a location or object, is that corect?");
                }
                else if (action_2 == "find") {
                    amigoSpeak("Do you want me to find a location or object, is that corect?");
                }
                else if (action_2 == "navigate") {
                    amigoSpeak("Do you want me to navigate to a location, is that corect?");
                }

                answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                setColor(1,0,0);
                if (answer_yes_no == "yes") {
                    action_steps.push_back(action_2);
                    break;
                }
                else if (answer_yes_no == "no_answer") {
                    amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                    no_answer = true;
                }
                else if (answer_yes_no == "no") {
                    if (action_1 == "transport") {
                        amigoSpeak("Do you want me to transport something to a location, that is not yours, is that corect?");
                    }
                    else if (action_1 == "get") {
                        amigoSpeak("Do you want me to get you something, is that corect?");
                    }
                    else if (action_1 == "point") {
                        amigoSpeak("Do you want me to point at a location or object, is that corect?");
                    }
                    else if (action_1 == "find") {
                        amigoSpeak("Do you want me to find a location or object, is that corect?");
                    }
                    else if (action_1 == "navigate") {
                        amigoSpeak("Do you want me to navigate to a location, is that corect?");
                    }

                    answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                    setColor(1,0,0);
                    if (answer_yes_no == "yes") {
                        action_steps.push_back(action_1);
                        break;
                    }
                    else if (answer_yes_no == "no_answer") {
                        amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                        no_answer = true;
                    }
                }
            }

            if (!(no_answer)) {
                amigoSpeak("Now I really wonder what action you would like me to perform.");
                if (!(action_1 == "get") && !(action_2 == "get")) {
                    amigoSpeak("Should I get you something?");
                    answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                    setColor(1,0,0);
                    if (answer_yes_no == "yes") {
                        action_steps.push_back("get");
                        break;
                    }
                    else if (answer_yes_no == "no_answer") {
                        amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                        no_answer = true;
                    }
                }

                if (!(action_1 == "transport") && !(action_2 == "transport")) {
                    amigoSpeak("Should I transport an object?");
                    answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                    setColor(1,0,0);
                    if (answer_yes_no == "yes") {
                        action_steps.push_back("transport");
                        break;
                    }
                    else if (answer_yes_no == "no_answer") {
                        amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                        no_answer = true;
                    }
                }

                if (!(action_1 == "point") && !(action_2 == "point")) {
                    amigoSpeak("Should I point at at a location or object?");
                    answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                    setColor(1,0,0);
                    if (answer_yes_no == "yes") {
                        action_steps.push_back("point");
                        break;
                    }
                    else if (answer_yes_no == "no_answer") {
                        amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                        no_answer = true;
                    }
                }

                if  (!(action_1 == "find") && !(action_2 == "find")) {
                    amigoSpeak("Should I find a location or an object, yes or no?");
                    answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                    setColor(1,0,0);
                    if (answer_yes_no == "yes") {
                        action_steps.push_back("find");
                        break;
                    }
                    else if (answer_yes_no == "no_answer") {
                        amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                        no_answer = true;
                    }
                }

                if  (!(action_1 == "navigate") && !(action_2 == "navigate")) {
                    amigoSpeak("Should I navigate to a location?");
                    answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                    setColor(1,0,0);
                    if (answer_yes_no == "yes") {
                        action_steps.push_back("navigate");
                        break;
                    }
                    else if (answer_yes_no == "no_answer") {
                        amigoSpeak("I did not get a confirmation. I will start again asking about the action.");
                        no_answer = true;
                    }
                    else {
                        amigoSpeak("These are all the actions I am able to do.. Maybe I misunderstood you somewhere along the way. Let's start over again.");
                    }
                }
            }
        }

        /////////////////////////////////////////////////////////
        //       Step 3: Determine object/location(s)/..       //
        /////////////////////////////////////////////////////////

        // for actions transport and get:
        if (action_steps[0] == "transport" || action_steps[0] == "get") {
            // determine which object category amigo should transport or get
            answer_object_class = askUser("object_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

            // determine exact object to transport or get
            answer_object_exact = askUser(answer_object_class, 10, time_out - (ros::Time::now().toSec() - t_start));

            // check if object should be grabbed from a certain location
            amigoSpeak("Do you know where I should get the object from?");
            answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
            setColor(1,0,0);
            if (answer_yes_no == "yes") {
                while (answer_location_exact_from == "empty" && ros::ok()) {
                    // determine location category
                    answer_location_class = askUser("location_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

                    if (answer_location_class == "exit") {
                        amigoSpeak("I can not find an object that is on an exit, please respond with another location class");
                    }
                    else {
                        // determine exact object to transport or get
                        answer_location_exact_from = askUser(answer_location_class, 10, time_out - (ros::Time::now().toSec() - t_start));
                    }
                }
            }
            else if (answer_yes_no == "no") {
                amigoSpeak("I will try to find the object myself.");
            }

            // for action transport, determine dropoff location
            if (action_steps[0] == "transport") {
                // determine location category
                amigoSpeak("I wonder where I should bring the object to.");
                answer_location_class = askUser("location_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

                if (answer_location_class == "exit") {
                    answer_location_exact_to == "exit";
                }
                else {
                    // determine exact location to transport object to
                    answer_location_exact_to = askUser(answer_location_class, 10, time_out - (ros::Time::now().toSec() - t_start));
                }
            }
        }

        // for actions find and point
        if (action_steps[0] == "point" || action_steps[0] == "find") {

            // check if location or object should be found / pointed at.
            if (action_steps[0] == "point") {
                amigoSpeak("Should I point at an object or a location?");
            }
            else {
                amigoSpeak("Should I find an object or a location?");
            }
            answer_location_or_object = askUser("object_or_location", 10, time_out - (ros::Time::now().toSec() - t_start));
            if (answer_location_or_object == "location") {
                // Make sure action will be find (since find is implemented as going to location)
                action_steps[0] = "find";

                // determine location category
                answer_location_class = askUser("location_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

                if (answer_location_class == "exit") {
                    answer_location_exact_to == "exit";
                }
                else {
                    // determine exact location to point at
                    answer_location_exact_to = askUser(answer_location_class, 10, time_out - (ros::Time::now().toSec() - t_start));
                }
            }
            else if (answer_location_or_object == "object") {
                // Make sure action will be point (since point is implemented as finding an object)
                action_steps[0] = "point";

                // determine object category
                answer_object_class = askUser("object_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

                // determine exact object to transport or get
                answer_object_exact = askUser(answer_object_class, 10, time_out - (ros::Time::now().toSec() - t_start));

                // check if object should be grabbed from a certain location
                amigoSpeak("Do you already know where the object is located?");
                answer_yes_no = getYesNoFunc(confirmation, n_tries_yesno, time_out - (ros::Time::now().toSec() - t_start));
                setColor(1,0,0);
                if (answer_yes_no == "yes") {
                    while (answer_location_exact_from == "empty" && ros::ok()) {
                        // determine location category
                        answer_location_class = askUser("location_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

                        if (answer_location_class == "exit") {
                            amigoSpeak("I can not find an object that is on an exit, please respond with another location class");
                        }
                        else {
                            // determine exact location
                            answer_location_exact_from = askUser(answer_location_class, 10, time_out - (ros::Time::now().toSec() - t_start));
                        }
                    }
                }
                else if (answer_yes_no == "no") {
                    amigoSpeak("I will try to find the object myself.");
                }
            }
        }

        else if (action_steps[0] == "navigate") {

            amigoSpeak("Where do you want me to go to?");

            // determine location category
            answer_location_class = askUser("location_classes", 10, time_out - (ros::Time::now().toSec() - t_start));

            if (answer_location_class == "exit") {
                answer_location_exact_to == "exit";
            }
            else {
                // determine exact location to go to
                answer_location_exact_to = askUser(answer_location_class, 10, time_out - (ros::Time::now().toSec() - t_start));
            }
        }

        action_steps.push_back(answer_location_exact_to);
        action_steps.push_back(answer_object_exact);
        action_steps.push_back(answer_location_exact_from);


            //        if (action_steps[0] == "transport") {

            //            // determine which object category amigo should transport
            //            // determine object to transport
            //            // check from  (Do you know where I should get the object from?)
            //            // -> yes -> get location class from
            //            //        -> get exact location
            //            // determine location dropoff category
            //            // determine exact dropoff location

            //        }
            //        else if (action_steps[0] == "get") {

            //            // determine which object category amigo should get
            //            // determine object to get
            //            // check from  (Do you know where I should get the object from?)
            //            // -> yes -> get location class from
            //            //        -> get exact location

            //        }

            //        else if (action_steps[0] == "point") {  // point at location

            //            // check if i should point at a location or at an object.
            //            // if object
            //            //      change action to find
            //            //      get object class
            //            //      get exact object
            //            //      check from
            //            //      if yes
            //            //          get location class from
            //            //          get exact location
            //            // if location
            //            //      get location class
            //            //      get exact location


            //        }
            //        else if (action_steps[0] == "find") {  // point at object

            //            // check if i should find a location or an object.
            //            // if object
            //            //      get object class
            //            //      get exact object
            //            //      check from
            //            //      if yes
            //            //          get location class from
            //            //          get exact location
            //            // if location
            //            //      change action to find
            //            //      get location class
            //            //      get exact location

            //        }

            //            else if (action_steps[0] == "navigate") {

            //                // determine location category
            //                // determine exact location
            //            }


        break; // breaks while loop of time_out
    }
    return action_steps;
}

/**
 * Function that makes AMIGO speak
 */
void Interpreter::amigoSpeak(std::string txt) {
   if(ros::param::has("/text_to_speech"))
        {
            ROS_INFO("%s", txt.c_str());
            text_to_speech_philips::Speak speak;

            // Also send the text over a topic (for simulation purposes, ask Sjoerd)
            std_msgs::String msg_txt;
            msg_txt.data = txt;
            pub_amigo_speech_sim_.publish(msg_txt);

            speak.request.sentence = txt;
            speak.request.language = "us";
            speak.request.character = "kyle";
            speak.request.voice = "default";
            speak.request.emotion = "excited";
            speak.request.blocking_call = true;
            client_speech_.call(speak);
        }
    else
        {
        // TODO Should be connected to the text to speech module topic
        ROS_INFO("%s", txt.c_str());
        }
}

/**
 * Function that gets the line number of the spoken input in gpsr_without_spaces.txt
 */
int Interpreter::getLineNumber(std::string text_at_line, std::string category) {

    std::string path;
    if (category == "action") {
        path = ros::package::getPath("speech_interpreter") + "/include/gpsr_magdeburg2013_without_spaces.txt";
    }
    else if (category == "cleanup") {
        path = ros::package::getPath("speech_interpreter") + "/include/cleanup_magdeburg2013_without_spaces.txt";
    }
    else if (category == "open_challenge") {
        path = ros::package::getPath("speech_interpreter") + "/include/open_challenge_magdeburg2013_without_spaces.txt";
    }
    else if (category == "demo_challenge") {
        path = ros::package::getPath("speech_interpreter") + "/include/demo_challenge_magdeburg2013_without_spaces.txt";
    }

    ifstream myfile;
    myfile.open(path.c_str());
    std::string line;
    if(myfile.fail())
        {
            cerr<<"\nThe file could not be opened.\n";
            return -1;
        }

    int i = 0;

    if (myfile.is_open())
        {
            while (! myfile.eof() && ! (line == text_at_line) && ros::ok())
            {
                getline (myfile,line);
                i++;
            }
            myfile.close();
    }
    return i;
}

/**
 * Function that gets the line number of the spoken input in gpsr_without_spaces.txt
 */
std::string Interpreter::getTextWithSpaces(int number, std::string category) {

    std::string path;
    if (category == "action") {
        path = ros::package::getPath("speech_interpreter") + "/include/gpsr_magdeburg2013_with_spaces.txt";
    }
    else if (category == "cleanup") {
        path = ros::package::getPath("speech_interpreter") + "/include/cleanup_magdeburg2013_with_spaces.txt";
    }
    else if (category == "open_challenge") {
        path = ros::package::getPath("speech_interpreter") + "/include/open_challenge_magdeburg2013_with_spaces.txt";
    }
    else if (category == "demo_challenge") {
        path = ros::package::getPath("speech_interpreter") + "/include/demo_challenge_magdeburg2013_with_spaces.txt";
    }

    ifstream myfile;
    myfile.open(path.c_str());
    std::string line;
    if(myfile.fail())
        {
            cerr<<"\nThe file could not be opened.\n";
            line = "File could not be opened";
            return line;
        }
    int i = 0;

    if (myfile.is_open())
        {
            while (! myfile.eof() && ! (i == number) && ros::ok())
            {
                getline (myfile,line);
                i++;
            }
            myfile.close();
        }
    return line;
}

void Interpreter::setColor(int r, int g, int b){

    std_msgs::ColorRGBA rgba;
    rgba.r = r;
    rgba.g = g;
    rgba.b = b;
    rgba.a = 1;

    amigo_msgs::RGBLightCommand rgb_msg;

    rgb_msg.color = rgba;
    rgb_msg.show_color.data = true;
    set_rgb_lights_.publish(rgb_msg);
}

/**
 * Service that asks if user says continue
 */
bool Interpreter::getContinue(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer) {

    // Get variables
    const double t_max_question = std::max(max_duration.toSec(), max_duration.toSec() / (2.0 * max_num_tries));
    unsigned int n_tries = 0;

    ROS_INFO("I will try to receive continue %d tries and time out of %f", max_num_tries, max_duration.toSec());

    while (n_tries < max_num_tries && ros::ok()) {

        if (waitForAnswer("continue", t_max_question)) {
            setColor(1,0,0); // color red
            //amigoSpeak("I heard continue");
            answer["answer"] = "true";
            setColor(0,0,1); // color blue
            return true;
        } else {
            setColor(1,0,0); // color red
            amigoSpeak("I did not hear continue.");
            answer["answer"] = "false";
            n_tries++;
            if (!(n_tries == max_num_tries)){
                amigoSpeak("Let's try again");
            }
        }
    }
    setColor(0,0,1); // color blue

    return true;

}

std::string Interpreter::getSentence(std::vector<std::string> possible_text) {

    int max = possible_text.size();
    //ROS_INFO("max =  %i", max);
    int output;
    output = (max) * ((double)rand() / (double)RAND_MAX);
    //ROS_INFO("output =  %i", output);
    std::string sentence;
    sentence = possible_text[output];
    //ROS_INFO("chosen sentence =  %s", sentence.c_str());

    return sentence;
}

/**
 * Service that asks user yes or no
 */
bool Interpreter::getYesNo(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer) {

    // Get variables
    const double t_max_question = std::max(max_duration.toSec(), max_duration.toSec() / (2.0 * max_num_tries));
    unsigned int n_tries = 0;
    double t_start = ros::Time::now().toSec();

    ROS_INFO("I will try to receive yes no %d tries and time out of %f", max_num_tries, max_duration.toSec());

    while (n_tries < max_num_tries  && ros::ok()) {
        if (waitForAnswer("yesno", t_max_question)) {
			if (answer_ == "yes" || answer_ == "y") {
				setColor(1,0,0); // color red
                amigoSpeak("I heard yes, is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                while (n_tries < max_num_tries && ros::ok()) {
                    if (waitForAnswer("yesno", t_max_question)){
                        if (answer_ == "yes" || answer_ == "y") {
                            std::vector<std::string> possible_text;
                            possible_text.push_back("Certainly");
                            possible_text.push_back("Okay");
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);
                            answer["answer"] = "true";
                            setColor(0,0,1); // color blue
                            return true;
                        }
                        else if (!(answer_=="no")){
                            std::vector<std::string> possible_text;
                            possible_text.push_back("Could you please answer with yes or no?");
                            possible_text.push_back("Please answer with yes or no");
                            possible_text.push_back("Yes or no?");
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);
                            ++n_tries;
                        }
                        else{
                            std::vector<std::string> possible_text;
                            possible_text.push_back("So I will assume that you said no.");
                            possible_text.push_back("Then I guess you would like to answer the question with no.");
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);
                            answer["answer"] = "false";
                            setColor(0,0,1); // color blue
                            return true;
                        }
                    }
                    else {
                        setColor(1,0,0); // color red
                        answer["answer"] = "false"; //= no answer
                        std::vector<std::string> possible_text;
                        if ((n_tries + 1) == max_num_tries) {
                            possible_text.push_back("I did not hear you for a longer time.");
                            possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                            setColor(0,0,1); // color blue
                            return true;
                        }
                        else {
                            possible_text.push_back("I did not hear you, I'm sorry. Please say yes or no.");
                            possible_text.push_back("Did you say yes or no? In any case, I was not able to hear you.");
                            possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                        }
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);

                        ROS_DEBUG("n_tries before = %i", n_tries);
                        ++n_tries;
                        ROS_DEBUG("n_tries now = %i", n_tries);
                        ROS_DEBUG("n_tries_max = %i", max_num_tries);
                        double time_check = ros::Time::now().toSec() - t_start;
                        ROS_DEBUG("rostime: now - t_start = %f", time_check);
                        ROS_DEBUG("time_out = %f", max_duration.toSec());
                    }
                    setColor(0,0,1); // color blue
                }
                setColor(0,0,1); // color blue
				return true;
			}
			else {
				if (!(answer_ == "no")) {
                    std::vector<std::string> possible_text;
                    possible_text.push_back("Could you please answer with yes or no?");
                    possible_text.push_back("Please answer with yes or no");
                    possible_text.push_back("Yes or no?");
                    std::string sentence;
                    sentence = getSentence(possible_text);
                    amigoSpeak(sentence);
                    answer["answer"] = "false";
                    
                }
                else {
                    amigoSpeak("I heard no, is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                    while (n_tries < max_num_tries && ros::ok()) {
						if (waitForAnswer("yesno", t_max_question)){
							if (answer_ == "yes" || answer_ == "y") {
                                std::vector<std::string> possible_text;
                                possible_text.push_back("Certainly");
                                possible_text.push_back("Okay");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
                                answer["answer"] = "false";
                                setColor(0,0,1); // color blue
								return true;
							}
							else if (!(answer_=="no")){
                                std::vector<std::string> possible_text;
                                possible_text.push_back("Could you please answer with yes or no?");
                                possible_text.push_back("Please answer with yes or no");
                                possible_text.push_back("Yes or no?");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
                                ++n_tries;
							}
							else{
                                std::vector<std::string> possible_text;
                                possible_text.push_back("So I will assume that you said yes.");
                                possible_text.push_back("Then I guess you would like to answer the question with yes.");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
                                answer["answer"] = "true";
                                setColor(0,0,1); // color blue
                                return true;
							}	
                        }
                        else {
                            setColor(1,0,0); // color red
                            answer["answer"] = "false"; //= no answer
                            std::vector<std::string> possible_text;
                            if ((n_tries + 1) == max_num_tries) {
                                possible_text.push_back("I did not hear you for a longer time.");
                                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                setColor(0,0,1); // color blue
                                return true;
                            }
                            else {
                                possible_text.push_back("I did not hear you, I'm sorry. Please say yes or no.");
                                possible_text.push_back("Did you say yes or no? In any case, I was not able to hear you.");
                                possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                            }
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);

                            ROS_DEBUG("n_tries before = %i", n_tries);
                            ++n_tries;
                            ROS_DEBUG("n_tries now = %i", n_tries);
                            ROS_DEBUG("n_tries_max = %i", max_num_tries);
                            double time_check = ros::Time::now().toSec() - t_start;
                            ROS_DEBUG("rostime: now - t_start = %f", time_check);
                            ROS_DEBUG("time_out = %f", max_duration.toSec());
                        }
                        setColor(0,0,1); // color blue
					}
				}
			}
			
		}
        else {
            setColor(1,0,0); // color red
            answer["answer"] = "false"; //= no answer
            std::vector<std::string> possible_text;
            if ((n_tries + 1) == max_num_tries) {
                possible_text.push_back("I did not hear you for a longer time.");
                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                setColor(0,0,1); // color blue
                return true;
            }
            else {
                possible_text.push_back("I did not hear you, I'm sorry. Please say yes or no.");
                possible_text.push_back("Did you say yes or no? In any case, I was not able to hear you.");
                possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
            }
            std::string sentence;
            sentence = getSentence(possible_text);
            amigoSpeak(sentence);

            ROS_DEBUG("n_tries before = %i", n_tries);
            ++n_tries;
            ROS_DEBUG("n_tries now = %i", n_tries);
            ROS_DEBUG("n_tries_max = %i", max_num_tries);
            double time_check = ros::Time::now().toSec() - t_start;
            ROS_DEBUG("rostime: now - t_start = %f", time_check);
            ROS_DEBUG("time_out = %f", max_duration.toSec());
        }
		setColor(0,0,1); // color blue
	}
    setColor(0,0,1); // color blue
	return true;
}

/**
 * Function that asks user yes or no and has extra input if there should be a confirmation on yes or no.
 */
std::string Interpreter::getYesNoFunc(bool confirmation, const double n_tries_max, const double time_out) {

    // Get variables
    const double t_max_question = 10; // std::max(time_out, time_out/(2.0*n_tries_max));
    unsigned int n_tries = 0;
    double t_start = ros::Time::now().toSec();
    std::string answer = "no_answer";

    ROS_INFO("I will try to receive yes no %d tries and time out of %f", n_tries_max, time_out);

    while (n_tries < n_tries_max  && ros::ok()) {
        if (waitForAnswer("yesno", t_max_question)) {
            if (answer_ == "yes" || answer_ == "y") {
                if (confirmation) {
                    setColor(1,0,0); // color red
                    amigoSpeak("I heard yes, is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                    while (n_tries < n_tries_max && ros::ok()) {
                        if (waitForAnswer("yesno", t_max_question)){
                            if (answer_ == "yes" || answer_ == "y") {
                                std::vector<std::string> possible_text;
                                possible_text.push_back("Certainly");
                                possible_text.push_back("Okay");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
                                answer = "yes";
                                setColor(0,0,1); // color blue
                                return answer;
                            }
                            else if (!(answer_=="no")){
                                std::vector<std::string> possible_text;
                                possible_text.push_back("Could you please answer with yes or no?");
                                possible_text.push_back("Please answer with yes or no");
                                possible_text.push_back("Yes or no?");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
                                ++n_tries;
                            }
                            else{
                                std::vector<std::string> possible_text;
                                possible_text.push_back("So I will assume that you said no.");
                                possible_text.push_back("Then I guess you would like to answer the question with no.");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
                                answer = "no";
                                setColor(0,0,1); // color blue
                                return answer;
                            }
                        }
                        else {
                            setColor(1,0,0); // color red
                            answer = "no_answer"; //= no answer
                            std::vector<std::string> possible_text;
                            if ((n_tries + 1) == n_tries_max) {
                                possible_text.push_back("I did not hear you for a longer time.");
                                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                setColor(0,0,1); // color blue
                                return answer;
                            }
                            else {
                                possible_text.push_back("I did not hear you, I'm sorry. Please say yes or no.");
                                possible_text.push_back("Did you say yes or no? In any case, I was not able to hear you.");
                                possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                            }
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);

                            ROS_DEBUG("n_tries before = %i", n_tries);
                            ++n_tries;
                            ROS_DEBUG("n_tries now = %i", n_tries);
                            ROS_DEBUG("n_tries_max = %i", n_tries_max);
                            double time_check = ros::Time::now().toSec() - t_start;
                            ROS_DEBUG("rostime: now - t_start = %f", time_check);
                            ROS_DEBUG("time_out = %f", time_out);
                        }
                        setColor(0,0,1); // color blue
                    }
                }
                else
                {
                    std::vector<std::string> possible_text;
                    possible_text.push_back("I heard yes.");
                    std::string sentence;
                    sentence = getSentence(possible_text);
                    amigoSpeak(sentence);
                    answer = "yes";
                    setColor(0,0,1); // color blue
                    return answer;
                }
            }
            else {
                if (!(answer_ == "no")) {
                    std::vector<std::string> possible_text;
                    possible_text.push_back("Could you please answer with yes or no?");
                    possible_text.push_back("Please answer with yes or no");
                    possible_text.push_back("Yes or no?");
                    std::string sentence;
                    sentence = getSentence(possible_text);
                    amigoSpeak(sentence);
                    answer = "no_answer";

                }
                else {
                    if (confirmation) {
                        amigoSpeak("I heard no, is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                        while (n_tries < n_tries_max && ros::ok()) {
                            if (waitForAnswer("yesno", t_max_question)){
                                if (answer_ == "yes" || answer_ == "y") {
                                    std::vector<std::string> possible_text;
                                    possible_text.push_back("Certainly");
                                    possible_text.push_back("Okay");
                                    std::string sentence;
                                    sentence = getSentence(possible_text);
                                    amigoSpeak(sentence);
                                    answer = "no";
                                    setColor(0,0,1); // color blue
                                    return answer;
                                }
                                else if (!(answer_=="no")){
                                    std::vector<std::string> possible_text;
                                    possible_text.push_back("Could you please answer with yes or no?");
                                    possible_text.push_back("Please answer with yes or no");
                                    possible_text.push_back("Yes or no?");
                                    std::string sentence;
                                    sentence = getSentence(possible_text);
                                    amigoSpeak(sentence);
                                    ++n_tries;
                                }
                                else{
                                    std::vector<std::string> possible_text;
                                    possible_text.push_back("So I will assume that you said yes.");
                                    possible_text.push_back("Then I guess you would like to answer the question with yes.");
                                    std::string sentence;
                                    sentence = getSentence(possible_text);
                                    amigoSpeak(sentence);
                                    answer = "yes";
                                    setColor(0,0,1); // color blue
                                    return answer;
                                }
                            }
                            else {
                                setColor(1,0,0); // color red
                                answer = "no_answer"; //= no answer
                                std::vector<std::string> possible_text;
                                if ((n_tries + 1) == n_tries_max) {
                                    possible_text.push_back("I did not hear you for a longer time.");
                                    possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                                    setColor(0,0,1); // color blue
                                    return answer;
                                }
                                else {
                                    possible_text.push_back("I did not hear you, I'm sorry. Please say yes or no.");
                                    possible_text.push_back("Did you say yes or no? In any case, I was not able to hear you.");
                                    possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
                                }
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);

                                ROS_DEBUG("n_tries before = %i", n_tries);
                                ++n_tries;
                                ROS_DEBUG("n_tries now = %i", n_tries);
                                ROS_DEBUG("n_tries_max = %i", n_tries_max);
                                double time_check = ros::Time::now().toSec() - t_start;
                                ROS_DEBUG("rostime: now - t_start = %f", time_check);
                                ROS_DEBUG("time_out = %f", time_out);
                            }
                            setColor(0,0,1); // color blue
                        }
                    }
                    else {
                        std::vector<std::string> possible_text;
                        possible_text.push_back("I heard no.");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                        answer = "no";
                        setColor(0,0,1); // color blue
                        return answer;
                    }

                }
            }

        }
        else {
            setColor(1,0,0); // color red
            answer = "no_answer"; //= no answer
            std::vector<std::string> possible_text;
            if ((n_tries + 1) == n_tries_max) {
                possible_text.push_back("I did not hear you for a longer time.");
                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
                setColor(0,0,1); // color blue
                return answer;
            }
            else {
                possible_text.push_back("I did not hear you, I'm sorry. Please say yes or no.");
                possible_text.push_back("Did you say yes or no? In any case, I was not able to hear you.");
                possible_text.push_back("I did not hear you, maybe you should get a little closer to me.");
            }
            std::string sentence;
            sentence = getSentence(possible_text);
            amigoSpeak(sentence);

            ROS_DEBUG("n_tries before = %i", n_tries);
            ++n_tries;
            ROS_DEBUG("n_tries now = %i", n_tries);
            ROS_DEBUG("n_tries_max = %i", n_tries_max);
            double time_check = ros::Time::now().toSec() - t_start;
            ROS_DEBUG("rostime: now - t_start = %f", time_check);
            ROS_DEBUG("time_out = %f", time_out);
        }
        setColor(0,0,1); // color blue
    }
    setColor(0,0,1); // color blue
    return answer;
}

/**
 * Service that asks user which room to clean
 */
bool Interpreter::getCleanup(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer) {

    // Initial response is to clean up the livingroom
    answer["answer"] = "cleanupthelivingroom";

    ROS_INFO("I will try to get the specific room to cleanup %d tries and time out of %f", max_num_tries, max_duration.toSec());

    double t_start = ros::Time::now().toSec();

    if (iExplainedLights == false) {
        // Explain lights during questioning:
        // Red: Amigo talks
        // Green: Questioner talks
        setColor(1,0,0); // color red
        std::string explaining_txt = "Before I ask you what I can do for you, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
        amigoSpeak(explaining_txt);

        iExplainedLights = true;
    }

    // Get the room
    std::string cleanuproom = askUser("cleanup", max_num_tries, max_duration.toSec());
    ROS_DEBUG("Received action %s, %f seconds left for refining action", cleanuproom.c_str(), ros::Time::now().toSec() - t_start);

    if (cleanuproom == "no_answer" || cleanuproom == "wrong_answer") {
        amigoSpeak("But I will just start to clean up the livingroom, I guess that that room was on your todo list.");
    }
    else {
        answer["answer"] = cleanuproom;
    }

    setColor(0,0,1); // color blue
    return true;
}

/**
 * Service that asks to which location to go, specific for open challenge magdeburg 2013
 */
bool Interpreter::getOpenChallenge(const ros::Duration& max_duration, unsigned int max_num_tries, std::map<std::string, std::string>& answer) {

    // Initial response is to clean up the livingroom
    answer["answer"] = "no_answer";

    ROS_INFO("I will try to know to which location I need to go in %d tries and time out of %f", max_num_tries, max_duration.toSec());

    double t_start = ros::Time::now().toSec();

    if (iExplainedLights == false) {
        // Explain lights during questioning:
        // Red: Amigo talks
        // Green: Questioner talks
        setColor(1,0,0); // color red
        std::string explaining_txt = "Before I ask you where I should go, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
        amigoSpeak(explaining_txt);

        iExplainedLights = true;
    }

    // Get the room
    std::string open_challenge = askUser("open_challenge", max_num_tries, max_duration.toSec());
    ROS_DEBUG("Received action %s, %f seconds left for refining action", open_challenge.c_str(), ros::Time::now().toSec() - t_start);

    /*
    if (open_challenge == "no_answer" || open_challenge == "wrong_answer") {
        amigoSpeak("But I will just start to clean up the livingroom, I guess that that room was on your todo list.");
    }
    else {
        answer["answer"] = cleanuproom;
    }
    */
    answer["answer"] = open_challenge;
    setColor(0,0,1); // color blue
    return true;
}

int Interpreter::getPosString(std::string input_text, std::string found_text) {

    int pos = -1;
    while(true && ros::ok()) {
        pos =  input_text.find(found_text, ++pos);
        if (pos != std::string::npos) {
            ROS_DEBUG("Found word '%s' at position = %i", found_text.c_str(), pos);
            return pos;
        } else break;
     }
    ROS_DEBUG("Did not found word '%s' in = %s", found_text.c_str(), input_text.c_str());
    return pos;
}

}
