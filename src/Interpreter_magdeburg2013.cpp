/*
 * Interpreter.cpp
 *
 *  Created on: Nov 20, 2012
 *      Authors: Jos Elfring
 *               Erik Geerts
 */

#include "Interpreter.h"

#include "std_srvs/Empty.h"

#include <text_to_speech_philips/amigo_speakup_advanced.h>

#include <iostream>
#include <fstream>
using std::ifstream;
using std::cerr;
using std::cout;
#include <ros/package.h>

#include <std_msgs/ColorRGBA.h>
#include <amigo_msgs/RGBLightCommand.h>

#include <vector>

ros::ServiceClient pub_speech_;
//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

namespace SpeechInterpreter {

Interpreter::Interpreter() : answer_("") {

	// Nodehandle
	ros::NodeHandle nh("~");

	// Initialize mappings
	initializeMappings(nh);

	// Initialize speech recognition services
	initializeSpeechServicesTopics(nh);

	// Offer services
	info_service_ = nh.advertiseService("get_info_user", &Interpreter::getInfo, this);
	action_service_ = nh.advertiseService("get_action_user", &Interpreter::getAction, this);
    continue_service_ = nh.advertiseService("get_continue", &Interpreter::getContinue, this);
    yes_no_service_ = nh.advertiseService("get_yes_no", &Interpreter::getYesNo, this);
    cleanup_service_ = nh.advertiseService("get_cleanup", &Interpreter::getCleanup, this);
    open_challenge_service_ = nh.advertiseService("get_open_challenge", &Interpreter::getOpenChallenge, this);

    pub_speech_ =  nh.serviceClient<text_to_speech_philips::amigo_speakup_advanced>("/amigo_speakup_advanced");

    set_rgb_lights_ = nh.advertise<amigo_msgs::RGBLightCommand>("/user_set_rgb_lights", 100);

    // Initialize explained lights for get_action
    iExplainedLights = false;
}

Interpreter::~Interpreter() {
	// TODO Auto-generated destructor stub
}


void Interpreter::initializeMappings(ros::NodeHandle& nh) {

	// TODO: load from file or receive from reasoner

	// Actions synonyms
	action_map_["bring"] = "transport";
	action_map_["carry"] = "transport";
	action_map_["get"] = "get";
	action_map_["give"] = "get";
	action_map_["point"] = "point";
	action_map_["detect"] = "find";
	action_map_["find"] = "find";
    action_map_["go"] = "leave";
	action_map_["move"] = "leave";
	action_map_["navigate"] = "leave";

	// Categories
    category_map_["object_category"].push_back(std::make_pair("drink", 7));
    category_map_["object_category"].push_back(std::make_pair("stuff", 10));
    category_map_["object_category"].push_back(std::make_pair("medicine", 3));
    category_map_["object_category"].push_back(std::make_pair("food", 5));
    category_map_["location_category"].push_back(std::make_pair("seat", 7));
	category_map_["location_category"].push_back(std::make_pair("table", 2));
    category_map_["location_category"].push_back(std::make_pair("storage", 2));
    category_map_["location_category"].push_back(std::make_pair("decoration", 2));
    category_map_["location_category"].push_back(std::make_pair("appliance", 4));

    category_map_["room_category"].push_back(std::make_pair("room", 2));
    category_map_["room_location_category"].push_back(std::make_pair("diningroom", 3));
    category_map_["room_location_category"].push_back(std::make_pair("livingroom", 5));
    category_map_["room_location_category"].push_back(std::make_pair("kitchen", 9));

	// Action category map
	action_category_map_["transport"].push_back("object_category");
	action_category_map_["transport"].push_back("location_category");
	action_category_map_["get"].push_back("object_category");
	action_category_map_["find"].push_back("object_category");
    action_category_map_["point"].push_back("location_category");
}

void Interpreter::initializeSpeechServicesTopics(ros::NodeHandle& nh) {

	// Empty message used to turn off all services
	std_srvs::Empty srv;

	// Add start and stop services for all objects and locations
	for (CategoryMap::const_iterator it1 = category_map_.begin(); it1 != category_map_.end(); ++it1) {
		for (std::vector<std::pair<std::string, int> >::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {

			// Only create start/stop services if category has more than one member
			if (it2->second > 1) {
				//ROS_INFO("Creating start and stop services for category %s", it2->first.c_str());
				category_srv_clients_map_[it2->first] =
						std::make_pair(nh.serviceClient<std_srvs::Empty>("/speech_" + it2->first + "/start"),
								nh.serviceClient<std_srvs::Empty>("/speech_" + it2->first + "/stop"));
				category_srv_clients_map_[it2->first].second.waitForExistence(ros::Duration(1.0));
				if (!category_srv_clients_map_[it2->first].second.call(srv)) {
					ROS_WARN("Unable to turn off speech recognition for %s", it2->first.c_str());
				} else ROS_INFO("Switched off speech recognition for %s", it2->first.c_str());

				// Create a listener
				category_sub_map_[it2->first] = nh.subscribe("/speech_" + it2->first + "/output", 10, &Interpreter::speechCallback, this);

			} else ROS_INFO("No start and stop services needed for category %s", it2->first.c_str());
		}
	}


	// Always add start and stop services for names and yes/no
	std::vector<std::string> fixed_list;
	fixed_list.push_back("yesno");     // TODO this should be loaded from yaml?
	fixed_list.push_back("name");      // TODO this should be loaded from yaml?
	fixed_list.push_back("sentences"); // TODO this should be loaded from yaml?
    fixed_list.push_back("continue"); // TODO this should be loaded from yaml?
    fixed_list.push_back("cleanup"); // TODO this should be loaded from yaml?
    fixed_list.push_back("open_challenge"); // TODO this should be loaded from yaml?
    fixed_list.push_back("drink_cocktail"); // TODO this should be loaded from yaml?

	for (std::vector<std::string>::const_iterator it = fixed_list.begin(); it != fixed_list.end(); ++it) {
		//ROS_INFO("Creating start and stop services for %s", it->c_str());
		category_srv_clients_map_[(*it)] =
				std::make_pair(nh.serviceClient<std_srvs::Empty>("/speech_" + (*it) + "/start"),
						nh.serviceClient<std_srvs::Empty>("/speech_" + (*it) + "/stop"));
		category_srv_clients_map_[(*it)].second.waitForExistence(ros::Duration(1.0));
		if (!category_srv_clients_map_[(*it)].second.call(srv)) {
			ROS_WARN("Unable to turn off speech recognition for %s",  it->c_str());
		} else ROS_INFO("Switched off speech recognition for %s", it->c_str());

		// Create a listener
		category_sub_map_[(*it)] = nh.subscribe("/speech_" + (*it) + "/output", 10, &Interpreter::speechCallback, this);
	}

}


/**
 * Service that asks info from user
 */
bool Interpreter::getInfo(speech_interpreter::GetInfo::Request  &req, speech_interpreter::GetInfo::Response &res) {

	// Get variables
    std::string type = req.type.data();
    unsigned int n_tries = req.n_tries;
    double time_out = req.time_out;

	// Convert to lower case
	std::transform(type.begin(), type.end(), type.begin(), ::tolower);
	std::string response = "";

	// Validate input
	if (type == "name") {
		ROS_INFO("I will get you a name, %d tries and time out of %f", n_tries, time_out);

    }
    else if (type == "drink_cocktail") {
        ROS_INFO("I will get you a drink in %d tries and time out of %f", n_tries, time_out);

    }
    else {

		// Iterate over categories and determine if it is present
		bool return_value = false;
		for (CategoryMap::const_iterator itc = category_map_.begin(); itc != category_map_.end(); ++itc) {
			// Iterate over vector of pairs
			for (std::vector<std::pair<std::string, int> >::const_iterator itp = itc->second.begin(); itp != itc->second.end(); ++itp) {
				// Check if the requested type equals the current category
				if (itp->first == type) {
					ROS_INFO("I will get you a %s, %d tries and time out of %f", type.c_str(), n_tries, time_out);
					return_value = true;
				}
			}
		}
        if (iExplainedLights == false) {
            // Explain lights during questioning:
            // Red: Amigo talks
            // Green: Questioner talks
            setColor(1,0,0); // color red
            std::string explaining_txt = "Before I ask you where I should go, I just want to tell you that if my lights are red during questioning, I will do the word and when my lights are green during questioning, you can talk.";
            amigoSpeak(explaining_txt);

            iExplainedLights = true;
        }
		// Feedback in case of invalid request
		if (!return_value) {
			ROS_WARN("Speech interpreter received request for %s, which is unknown", type.c_str());
			return false;
		}
	}

    response = askUser(type, n_tries, time_out);
	ROS_INFO("Answer is %s", response.c_str());

	res.answer = response;


	return (!response.empty());
}


/**
 * Function that gets an action from a user and asks for missing information
 */
bool Interpreter::getAction(speech_interpreter::GetAction::Request  &req, speech_interpreter::GetAction::Response &res) {

	// Initial response should be empty
	res.action = "empty";
	res.start_location = "meeting_point";
	res.end_location = "meeting_point";
	res.object = "empty";
    res.object_room = "empty";
    res.object_location = "empty";

	// Get the action from the user
	double time_out_action = req.time_out;
	ROS_INFO("I will get you an action within %f seconds", time_out_action);
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
    int max_num_tries_action = 1e6;
    std::string action = askUser("action", max_num_tries_action, time_out_action);
    ROS_DEBUG("Received action %s, %f seconds left for refining action", action.c_str(), ros::Time::now().toSec() - t_start);

    // Check action type that is requested
	for (std::map<std::string, std::string>::const_iterator it = action_map_.begin(); it != action_map_.end(); ++it) {
		if (action.find(it->first) != std::string::npos) {
            ROS_DEBUG("Action includes %s which is a %s action", it->first.c_str(), it->second.c_str());
			res.action = it->second;
			break;
		}
	}

	// Check if action is known
	if (res.action.empty()) {
        ROS_ERROR("Code error: Action %s received over speech_sentences/output topic does not contain a known action", action.c_str());
		return false;
	}

    // All actions in action_map_ must be defined in action_category_map_ as well
	std::map<std::string, std::vector<std::string> >::const_iterator it_action = action_category_map_.find(res.action);
	if (it_action == action_category_map_.end()) {
		ROS_DEBUG("All information is available, no need to ask questions");
        amigoSpeak("I know everything I need to know, I will go to an exit now");
		res.end_location = "exit";
		res.action = "leave";
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
                    response = askUser(it_cat->first, 5, time_out_action - (ros::Time::now().toSec() - t_start));

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

                // If category is object, location is queried. First which room -> if yes -> which specific location?
                unsigned int n_tries = 0;
                bool heard_one_answer = false;
                bool object_room_known = false;
                std::string response_object_room = "empty";
                if ((*it) == "object_category") {
                	if (response == "no_answer") {
                		res.object_room = "room_not_known";
                        res.object_location = "location_not_known";

                        std::vector<std::string> possible_text;
                        possible_text.push_back("We should start all over.");
                        possible_text.push_back("Let's start at the beginning");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                        ++n_tries;

                   	}
                    else if (response == "wrong_answer") {
                        res.object_room = "room_not_known";
                        res.object_location = "location_not_known";

                        std::vector<std::string> possible_text;
                        possible_text.push_back("We should start all over.");
                        possible_text.push_back("Let's start at the beginning");
                        std::string sentence;
                        sentence = getSentence(possible_text);
                        amigoSpeak(sentence);
                        ++n_tries;
                    }
                	else {
	                    while (ros::Time::now().toSec() - t_start < (time_out_action - (ros::Time::now().toSec() - t_start))) {

	                        // Check if the response starts with a vowel
	                        std::string vowels = "auioe";
	                        bool start_with_vowel = (vowels.find(response[0])< vowels.length());

	                        std::string art = (start_with_vowel)?"an ":"a ";
	                        std::string starting_txt = "Are you aware of the room in which I can find " + art + response;
	                        amigoSpeak(starting_txt);

	                        while (n_tries < 5) {
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
	                                    response_object_room = askUser("room", 5, time_out_action - (ros::Time::now().toSec() - t_start));
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
	                            res.object_room = "room_not_known";
	                            res.object_location = "location_not_known";
	                            break;
	                        }
	                        else {
	                            res.object_room = response_object_room;
	                            std::string starting_txt = "Since you know which room, are you also aware of the specific location to find " + art + response;
	                            amigoSpeak(starting_txt);
	                            n_tries = 0;
	                            bool object_location_known = false;
	                            std::string response_object_location = "empty";

	                            while (n_tries < 5) {
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
	                                        response_object_location = askUser(response_object_room, 5, time_out_action - (ros::Time::now().toSec() - t_start));
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
	                                res.object_location = "location_not_known";
	                                break;
	                            }
	                            else {
	                                res.object_location = response_object_location;
	                                break;
	                            }
	                        }
	                    } // end while loop
	                }
                }
				break;
			}
		}

		// Response from user
		if ((*it) == "location_category") {
			res.end_location = response;
		} else if ((*it) == "object_category") {
			res.object = response;
		}

	}

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
		ROS_DEBUG("Received response: %s", res.data.c_str());
	}
}

/**
 * Function that switches on the requested speech recognition module and waits for an answer over the appropriate topic
 */
bool Interpreter::waitForAnswer(std::string category, double t_max) {

	if (category_srv_clients_map_.find(category) == category_srv_clients_map_.end()) {
		ROS_WARN("Request to listen to category %s, which is unknown", category.c_str());
		return false;
	}

	// Empty message used to turn off all services
	std_srvs::Empty srv;

	// Turn on speech recognition for requested category
	if (!category_srv_clients_map_[category].first.call(srv)) {
		ROS_WARN("Unable to turn on speech recognition for %s", category.c_str());
    }
    else {
        ROS_INFO("Switched on speech recognition for: %s", category.c_str());
        setColor(0,1,0); // color green
    }

	answer_.clear();
	double start_time = ros::Time::now().toSec();
	ros::Rate r(10);
    while (answer_.empty() || (answer_== " ")|| (answer_== "  ") || (answer_== "   ")) {
		if (ros::Time::now().toSec() - start_time > t_max) {
			ROS_WARN("Timeout: No input over speech topic %s for %f seconds", category.c_str(), t_max);
			break;
		}
		ros::spinOnce();
		r.sleep();
	}

    // If input is more than one word -> only pick the first word:
    if (answer_.size() > 2) {
        unsigned found=answer_.find(" ");
        std::string part = answer_.substr(0,found);
        answer_= part;
    }

    // Turn off speech recognition for requested category
	if (!category_srv_clients_map_[category].second.call(srv)) {
		ROS_WARN("Unable to turn off speech recognition for %s", category.c_str());
	} ROS_DEBUG("Switched off speech recognition for: %s", category.c_str());

	return (!answer_.empty());
}


/**
 * Function that does the actual human robot interaction, i.e., ask the right questions
 */
std::string Interpreter::askUser(std::string type, const unsigned int n_tries_max, const double time_out) {

	// Initialize
	double t_start = ros::Time::now().toSec();
	unsigned int n_tries = 0;

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

    } else {
		std::string art = (start_with_vowel)?"an ":"a ";
        starting_txt = "Can you specify which " + type + " you mean?";
    }    

    // Ask
    amigoSpeak(starting_txt);
    ROS_DEBUG("Max time to wait for answer = %f", t_max_question);
    while (ros::Time::now().toSec() - t_start < time_out && n_tries < n_tries_max) {

        // If an answer was heared, verify
        if (waitForAnswer(type, t_max_question)) {
            setColor(1,0,0); // color red
            result = answer_;
            int line_number;
            std::string result2;

            if ( type == "sentences") {
                line_number = getLineNumber(answer_, "action");
                result2 = getTextWithSpaces(line_number, "action");

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
            else if ( type == "cleanup") {
                line_number = getLineNumber(answer_, "cleanup");
                result2 = getTextWithSpaces(line_number, "cleanup");

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
                    result = "wrong_answer";
                    std::vector<std::string> possible_text;
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
                    ++n_tries;
				}
			}

			// If no answer heard to confirmation question, ask for confirmation again
			else {
                setColor(1,0,0); // color red
                if ( type == "sentences" || type == "cleanup" || type == "open_challenge") {
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

                // Check if the answer is confirmed after the second confirmation question
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
						result = "wrong_answer";
                        std::vector<std::string> possible_text;
                        if ((n_tries + 1) == n_tries_max) {
                            possible_text.push_back("I was not able to understand what you were saying for a longer time.");
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
						++n_tries;
					}
				}

				// Second confirmation question again did not lead to a yes or no, start all over again
				else {
                    setColor(1,0,0); // color red
                    result = "no_answer";
                    if (type == "sentences" || type == "cleanup" || type == "open_challenge") {
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
 * Function that makes AMIGO speak
 */
void Interpreter::amigoSpeak(std::string txt) {
   if(ros::param::has("/text_to_speech"))
        {
            ROS_INFO("%s", txt.c_str());
            text_to_speech_philips::amigo_speakup_advanced speak;

            speak.request.sentence = txt;
            speak.request.language = "us";
            speak.request.character = "kyle";
            speak.request.voice = "default";
            speak.request.emotion = "excited";
            pub_speech_.call(speak);
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
            while (! myfile.eof() && ! (line == text_at_line))
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
            while (! myfile.eof() && ! (i == number))
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
bool Interpreter::getContinue(speech_interpreter::GetContinue::Request  &req, speech_interpreter::GetContinue::Response &res) {

    // Get variables
    unsigned int n_tries_max = req.n_tries_max;
    double time_out = req.time_out;
    const double t_max_question = std::max(time_out, time_out/(2.0*n_tries_max));
    unsigned int n_tries = 0;

    ROS_INFO("I will try to receive continue %d tries and time out of %f", n_tries_max, time_out);

    while (n_tries < n_tries_max) {

        if (waitForAnswer("continue", t_max_question)) {
            setColor(1,0,0); // color red
            //amigoSpeak("I heard continue");
            res.answer = "true";
            setColor(0,0,1); // color blue
            return true;
        } else {
            setColor(1,0,0); // color red
            amigoSpeak("I did not hear continue.");
            res.answer = "false";
            n_tries++;
            if (!(n_tries == n_tries_max)){
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
bool Interpreter::getYesNo(speech_interpreter::GetYesNo::Request  &req, speech_interpreter::GetYesNo::Response &res) {

    // Get variables
    unsigned int n_tries_max = req.n_tries_max;
    double time_out = req.time_out;
    const double t_max_question = std::max(time_out, time_out/(2.0*n_tries_max));
    unsigned int n_tries = 0;
    double t_start = ros::Time::now().toSec();

    ROS_INFO("I will try to receive yes no %d tries and time out of %f", n_tries_max, time_out);

    while (n_tries < n_tries_max) {
        if (waitForAnswer("yesno", t_max_question)) {
			if (answer_ == "yes" || answer_ == "y") {
				setColor(1,0,0); // color red
                amigoSpeak("I heard yes, is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                while (n_tries < n_tries_max) {
                    if (waitForAnswer("yesno", t_max_question)){
                        if (answer_ == "yes" || answer_ == "y") {
                            std::vector<std::string> possible_text;
                            possible_text.push_back("Certainly");
                            possible_text.push_back("Okay");
                            std::string sentence;
                            sentence = getSentence(possible_text);
                            amigoSpeak(sentence);
                            res.answer = "true";
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
                            res.answer = "false";
                            return true;
                        }
                    }
                    else {
                        setColor(1,0,0); // color red
                        res.answer = "false"; //= no answer
                        std::vector<std::string> possible_text;
                        if ((n_tries + 1) == n_tries_max) {
                            possible_text.push_back("I did not hear you for a longer time.");
                            possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
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
                        ROS_DEBUG("n_tries_max = %i", n_tries_max);
                        double time_check = ros::Time::now().toSec() - t_start;
                        ROS_DEBUG("rostime: now - t_start = %f", time_check);
                        ROS_DEBUG("time_out = %f", time_out);
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
					res.answer = "false";
                    
                }
                else {
                    amigoSpeak("I heard no, is that corect?"); //Amigo's output with "corect?" is better than "correct?"
                    while (n_tries < n_tries_max) {
						if (waitForAnswer("yesno", t_max_question)){
							if (answer_ == "yes" || answer_ == "y") {
                                std::vector<std::string> possible_text;
                                possible_text.push_back("Certainly");
                                possible_text.push_back("Okay");
                                std::string sentence;
                                sentence = getSentence(possible_text);
                                amigoSpeak(sentence);
								res.answer = "false";
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
                                res.answer = "true";
                                return true;
							}	
                        }
                        else {
                            setColor(1,0,0); // color red
                            res.answer = "false"; //= no answer
                            std::vector<std::string> possible_text;
                            if ((n_tries + 1) == n_tries_max) {
                                possible_text.push_back("I did not hear you for a longer time.");
                                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
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
                            ROS_DEBUG("n_tries_max = %i", n_tries_max);
                            double time_check = ros::Time::now().toSec() - t_start;
                            ROS_DEBUG("rostime: now - t_start = %f", time_check);
                            ROS_DEBUG("time_out = %f", time_out);
                        }
                        setColor(0,0,1); // color blue
					}
				}
			}
			
		}
        else {
            setColor(1,0,0); // color red
            res.answer = "false"; //= no answer
            std::vector<std::string> possible_text;
            if ((n_tries + 1) == n_tries_max) {
                possible_text.push_back("I did not hear you for a longer time.");
                possible_text.push_back("I'm sorry, I did not hear from you for a longer time.");
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
            ROS_DEBUG("n_tries_max = %i", n_tries_max);
            double time_check = ros::Time::now().toSec() - t_start;
            ROS_DEBUG("rostime: now - t_start = %f", time_check);
            ROS_DEBUG("time_out = %f", time_out);
        }
		setColor(0,0,1); // color blue
	}
	return true;
}

/**
 * Service that asks user which room to clean
 */
bool Interpreter::getCleanup(speech_interpreter::GetCleanup::Request  &req, speech_interpreter::GetCleanup::Response &res) {

    // Get variables
    unsigned int n_tries_max = req.n_tries_max;
    double time_out = req.time_out;

    // Initial response is to clean up the livingroom
    res.answer = "cleanupthelivingroom";

    ROS_INFO("I will try to get the specific room to cleanup %d tries and time out of %f", n_tries_max, time_out);

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
    std::string cleanuproom = askUser("cleanup", n_tries_max, time_out);
    ROS_DEBUG("Received action %s, %f seconds left for refining action", cleanuproom.c_str(), ros::Time::now().toSec() - t_start);

    if (cleanuproom == "no_answer" || cleanuproom == "wrong_answer") {
        amigoSpeak("But I will just start to clean up the livingroom, I guess that that room was on your todo list.");
    }
    else {
        res.answer = cleanuproom;
    }


    return true;
}

/**
 * Service that asks to which location to go, specific for open challenge magdeburg 2013
 */
bool Interpreter::getOpenChallenge(speech_interpreter::GetOpenChallenge::Request  &req, speech_interpreter::GetOpenChallenge::Response &res) {

    // Get variables
    unsigned int n_tries_max = req.n_tries_max;
    double time_out = req.time_out;

    // Initial response is to clean up the livingroom
    res.answer = "no_answer";

    ROS_INFO("I will try to know to which location I need to go in %d tries and time out of %f", n_tries_max, time_out);

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
    std::string open_challenge = askUser("open_challenge", n_tries_max, time_out);
    ROS_DEBUG("Received action %s, %f seconds left for refining action", open_challenge.c_str(), ros::Time::now().toSec() - t_start);

    /*
    if (open_challenge == "no_answer" || open_challenge == "wrong_answer") {
        amigoSpeak("But I will just start to clean up the livingroom, I guess that that room was on your todo list.");
    }
    else {
        res.answer = cleanuproom;
    }
    */
    res.answer = open_challenge;

    return true;
}


}
