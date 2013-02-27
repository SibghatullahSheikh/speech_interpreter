/*
 * Interpreter.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jelfring
 */

#include "Interpreter.h"

#include "std_srvs/Empty.h"

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
    category_map_["object_category"].push_back(std::make_pair("drink", 8));
    category_map_["object_category"].push_back(std::make_pair("snack", 4));
    category_map_["object_category"].push_back(std::make_pair("food", 6));
    category_map_["object_category"].push_back(std::make_pair("bathroomstuff", 8));
    category_map_["location_category"].push_back(std::make_pair("seat", 6));
    category_map_["location_category"].push_back(std::make_pair("table", 4));
    category_map_["location_category"].push_back(std::make_pair("shelf", 4));
    category_map_["location_category"].push_back(std::make_pair("appliance", 3));
    category_map_["location_category"].push_back(std::make_pair("bin", 1));

    // Action category map
    action_category_map_["transport"].push_back("object_category");
    action_category_map_["transport"].push_back("location_category");
    action_category_map_["get"].push_back("object_category");
    action_category_map_["find"].push_back("object_category");
    action_category_map_["point"].push_back("object_category");

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

    } else {

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
    res.action = "";
    res.start_location = "meeting_point";
    res.end_location = "";
    res.object = "";

    // Get the action from the user
    double time_out_action = req.time_out;
    ROS_INFO("I will get you an action within %f seconds", time_out_action);
    double t_start = ros::Time::now().toSec();

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
        return true;
    }

    // Ask for additional information
    for (std::vector<std::string>::const_iterator it = it_action->second.begin(); it != it_action->second.end(); ++it) {
        ROS_DEBUG("%s is needed", it->c_str());

        // Check which instance of the object/location is already given
        std::string response = "";
        for (std::vector<std::pair<std::string, int> >::const_iterator it_cat = category_map_[(*it)].begin();
                it_cat != category_map_[(*it)].end(); ++it_cat) {

            // Check if action contains current object/location category
            if (action.find(it_cat->first) != std::string::npos) {

                // Found category, check if category has multiple possible values
                if (it_cat->second > 1) {

                    // TODO Now fixed maximum number of attempts, change?
                    ROS_DEBUG("%s has %d possible values, need to ask user for more information", it_cat->first.c_str(), it_cat->second);
                    amigoSpeak("I am sorry, but I need more information. Can you be more specific?");
                    response = askUser(it_cat->first, 5, ros::Time::now().toSec() - t_start);

                } else {
                    ROS_DEBUG("%s has only one possible value", it_cat->first.c_str());
                    amigoSpeak("I only know one " + it_cat->first);
                    response = it_cat->first;
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

    // If not asked for a location, end location is meeting_point (in case of get me, give me)
    if (res.end_location.empty()) {
        res.end_location = "meeting_point";
        // TODO Note point action does not need an end location, in that case start location can be ignored
    }

    return true;
}


/**
 * Callback for speech
 */
void Interpreter::speechCallback(std_msgs::String res) {

    if (res.data != "") {
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
    } else ROS_INFO("Switched on speech recognition for: %s", category.c_str());

    answer_.clear();
    double start_time = ros::Time::now().toSec();
    ros::Rate r(10);
    while (answer_.empty()) {
        if (ros::Time::now().toSec() - start_time > t_max) {
            ROS_WARN("Timeout: No input over speech topic %s for %f seconds", category.c_str(), t_max);
            break;
        }
        ros::spinOnce();
        r.sleep();
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
    const double t_max_question = std::max(9.0, time_out/(2.0*n_tries_max));

    // Check if the type starts with a vowel
    std::string vowels = "auioe";
    bool start_with_vowel = (vowels.find(type[0])< vowels.length());


    // Determine first question
    std::string txt = "", starting_txt, result = "";
    if (type == "name") {
        starting_txt = "can you give me your name?";
    } else if (type == "action") {
        starting_txt = "what do you want me to do?";

        // Remap type such that it matches the topic name
        type = "sentences";
    } else {
        std::string art = (start_with_vowel)?"an ":"a ";
        starting_txt = "can you give me " + art + type;
    }

    while (ros::Time::now().toSec() - t_start < time_out && n_tries < n_tries_max) {

        // Ask
        amigoSpeak(starting_txt);

        // If an answer was heared, verify
        if (waitForAnswer(type, t_max_question)) {
            result = answer_;
            amigoSpeak("I heard " + answer_ + ", is that correct?");

            // If answer received, ask for confirmation
            if (waitForAnswer("yesno", t_max_question)) {

                // Check if answer is confirmed
                if (answer_ == "yes" || answer_ == "y") {
                    amigoSpeak("Thank you.");
                    break;
                } else {
                    result = "wrong_answer";
                    amigoSpeak("I heard the wrong answer");
                    ++n_tries;
                }
            }

            // If no answer heard to confirmation question, ask for confirmation again
            else {
                txt = "I did not hear you, did you say " + result + "?";
                amigoSpeak(txt);

                // Check if the answer is confirmed after the second confirmation question
                if (waitForAnswer("yesno", t_max_question)) {

                    // Check if answer is confirmed (second time)
                    if (answer_ == "yes" || answer_ == "y") {
                        amigoSpeak("Thank you.");
                        break;
                    } else {
                        result = "wrong_answer";
                        amigoSpeak("I heard the wrong answer");
                        ++n_tries;
                    }
                }

                // Second confirmation question again did not lead to a yes or no, start all over again
                else {
                    result = "no_answer";
                    amigoSpeak("I did not hear you");
                    ++n_tries;
                }
            }
        }

        // If no answer, ask again




        else {
            result = "no_answer";
            amigoSpeak("I did not hear you");
            ++n_tries;
        }
    }

    return result;
}


/**
 * Function that makes AMIGO speak
 */
void Interpreter::amigoSpeak(std::string txt) {

        // TODO Should be connected to the text to speech module topic
        ROS_INFO("%s", txt.c_str());

}
}