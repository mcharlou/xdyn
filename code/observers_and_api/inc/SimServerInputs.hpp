/*
 * SimServerInputs.hpp
 *
 *  Created on: Nov 15, 2018
 *      Author: cady
 */

#ifndef OBSERVERS_AND_API_INC_SIMSERVERINPUTS_HPP_
#define OBSERVERS_AND_API_INC_SIMSERVERINPUTS_HPP_


#include <map>
#include <string>
#include "State.hpp"
#include "YamlSimServerInputs.hpp"

struct SimServerInputs
{
    SimServerInputs(const YamlSimServerInputs& server_inputs, const double max_history_length);
    SimServerInputs(const double Dt);
    double t;
    double Dt;
    StateType state_at_t;
    State state_history_except_last_point;
    State full_state_history;
    bool reset_history;
    std::map<std::string, double> commands;
    Request request;
    private: SimServerInputs(); // Disabled
};


#endif /* OBSERVERS_AND_API_INC_SIMSERVERINPUTS_HPP_ */
