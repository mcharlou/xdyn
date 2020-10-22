#ifndef EXTERNAL_DATA_STRUCTURES_INC_YAMLSIMSERVERINPUTS_HPP_
#define EXTERNAL_DATA_STRUCTURES_INC_YAMLSIMSERVERINPUTS_HPP_
#include "YamlState.hpp"
#include <map>
#include <string>

enum class Request{None,get_Tmax,quaternion2euler,euler2quaternion,force_signals};

struct YamlSimServerInputs
{
    YamlSimServerInputs();
    double Dt;
    std::vector<YamlState> states;
    std::map<std::string, double> commands;
    Request request;
};



#endif /* EXTERNAL_DATA_STRUCTURES_INC_YAMLSIMSERVERINPUTS_HPP_ */
