#ifndef OBSERVERS_AND_API_INC_SIMSTEPPER_HPP_
#define OBSERVERS_AND_API_INC_SIMSTEPPER_HPP_


#include "ConfBuilder.hpp"
#include "Sim.hpp"
#include "YamlState.hpp"

#include <string>
#include <vector>

class SimServerInputs;

class SimStepper
{
    public:
        SimStepper(const ConfBuilder& builder, const std::string& solver, const double dt);
        std::vector<YamlState> step(const SimServerInputs& input, double Dt);
        std::vector<YamlState> handle_request(const SimServerInputs& input);

    private:
        Sim sim;
        const std::string solver;
        const double dt;
        double current_time;
};

#endif /* OBSERVERS_AND_API_INC_SIMSTEPPER_HPP_ */
