/*
 * ForceModelAtH.hpp
 *
 *  Created on: 11 sept. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_FORCEMODELATH_HPP_
#define CORE_INC_FORCEMODELATH_HPP_

#include "ForceModel.hpp"

class ForceModelAtH : public ForceModel
{
public:
	ForceModelAtH(const std::string& force_name, const std::string body_name);
	ForceModelAtH(const std::string& force_name, const std::string body_name, const std::vector<std::string>& commands_);

protected:
	ssc::kinematics::Wrench compute_wrench_from_force(const BodyStates& states, const EnvironmentAndFrames& env, Vector6d& force) override;

private:
	ForceModelAtH(); // Deactivated
	// Note: The following is not really necessary since derived class does not inherit constructors in any case
	ForceModelAtH(const std::string& force_name, const std::string body_name, const EnvironmentAndFrames& env, const YamlPosition& internal_frame); // Deactivated
	ForceModelAtH(const std::string& force_name, const std::string body_name, const EnvironmentAndFrames& env, const YamlPosition& internal_frame, const std::vector<std::string>& commands_); // Deactivated

};

#endif /* CORE_INC_FORCEMODELATH_HPP_ */
