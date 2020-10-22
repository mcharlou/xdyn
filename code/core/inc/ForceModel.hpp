/*
 * ForceModel.hpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */

#ifndef FORCEMODEL_HPP_
#define FORCEMODEL_HPP_

#include <functional>
#include <vector>
#include <memory>
#include <ssc/kinematics.hpp>
#include <ssc/data_source.hpp>


#include <boost/optional/optional.hpp>
#include <boost/utility/enable_if.hpp>

//#include <ssc/macros.hpp>

//#include "Body.hpp"
#include "yaml-cpp/exceptions.h"
#include "InvalidInputException.hpp"
#include "YamlBody.hpp"
#include "EnvironmentAndFrames.hpp"
#include "BodyStates.hpp"

//struct BodyStates;
struct EnvironmentAndFrames;
class ForceModel;

typedef std::shared_ptr<ForceModel> ForcePtr;
typedef std::function<boost::optional<ForcePtr>(const YamlModel&, const std::string&, const EnvironmentAndFrames&)> ForceParser;

class Observer;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

// SFINAE test for 'parse' method
template<typename T>
struct HasParse
{
	typedef char yes[1];
	typedef char no [2];
	template<typename U> static yes &check(decltype(&U::parse));
	template<typename U> static no &check(...);
	static const bool value = sizeof(check<T>(0)) == sizeof(yes);
};

class ForceModel
{
public:
	virtual ~ForceModel(){}
	void update(const BodyStates& states, const double t, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener);
	ssc::kinematics::Wrench operator()(const BodyStates& states, const double t, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener);
	// TODO: The following method is bad practice and should be removed
	//ssc::kinematics::Wrench operator()(const BodyStates& states, const double t); // This method is provided for testing purposes, when we know the force model will not require any commands. It will throw an error if the force model needs commands.
	virtual std::string expression_frame() const;
	virtual ssc::kinematics::Point application_point() const;
	virtual double potential_energy(const BodyStates& body, const EnvironmentAndFrames& env, const std::vector<double>& x) const {(void)body;(void)env;(void)x;return 0;}
	std::string get_name() const;
	//virtual std::string model_name() const = 0; // model_name() must be defined for all instantiable force models, but can't be pure virtual from here because it must also be static
	std::string get_body_name() const;
	virtual bool is_a_surface_force_model() const;
	ssc::kinematics::Wrench get_force_in_body_frame() const; // WARNING: this method returns the current value of the force without updating (recomputing)
	ssc::kinematics::Wrench get_force_in_ned_frame() const; // WARNING: this method returns the current value of the force without updating (recomputing)
	void feed(Observer& observer) const;
	virtual double get_Tmax() const; // Can be overloaded if model needs access to History (not a problem, just has to say how much history to keep)

	static ssc::kinematics::Wrench project_into_NED_frame(const ssc::kinematics::Wrench& F, const ssc::kinematics::RotationMatrix& R);

	template <typename ForceType>
	static typename boost::enable_if<HasParse<ForceType>, ForceParser>::type build_parser()
	{
		auto parser = [](const YamlModel& yaml, const std::string body_name, const EnvironmentAndFrames& env) -> boost::optional<ForcePtr>
		{
			boost::optional<ForcePtr> ret;
			if (yaml.model == ForceType::model_name())
			{
				std::string context = "Invalid input data for model '" + ForceType::model_name() + "'.";
				try
				{
					ret.reset(ForcePtr(new ForceType(ForceType::parse(yaml.yaml), body_name, env)));
				}
				catch (const InvalidInputException& exception)
				{
					THROW(__PRETTY_FUNCTION__, InvalidInputException, context << std::endl << "The error was: " << exception.get_message() << std::endl << "Model containing error is defined line "
						  << yaml.index_of_first_line_in_global_yaml << " of the YAML file." << std::endl);
				}
				catch (const YAML::Exception& exception)
				{
					const size_t line_number = yaml.index_of_first_line_in_global_yaml;
					THROW(__PRETTY_FUNCTION__, InvalidInputException, context << std::endl << "Model containing error is defined line "
						  << line_number << " of the YAML file." << std::endl << "The error was: " << exception.msg);
				}
			}
			return ret;
		};
		return parser;
	}

	template <typename ForceType>
	static typename boost::disable_if<HasParse<ForceType>, ForceParser>::type build_parser()
	{
		auto parser = [](const YamlModel& yaml, const std::string body_name, const EnvironmentAndFrames& env) -> boost::optional<ForcePtr>
		{
			boost::optional<ForcePtr> ret;
			if (yaml.model == ForceType::model_name())
			{
				ret.reset(ForcePtr(new ForceType(body_name, env)));
			}
			return ret;
		};
		return parser;
	}

protected:
	ForceModel(const std::string& force_name, const std::string body_name);
	ForceModel(const std::string& force_name, const std::string body_name, const EnvironmentAndFrames& env, const YamlPosition& internal_frame);
	ForceModel(const std::string& force_name, const std::string body_name, const std::vector<std::string>& commands_);
	ForceModel(const std::string& force_name, const std::string body_name, const EnvironmentAndFrames& env, const YamlPosition& internal_frame, const std::vector<std::string>& commands_);
	virtual void extra_observations(Observer& observer) const;
	virtual Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const = 0;
	virtual ssc::kinematics::Wrench compute_wrench_from_force(const BodyStates& states, const EnvironmentAndFrames& env, Vector6d& force);

	const std::string force_name;
	const std::string body_name;
	ssc::kinematics::Wrench force_in_body_frame;
	ssc::kinematics::Wrench force_in_ned_frame;
	//ssc::kinematics::Transform from_internal_frame_to_a_known_frame;

private:
	ForceModel(); // Disabled
	double get_command(const std::string& command_name, ssc::data_source::DataSource& command_listener) const;
	std::map<std::string,double> get_commands(ssc::data_source::DataSource& command_listener, const double t) const;

	const std::string frame;
	const std::vector<std::string> commands;
};

#endif /* FORCEMODEL_HPP_ */
