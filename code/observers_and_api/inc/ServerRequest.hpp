/*
 * ServerRequest.hpp
 *
 *  Created on: 24 juin 2020
 *      Author: mcharlou2016
 */

#ifndef OBSERVERS_AND_API_INC_SERVERREQUEST_HPP_
#define OBSERVERS_AND_API_INC_SERVERREQUEST_HPP_

#include <string>

#include "Sim.hpp"
#include "YamlState.hpp"
#include "State.hpp"
#include <ssc/json.hpp>
#include <ssc/kinematics.hpp>

// Base (abstract) class for all requests made to the server. The interface declared here is used by the MessageHandler.
class ServerRequest
{
public:
	ServerRequest(){}
	virtual ~ServerRequest(){}

	virtual void execute(Sim& sim) =0;
	virtual rapidjson::Document response() =0;
};

// ComputeRequest
class ComputeRequest : public ServerRequest
{
public:
	ComputeRequest(const rapidjson::Value& JSON_input, const double dt_, const std::string solver_);
	void execute(Sim& sim);
	rapidjson::Document response();
private:
	const double dt;
	const std::string solver;
	double Dt;
	rapidjson::Document results;
};

// GetTmaxRequest
class GetTmaxRequest : public ServerRequest
{
public:
	GetTmaxRequest();
	void execute(Sim& sim);
	rapidjson::Document response();
private:
	double Tmax;
};

// Euler2QuaternionRequest
class Euler2QuaternionRequest : public ServerRequest
{
public:
	Euler2QuaternionRequest(const rapidjson::Value& JSON_input);
	void execute(Sim& sim);
	rapidjson::Document response();
private:
	ssc::kinematics::EulerAngles angles;
	std::tuple<double,double,double,double> quat;
};

// Quaternion2EulerRequest
class Quaternion2EulerRequest : public ServerRequest
{
public:
	Quaternion2EulerRequest(const rapidjson::Value& JSON_input);
	void execute(Sim& sim);
	rapidjson::Document response();
private:
	ssc::kinematics::EulerAngles angles;
	std::tuple<double,double,double,double> quat;
};

// SetSignalsRequest
class SetSignalsRequest : public ServerRequest
{
public:
	SetSignalsRequest(const rapidjson::Value& JSON_input);
	void execute(Sim& sim);
	rapidjson::Document response();
private:
	std::map<std::string,double> signals;
};

// SetStatesHistoryRequest
class SetStatesHistoryRequest : public ServerRequest
{
public:
	SetStatesHistoryRequest(const rapidjson::Value& JSON_input, const double Tmax);
	void execute(Sim& sim);
	rapidjson::Document response();
private:
	void record(const rapidjson::Value& JSON_input, const double t ,const rapidjson::SizeType i);
	void check_input(const rapidjson::Value& JSON_input);
	State states;
};

#endif /* OBSERVERS_AND_API_INC_SERVERREQUEST_HPP_ */
