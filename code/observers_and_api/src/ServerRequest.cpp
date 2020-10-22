/*
 * ServerRequest.cpp
 *
 *  Created on: 24 juin 2020
 *      Author: mcharlou2016
 */

#include <vector>

#include <ssc/json.hpp>
#include "rapidjson/document.h"
#include "Res.hpp"
#include "simulator_api.hpp"
#include "JSONServerObserver.hpp"

#include "ServerRequest.hpp"

// ComputeRequest
ComputeRequest::ComputeRequest(const rapidjson::Value& JSON_input, const double dt_, const std::string solver_):dt(dt_),solver(solver_),Dt(),results()
{
	Dt = ssc::json::find_double("Dt", JSON_input);
}

void ComputeRequest::execute(Sim& sim)
{
	std::vector<Res> resList;
	double tstart = sim.get_bodies().at(0)->get_states().x.get_current_time();
	sim.update_forces(tstart);

	JSONServerObserver observer;

	if(Dt<dt)
	{
		observer.observe(sim,tstart);
	}
	else
	{
		if(solver == "euler")
		{
			ssc::solver::quicksolve<ssc::solver::EulerStepper>(sim, tstart, tstart+Dt, dt, observer);
		}
		else if (solver == "rk4")
		{
			ssc::solver::quicksolve<ssc::solver::RK4Stepper>(sim, tstart, tstart+Dt, dt, observer);
		}
		else if (solver == "rkck")
		{
			ssc::solver::quicksolve<ssc::solver::RKCK>(sim, tstart, tstart+Dt, dt, observer);
		}
		else
		{
			THROW(__PRETTY_FUNCTION__, InvalidInputException, "unknown solver");
		}
	}

	results = observer.get();
}

rapidjson::Document ComputeRequest::response()
{
	rapidjson::Document ret;
	ret.CopyFrom(results,ret.GetAllocator());
	return ret;
}

// GetTmaxRequest
GetTmaxRequest::GetTmaxRequest():Tmax(0.){}

void GetTmaxRequest::execute(Sim& sim)
{
	Tmax = sim.get_bodies().at(0)->get_Tmax();
}

rapidjson::Document GetTmaxRequest::response()
{
	rapidjson::Document doc(rapidjson::kObjectType);
	doc.AddMember("Tmax",Tmax,doc.GetAllocator());
	return doc;
}

// Euler2QuaternionRequest
Euler2QuaternionRequest::Euler2QuaternionRequest(const rapidjson::Value& JSON_input):angles(),quat()
{
	angles.phi = ssc::json::find_double("phi", JSON_input);
	angles.theta = ssc::json::find_double("theta", JSON_input);
	angles.psi = ssc::json::find_double("psi", JSON_input);
}

void Euler2QuaternionRequest::execute(Sim& sim)
{
	quat = sim.get_bodies()[0]->get_states().convert(angles,sim.get_bodies()[0]->get_states().convention);
}

rapidjson::Document Euler2QuaternionRequest::response()
{
	rapidjson::Document doc(rapidjson::kObjectType);
	doc.AddMember("qr",std::get<0>(quat),doc.GetAllocator());
	doc.AddMember("qi",std::get<1>(quat),doc.GetAllocator());
	doc.AddMember("qj",std::get<2>(quat),doc.GetAllocator());
	doc.AddMember("qk",std::get<3>(quat),doc.GetAllocator());
	return doc;
}

// Euler2QuaternionRequest
Quaternion2EulerRequest::Quaternion2EulerRequest(const rapidjson::Value& JSON_input):angles(),quat()
{
	std::get<0>(quat) = ssc::json::find_double("qr", JSON_input);
	std::get<1>(quat) = ssc::json::find_double("qi", JSON_input);
	std::get<2>(quat) = ssc::json::find_double("qj", JSON_input);
	std::get<3>(quat) = ssc::json::find_double("qk", JSON_input);
}

void Quaternion2EulerRequest::execute(Sim& sim)
{
	angles = sim.get_bodies()[0]->get_states().convert(quat,sim.get_bodies()[0]->get_states().convention);
}

rapidjson::Document Quaternion2EulerRequest::response()
{
	rapidjson::Document doc(rapidjson::kObjectType);
	doc.AddMember("phi",angles.phi,doc.GetAllocator());
	doc.AddMember("theta",angles.theta,doc.GetAllocator());
	doc.AddMember("psi",angles.psi,doc.GetAllocator());
	return doc;
}

// SetSignalsRequest
SetSignalsRequest::SetSignalsRequest(const rapidjson::Value& JSON_input):signals()
{
	if (not(JSON_input.IsObject()) && not(JSON_input.IsNull()))
	{
		THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "'input' should be a JSON object (key-values): got " << ssc::json::print_type(JSON_input))
	}
	else
	{
		if (not(JSON_input.IsNull()))
		{
			for (rapidjson::Value::ConstMemberIterator it = JSON_input.MemberBegin(); it != JSON_input.MemberEnd(); ++it)
			{
				signals[it->name.GetString()] = ssc::json::find_double(it->name.GetString(), JSON_input);
			}
		}
	}
}

void SetSignalsRequest::execute(Sim& sim)
{
	sim.set_command_listener(signals);
}

rapidjson::Document SetSignalsRequest::response()
{
	rapidjson::Document doc(rapidjson::kObjectType);
	return doc;
}

// SetStatesHistoryRequest
SetStatesHistoryRequest::SetStatesHistoryRequest(const rapidjson::Value& JSON_input, const double Tmax):states(Tmax)
{
	check_input(JSON_input);
	rapidjson::SizeType n = JSON_input["t"].Size();
	for(rapidjson::SizeType i=0;i<n;i++)
	{
		record(JSON_input,JSON_input["t"].GetArray()[i].GetDouble(),i);
	}
}

void SetStatesHistoryRequest::check_input(const rapidjson::Value& JSON_input)
{
	if(not(JSON_input.IsObject()))
	{
		THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "'input' should be a JSON object (key-values): got " << ssc::json::print_type(JSON_input));
	}
	if(not(JSON_input.HasMember("t")))
	{
		THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "'input' should have an array of doubles named 't'");
	}
	if(not(JSON_input["t"].IsArray()))
	{
		THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "'t' should be an array of doubles");
	}
	rapidjson::SizeType n = JSON_input["t"].Size();
	std::vector<std::string> fields = {"x","y","z","u","v","w","p","q","r","qr","qi","qj","qk"};
	for(std::string field:fields)
	{
		if(not(JSON_input.HasMember(field.c_str())))
		{
			THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "'input' should have an array of doubles named '"+field+"'");
		}
		if(not(JSON_input[field.c_str()].IsArray()))
		{
			THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "'input' should have an array of doubles named '"+field+"'");
		}
		if(JSON_input[field.c_str()].Size()!=n)
		{
			THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "all state fields should have the same size but '"+field+"' does not have the same size as 't'");
		}
	}
}

void SetStatesHistoryRequest::record(const rapidjson::Value& JSON_input, const double t ,const rapidjson::SizeType i)
{
	states.u.record(t,JSON_input["u"].GetArray()[i].GetDouble());
	states.v.record(t,JSON_input["v"].GetArray()[i].GetDouble());
	states.w.record(t,JSON_input["w"].GetArray()[i].GetDouble());
	states.x.record(t,JSON_input["x"].GetArray()[i].GetDouble());
	states.y.record(t,JSON_input["y"].GetArray()[i].GetDouble());
	states.z.record(t,JSON_input["z"].GetArray()[i].GetDouble());
	states.p.record(t,JSON_input["p"].GetArray()[i].GetDouble());
	states.q.record(t,JSON_input["q"].GetArray()[i].GetDouble());
	states.r.record(t,JSON_input["r"].GetArray()[i].GetDouble());
	states.qr.record(t,JSON_input["qr"].GetArray()[i].GetDouble());
	states.qi.record(t,JSON_input["qi"].GetArray()[i].GetDouble());
	states.qj.record(t,JSON_input["qj"].GetArray()[i].GetDouble());
	states.qk.record(t,JSON_input["qk"].GetArray()[i].GetDouble());
}

void SetStatesHistoryRequest::execute(Sim& sim)
{
	sim.set_bodystates(std::vector<State>({states}));
}

rapidjson::Document SetStatesHistoryRequest::response()
{
	rapidjson::Document doc(rapidjson::kObjectType);
	return doc;
}

