/*
 * BodySatesModule.cpp
 *
 *  Created on: 18 févr. 2020
 *      Author: mcharlou2016
 */

#include "BodyStatesModule.hpp"

BodyStatesModule::BodyStatesModule(DataSource* const data_source, const BodyPtr body_) : DataSourceModule(data_source, body_->get_name()+"_BodyStates"), body(body_)
{
	std::string bodyName=body->get_name();
	ds->alias<double>(namify(bodyName,"x"),"NoOutput_"+namify(bodyName,"x"));
	ds->alias<double>(namify(bodyName,"y"),"NoOutput_"+namify(bodyName,"y"));
	ds->alias<double>(namify(bodyName,"z"),"NoOutput_"+namify(bodyName,"z"));
	ds->alias<double>(namify(bodyName,"u"),"NoOutput_"+namify(bodyName,"u"));
	ds->alias<double>(namify(bodyName,"v"),"NoOutput_"+namify(bodyName,"v"));
	ds->alias<double>(namify(bodyName,"w"),"NoOutput_"+namify(bodyName,"w"));
	ds->alias<double>(namify(bodyName,"phi"),"NoOutput_"+namify(bodyName,"phi"));
	ds->alias<double>(namify(bodyName,"theta"),"NoOutput_"+namify(bodyName,"theta"));
	ds->alias<double>(namify(bodyName,"psi"),"NoOutput_"+namify(bodyName,"psi"));
	ds->alias<double>(namify(bodyName,"p"),"NoOutput_"+namify(bodyName,"p"));
	ds->alias<double>(namify(bodyName,"q"),"NoOutput_"+namify(bodyName,"q"));
	ds->alias<double>(namify(bodyName,"r"),"NoOutput_"+namify(bodyName,"r"));
}

BodyStatesModule::BodyStatesModule(const BodyStatesModule& rhs, DataSource* const data_source) : DataSourceModule(rhs, data_source), body(rhs.body)
{}

std::string BodyStatesModule::get_name() const
{
	return body->get_name()+"_BodyStates";
}

std::string BodyStatesModule::namify(const std::string& bodyName, const std::string& variableName) const
{
	return variableName+"("+bodyName+")";
}

void BodyStatesModule::update() const
{
	ds->get<double>("t"); // So the DataSource knows that this modules requires an update when t changes.
	std::string bodyName=body->get_name();
	//std::cout << "Updating BodyStatesModule for " << bodyName << ", ";
	ds->set<double>("NoOutput_"+namify(bodyName,"x"),body->get_states().x());
	ds->set<double>("NoOutput_"+namify(bodyName,"y"),body->get_states().y());
	ds->set<double>("NoOutput_"+namify(bodyName,"z"),body->get_states().z());
	//std::cout << "value for u is " << body->get_states().u() << std::endl;
	ds->set<double>("NoOutput_"+namify(bodyName,"u"),body->get_states().u());
	ds->set<double>("NoOutput_"+namify(bodyName,"v"),body->get_states().v());
	ds->set<double>("NoOutput_"+namify(bodyName,"w"),body->get_states().w());
	ds->set<double>("NoOutput_"+namify(bodyName,"p"),body->get_states().p());
	ds->set<double>("NoOutput_"+namify(bodyName,"q"),body->get_states().q());
	ds->set<double>("NoOutput_"+namify(bodyName,"r"),body->get_states().r());
	ds->set<double>("NoOutput_"+namify(bodyName,"phi"),body->get_states().get_angles().phi);
	ds->set<double>("NoOutput_"+namify(bodyName,"theta"),body->get_states().get_angles().theta);
	ds->set<double>("NoOutput_"+namify(bodyName,"psi"),body->get_states().get_angles().psi);
}
