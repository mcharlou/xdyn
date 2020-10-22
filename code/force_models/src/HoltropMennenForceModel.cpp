/*
 * HoltropMennenForceModel.cpp
 *
 *  Created on: 16 janv. 2020
 *      Author: mcharlou2016
 */

#include <math.h>
#include <iostream>
#include <ssc/yaml_parser.hpp>
#include <ssc/kinematics.hpp>
#include <ssc/exception_handling.hpp>

#include "Body.hpp"
#include "yaml.h"
#include "EnvironmentAndFrames.hpp"
#include "InvalidInputException.hpp"
#include "YamlPosition.hpp"

#include "HoltropMennenForceModel.hpp"

#define CHECK_VALID_DOUBLE(name,value) if(isnan(value) || isinf(value)) std::cout << "Variable '" << name << "' has invalid value '" << value << "' in Holtrop & Mennen force model. Please check the input." << std::endl;
//#define CHECK_VALID_DOUBLE(name,value) std::cout << name << " = " << value << std::endl;

std::string HoltropMennenForceModel::model_name() {return "Holtrop & Mennen";}

HoltropMennenForceModel::Yaml::Yaml():  Lwl(),
					Lpp(),
					B(),
					Ta(),
					Tf(),
					Vol(),
					lcb(),
					S(),
					Abt(),
					hb(),
					Cm(),
					Cwp(),
					At(),
					Sapp(),
					Cstern(),
					form_coeff_app(),
					apply_on_ship_speed_direction()
{}

HoltropMennenForceModel::Yaml HoltropMennenForceModel::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    Yaml ret;

    ssc::yaml_parser::parse_uv(node["Lwl"], ret.Lwl);
    ssc::yaml_parser::parse_uv(node["Lpp"], ret.Lpp);
    ssc::yaml_parser::parse_uv(node["B"], ret.B);
    ssc::yaml_parser::parse_uv(node["Ta"], ret.Ta);
    ssc::yaml_parser::parse_uv(node["Tf"], ret.Tf);
    ssc::yaml_parser::parse_uv(node["Vol"], ret.Vol);
    node["lcb"] >> ret.lcb;
    ssc::yaml_parser::parse_uv(node["S"], ret.S);
    ssc::yaml_parser::parse_uv(node["Abt"], ret.Abt);
    ssc::yaml_parser::parse_uv(node["hb"], ret.hb);
    node["Cm"] >> ret.Cm;
    node["Cwp"] >> ret.Cwp;
    ssc::yaml_parser::parse_uv(node["At"], ret.At);
    ssc::yaml_parser::parse_uv(node["Sapp"], ret.Sapp);
    node["Cstern"] >> ret.Cstern;
    node["1+k2"] >> ret.form_coeff_app;
    node["apply on ship speed direction"] >> ret.apply_on_ship_speed_direction;

    return ret;
}

HoltropMennenForceModel::HoltropMennenForceModel(const Yaml& data, const std::string& body_name, const EnvironmentAndFrames& env):
	ForceModelAtH(HoltropMennenForceModel::model_name(), body_name),
	apply_on_ship_speed_direction(data.apply_on_ship_speed_direction),
	d(-0.9),
	L(data.Lwl),
	Lpp(data.Lpp),
	B(data.B),
	c7((B/L<0.11 ? 0.229577*pow(B/L,0.33333) : (B/L>0.25 ? 0.5-0.0625*L/B : B/L))),
	Ta(data.Ta),
	Tf(data.Tf),
	c4((Tf/L>0.04 ? 0.04 : Tf/L)),
	T((data.Ta+data.Tf)/2),
	m3(-7.2035*pow(B/L,0.326869)*pow(T/B,0.605375)),
	Vol(data.Vol),
	Cb(Vol/(L*B*T)),
	c15((pow(L,3)/Vol<512 ? -1.69385 : (pow(L,3)/Vol>1726.91 ? 0 : -1.69385+(L/pow(Vol,1/3.)-8)/2.36))),
	lcb(data.lcb),
	S(data.S),
	Abt(data.Abt),
	hb(data.hb),
	Pb(0.56*sqrt(Abt)/(Tf-1.5*hb)),
	c3(0.56*pow(Abt,1.5)/(B*T*(0.31*sqrt(Abt)+Tf-hb))),
	c2(exp(-1.89*sqrt(c3))),
	Ca(),
	Cm(data.Cm),
	c17(6919.3*pow(Cm,-1.3346)*pow(Vol/pow(L,3.),2.00977)*pow(L/B-2,1.40692)),
	Cp(Vol/(Cm*B*T*L)),
	c16((Cp<0.8 ? 8.07981*Cp-13.8673*pow(Cp,2.)+6.984388*pow(Cp,3.) : 1.73014-0.7067*Cp)),
	m1(0.0140407*L/T-1.75254*pow(Vol,1/3.)/L-4.79323*B/L-c16),
	Lr(L*(1-Cp+0.06*Cp*lcb/(4*Cp-1))),
	lambda((L/B<12 ? 1.446*Cp-0.03*L/B : 1.446*Cp-0.36)),
	Cwp(data.Cwp),
	iE(1+89*exp(-pow(L/B,0.80856)*pow(1-Cwp,0.30484)*pow(1-Cp-0.0225*lcb,0.6367)*pow(Lr/B,0.34574)*pow(100*Vol/pow(L,3.),0.16302))),
	c1(2223105*pow(c7,3.78613)*pow(T/B,1.07961)*pow(90-iE,-1.37565)),
	At(data.At),
	c5(1-0.8*At/(B*T*Cm)),
	Sapp(data.Sapp),
	Cstern(data.Cstern),
	c14(1+0.011*Cstern),
	form_coeff_hull(0.93+0.487118*c14*pow(B/Lpp,1.06806)*pow(T/L,0.46106)*pow(L/Lr,0.121563)*pow(pow(L,3.)/Vol,0.36486)*pow(1-Cp,-0.604247)),
	form_coeff_app(data.form_coeff_app),
	Rw_a([&](double Fn, double m4){return c1*c2*c5*Vol*env.rho*env.g*exp(m1*pow(Fn,d)+m4*cos(lambda*pow(Fn,-2.)));}),
	Rw_b([&](double Fn, double m4){return c17*c2*c5*Vol*env.rho*env.g*exp(m3*pow(Fn,d)+m4*cos(lambda*pow(Fn,-2.)));})
{
	Ca = 0.006*pow(L+100.,-0.16)-0.00205+0.003*sqrt(L/7.5)*pow(Cb,4.)*c2*(0.04-c4);
	CHECK_VALID_DOUBLE("Lwl",L);
	CHECK_VALID_DOUBLE("Lpp",Lpp);
	CHECK_VALID_DOUBLE("B",B);
	CHECK_VALID_DOUBLE("c7",c7);
	CHECK_VALID_DOUBLE("Ta",Ta);
	CHECK_VALID_DOUBLE("Tf",Tf);
	CHECK_VALID_DOUBLE("c4",c4);
	CHECK_VALID_DOUBLE("T",T);
	CHECK_VALID_DOUBLE("m3",m3);
	CHECK_VALID_DOUBLE("Vol",Vol);
	CHECK_VALID_DOUBLE("Cb",Cb);
	CHECK_VALID_DOUBLE("c15",c15);
	CHECK_VALID_DOUBLE("lcb",lcb);
	CHECK_VALID_DOUBLE("S",S);
	CHECK_VALID_DOUBLE("Abt",Abt);
	CHECK_VALID_DOUBLE("hb",hb);
	CHECK_VALID_DOUBLE("Pb",Pb);
	CHECK_VALID_DOUBLE("c3",c3);
	CHECK_VALID_DOUBLE("c2",c2);
	CHECK_VALID_DOUBLE("Ca",Ca);
	CHECK_VALID_DOUBLE("Cm",Cm);
	CHECK_VALID_DOUBLE("c17",c17);
	CHECK_VALID_DOUBLE("Cp",Cp);
	CHECK_VALID_DOUBLE("c16",c16);
	CHECK_VALID_DOUBLE("m1",m1);
	CHECK_VALID_DOUBLE("Lr",Lr);
	CHECK_VALID_DOUBLE("lambda",lambda);
	CHECK_VALID_DOUBLE("Cwp",Cwp);
	CHECK_VALID_DOUBLE("iE",iE);
	CHECK_VALID_DOUBLE("c1",c1);
	CHECK_VALID_DOUBLE("At",At);
	CHECK_VALID_DOUBLE("c5",c5);
	CHECK_VALID_DOUBLE("Sapp",Sapp);
	CHECK_VALID_DOUBLE("Cstern",Cstern);
	CHECK_VALID_DOUBLE("c14",c14);
	CHECK_VALID_DOUBLE("form_coeff_hull",form_coeff_hull);
	CHECK_VALID_DOUBLE("form_coeff_app",form_coeff_app);
}

Vector6d HoltropMennenForceModel::get_force(const BodyStates& states, const double, const EnvironmentAndFrames& env, const std::map<std::string,double>&) const
{
    Vector6d tau = Vector6d::Zero();
    if(states.u()>0){
//	std::cout << "---------------------------------------------------" << std::endl;
//	std::cout << "Fn:\t" << states.u()/sqrt(g*L) << std::endl;
    	double R=Rf(states, env)+Rapp(states, env)+Rw(states, env)+Rb(states, env)+Rtr(states, env)+Ra(states, env);
    	if(!apply_on_ship_speed_direction){
    		tau(0)=-R;
    	}
    	else{
    		Eigen::Vector3d dir=states.get_speed().normalized();
    		tau(0)=-dir(0)*R;
    		tau(1)=-dir(1)*R;
    		tau(2)=-dir(2)*R;
    	}
//	std::cout << "Total resistance:\t" << -tau(0)/1000 << std::endl;
    }
    return tau;
}

double HoltropMennenForceModel::Rf(const BodyStates& states, const EnvironmentAndFrames& env) const
{
  const double Re=states.u()*L/env.nu;
  const double Cf=0.075/pow(log10(Re)-2,2.);
  const double Rf=Cf*0.5*env.rho*pow(states.u(),2.)*S;
//  std::cout << "Rf:\t" << form_coeff_hull*Rf/1000 << std::endl;
  return form_coeff_hull*Rf;
}

double HoltropMennenForceModel::Rapp(const BodyStates& states, const EnvironmentAndFrames& env) const
{
  const double Re=states.u()*L/env.nu;
  const double Cf=0.075/pow(log10(Re)-2,2.);
  const double Rapp=Cf*0.5*env.rho*pow(states.u(),2.)*Sapp;
//  std::cout << "Rapp:\t" << form_coeff_app*Rapp/1000 << std::endl;
  return form_coeff_app*Rapp;
}

double HoltropMennenForceModel::Rw(const BodyStates& states, const EnvironmentAndFrames& env) const
{
  const double Fn=states.u()/sqrt(env.g*L);
  double m4=c15*0.4*exp(-0.034*pow(Fn,-3.29));

  double Rw;

  if(Fn==0){
      Rw=0;
  }
  else if(Fn<0.4){
      Rw=Rw_a(Fn,m4);
  }
  else if(Fn>0.55){
      Rw=Rw_b(Fn,m4);
  }
  else{
      Rw=Rw_a(0.4,m4)+(10*Fn-4)*(Rw_b(0.55,m4)-Rw_a(0.4,m4))/1.5;
  }
//  std::cout << "Rw:\t" << Rw/1000 << std::endl;
  return Rw;
}

double HoltropMennenForceModel::Rb(const BodyStates& states, const EnvironmentAndFrames& env) const
{
  const double Fni=states.u()/sqrt(env.g*(Tf-hb-0.25*sqrt(Abt))+0.15*pow(states.u(),2.));
  const double Rb=0.11*exp(-3*pow(Pb,-2)*pow(Fni,3.)*pow(Abt,1.5)*env.rho*env.g*(1+pow(Fni,2.)));
//  std::cout << "Rb:\t" << Rb/1000 << std::endl;
  return Rb;
}

double HoltropMennenForceModel::Rtr(const BodyStates& states, const EnvironmentAndFrames& env) const
{
  const double FnT=states.u()/sqrt(2*env.g*At/(B+B*Cwp));
  const double c6=(FnT<5 ? 0.2*(1-0.2*FnT) : 0);
  const double Rtr=0.5*env.rho*pow(states.u(),2.)*At*c6;
//  std::cout << "Rtr:\t" << Rtr/1000 << std::endl;
  return Rtr;
}

double HoltropMennenForceModel::Ra(const BodyStates& states, const EnvironmentAndFrames& env) const
{
  const double Ra=0.5*env.rho*pow(states.u(),2.)*S*Ca;
//  std::cout << "Ra:\t" << Ra/1000 << std::endl;
  return Ra;
}


