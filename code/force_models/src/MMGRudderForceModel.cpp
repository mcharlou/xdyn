/*
 * MMGRudderForceModel.cpp
 *
 *  Created on: 5 mai 2020
 *      Author: mcharlou2016
 */

#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <ssc/yaml_parser.hpp>
#include <ssc/kinematics.hpp>
#include "ForceModel.hpp"
#include "BodyStates.hpp"
#include "yaml.h"
#include "external_data_structures_parsers.hpp"
#include "Observer.hpp"

#include "MMGRudderForceModel.hpp"

#define PI M_PI

class MMGRudderForceModel::Impl
{
public:

	Impl(const Yaml& input, const EnvironmentAndFrames& env,const std::string name_) :
					name(name_),
					rho(env.rho),
					Lpp(input.Lpp),
					Dp(input.Dp),
					xp(input.xp_*input.Lpp),
					k0(input.k0),
					k1(input.k1),
					k2(input.k2),
					wp0(input.wp0),
					C1(input.C1),
					C2_pos(input.C2_pos),
					C2_neg(input.C2_neg),
					tp(input.tp),
					Jp_max(input.Jp_max),
					xr(input.xr_*input.Lpp),
					tr(input.tr),
					ah(input.ah),
					xh(input.xh_*input.Lpp),
					kappa(input.kappa),
					lr_(input.lr_),
					epsilon(input.epsilon),
					fx(input.fx),
					gamma_r_pos(input.gamma_r_pos),
					gamma_r_neg(input.gamma_r_neg),
					Ar(input.Ar),
					Hr(input.Hr),
					f_alpha(input.f_alpha),
					eta(input.Dp/input.Hr),
					F_N(),
					X_P(),
					X_R(),
					Y_R(),
					N_R()
	{}

	void update(const BodyStates& states, const std::map<std::string,double>& commands, const EnvironmentAndFrames& env)
		{
			//std::cout << "New step ............................... "<< std::endl;
			double v = states.v();
			//std::cout << "v: " << v << std::endl;
		    double xG = states.G.v(0) - env.k->get(states.name,name).get_point().x();
		    //std::cout << "xG: " << xG << std::endl;
		    double vm = v - xG*states.r();
		    //std::cout << "r: " << states.r() << std::endl;
		    //std::cout << "vm: " << vm << std::endl;
			double U = sqrt(pow(states.u(),2)+pow(vm,2));
			double r_ = (U!=0 ? states.r()*Lpp/U : 0);

			// Propeller part
			double beta = atan2(-vm,states.u());//(states.u()!=0 ? atan(-states.v()/states.u()) : (states.v()!=0 ? states.v()*PI/2/abs(states.v()) : 0));
			double np = commands.at("rpm")/(2*PI);// Convert from rad/s to rps
			//std::cout << "np (rad/s): " << commands.at("rpm") << std::endl;
			//std::cout << "np (rps): " << np << std::endl;
			double beta_p = beta - xp/Lpp*r_;
			double C2 = (beta_p>0 ? C2_pos : C2_neg);
			double wp = 1 - (1 + (1 - exp(-C1*abs(beta_p)))*(C2-1))*(1 - wp0);
			double Jp;
			if(states.u()==0) Jp = 0;
			else if(np==0) Jp = Jp_max;
			else
			{
				Jp = states.u()*(1 - wp)/(np*Dp);
				if(Jp<0) Jp = 0;
				else if(Jp>Jp_max) Jp = Jp_max;
			}
			//std::cout << "Jp: " << Jp << std::endl;
			double Kt = k2*Jp*Jp + k1*Jp + k0;
			//std::cout << "Kt: " << Kt << std::endl;
			double T = rho*pow(np,2)*pow(Dp,4)*Kt;

			X_P = (1 - tp)*T;

			//std::cout << "X_P: " << X_P << std::endl;

			// Rudder part
			//std::cout << "u: " << states.u() << ", vm: " << vm << std::endl;
			//std::cout << "wp: " << wp << std::endl;
			double beta_r = beta - lr_*r_;
			double gamma_r = (beta_r>0 ? gamma_r_pos : gamma_r_neg);
			double vr = U*gamma_r*beta_r;
			//std::cout << "Jp: " << Jp << std::endl;
			//std::cout << "Kt: " << Kt << std::endl;
			double inf_amp = (Jp>0 ? sqrt(1 + 8*Kt/(PI*Jp*Jp)) : 1);
			//std::cout << "inf_amp: " << inf_amp << std::endl;
			double ur = epsilon*states.u()*(1 - wp)*sqrt(eta*pow(1 + kappa*(inf_amp - 1),2) + 1 - eta);
			//std::cout << "ur: " << ur << ", vr: " << vr << std::endl;
			double Ur = sqrt(ur*ur+vr*vr);
			double delta = commands.at("beta");
			double alpha_r = delta - atan2(vr,ur);//(ur!=0 ? atan(vr/ur) : (vr!=0 ? vr*PI/2/abs(vr) : 0));
			F_N = 0.5*rho*Ar*pow(Ur,2)*f_alpha*sin(alpha_r);

			//std::cout << "F_N: " << F_N << std::endl;

			X_R = -(1 - tr)*F_N*sin(delta);
			Y_R = -(1 + ah)*F_N*cos(delta);
			N_R = -(xr + ah*xh)*F_N*cos(delta);

			//std::cout << "X_R: " << X_R << std::endl;
		}

	ssc::kinematics::Vector6d get_total_force()
	{
		ssc::kinematics::Vector6d force = ssc::kinematics::Vector6d::Zero();
		force[0] = X_P + X_R;
		force[1] = Y_R;
		force[5] = N_R;
		return force;
	}

	ssc::kinematics::Vector6d get_rudder_force()
	{
		ssc::kinematics::Vector6d force = ssc::kinematics::Vector6d::Zero();
		force[0] = X_R;
		force[1] = Y_R;
		force[5] = N_R;
		return force;
	}

	ssc::kinematics::Vector6d get_propeller_force()
	{
		ssc::kinematics::Vector6d force = ssc::kinematics::Vector6d::Zero();
		force[0] = X_P;
		return force;
	}

	double get_F_N() const
	{
		return F_N;
	}

private:
	std::string name;

	double rho;
	double Lpp;

	double Dp;
	double xp;
	double k0;
	double k1;
	double k2;
	double wp0;
	double C1;
	double C2_pos;
	double C2_neg;
	double tp;
	double Jp_max;

	double xr;
	double tr;
	double ah;
	double xh;
	double kappa;
	double lr_;
	double epsilon;
	double fx;
	double gamma_r_pos;
	double gamma_r_neg;
	double Ar;
	double Hr;
	double f_alpha;
	double eta;

	double F_N;
	double X_P;
	double X_R;
	double Y_R;
	double N_R;
};

std::string MMGRudderForceModel::model_name() {return "MMG propeller+rudder";}

MMGRudderForceModel::Yaml::Yaml() :
			name(),
			position_of_local_frame(),
			Lpp(),
			Dp(),
			xp_(),
			k0(),
			k1(),
			k2(),
			wp0(),
			C1(),
			C2_pos(),
			C2_neg(),
			tp(),
			Jp_max(),
			xr_(),
			tr(),
			ah(),
			xh_(),
			kappa(),
			lr_(),
			epsilon(),
			fx(),
			gamma_r_pos(),
			gamma_r_neg(),
			Ar(),
			Hr(),
			f_alpha()
{
}

MMGRudderForceModel::MMGRudderForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env) :
				ForceModel(input.name, body_name, env, input.position_of_local_frame, {"rpm","beta"}),
				pimpl(new Impl(input,env,input.name))
{
}

MMGRudderForceModel::Yaml MMGRudderForceModel::parse(const std::string& yaml)
{
	std::stringstream stream(yaml);
	YAML::Parser parser(stream);
	YAML::Node node;
	parser.GetNextDocument(node);
	Yaml ret;

	node["name"] >> ret.name;
	ssc::yaml_parser::parse_uv(node["Lpp"], ret.Lpp);
	ssc::yaml_parser::parse_uv(node["Dp"], ret.Dp);
	node["position of calculation frame"] >> ret.position_of_local_frame;

	node["xp_"] >> ret.xp_;
	node["k0"] >> ret.k0;
	node["k1"] >> ret.k1;
	node["k2"] >> ret.k2;
	node["wp0"] >> ret.wp0;
	node["C1"] >> ret.C1;
	node["C2_pos"] >> ret.C2_pos;
	node["C2_neg"] >> ret.C2_neg;
	node["tp"] >> ret.tp;
	node["Jp_max"] >> ret.Jp_max;
	node["xr_"] >> ret.xr_;
	node["tr"] >> ret.tr;
	node["ah"] >> ret.ah;
	node["xh_"] >> ret.xh_;
	node["kappa"] >> ret.kappa;
	node["lr_"] >> ret.lr_;
	node["epsilon"] >> ret.epsilon;
	node["fx"] >> ret.fx;
	node["gamma_r_pos"] >> ret.gamma_r_pos;
	node["gamma_r_neg"] >> ret.gamma_r_neg;
	node["Ar"] >> ret.Ar;
	node["Hr"] >> ret.Hr;
	node["f_alpha"] >> ret.f_alpha;

	return ret;
}

ssc::kinematics::Vector6d MMGRudderForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
	pimpl->update(states,commands,env);
	return pimpl->get_total_force();
}

#define WRITE(observer,body,force,frame,axe,value) observer.write(value,DataAddressing({"efforts",body,force,frame,axe},std::string(axe)+"("+force+","+body+","+frame+")"));
void MMGRudderForceModel::feed(Observer& observer, ssc::kinematics::KinematicsPtr& k, const ssc::kinematics::Point& G) const
{
    // G is the point in which 'latest_force_in_body_frame' is expressed (sum of forces)
    // O is the origin of the NED frame
    // O1 is the origin of the body frame (current ship position)
    // P is the origin of the ControllableForceModel's internal frame
	std::string name = get_name();
	std::string body_name = get_body_name();
	const ssc::kinematics::Vector6d rudder_force_in_internal_frame = pimpl->get_rudder_force();
	const ssc::kinematics::UnsafeWrench tau_rudder_in_internal_frame(ssc::kinematics::Point(name, 0, 0, 0), rudder_force_in_internal_frame.segment(0,2), rudder_force_in_internal_frame.segment(3,5));
    const ssc::kinematics::Vector6d propeller_force_in_internal_frame = pimpl->get_propeller_force();
    const ssc::kinematics::UnsafeWrench tau_propeller_in_internal_frame(ssc::kinematics::Point(name, 0, 0, 0), propeller_force_in_internal_frame.segment(0,2), propeller_force_in_internal_frame.segment(3,5));

    const auto Tinternal_to_body = k->get(name, body_name);
    const auto rot_from_internal_frame_to_body_frame = Tinternal_to_body.get_rot().transpose();
    const auto Tinternal_to_ned = k->get(name, "NED");
    const auto rot_from_internal_frame_to_ned = Tinternal_to_ned.get_rot().transpose();

    const Eigen::Vector3d O1P = k->get(body_name, name).get_point().v;
    std::cout << "P: " << k->get(body_name, name).get_point() << std::endl;
    std::cout << "O1P: " << O1P.transpose() << std::endl;
    const Eigen::Vector3d O1G = G.v;
    std::cout << "G: " << G << std::endl;
    std::cout << "O1G: " << O1G.transpose() << std::endl;
    const Eigen::Vector3d OP = Tinternal_to_ned.get_point().v;

    const Eigen::Vector3d GP = -O1G + O1P;
    const auto rudder_force_in_body_frame_at_G = rot_from_internal_frame_to_body_frame*tau_rudder_in_internal_frame.force;
    const auto rudder_torque_in_body_frame_at_G = rot_from_internal_frame_to_body_frame*(tau_rudder_in_internal_frame.torque+GP.cross(tau_rudder_in_internal_frame.force));
    const auto propeller_force_in_body_frame_at_G = rot_from_internal_frame_to_body_frame*tau_propeller_in_internal_frame.force;
    const auto propeller_torque_in_body_frame_at_G = rot_from_internal_frame_to_body_frame*(tau_propeller_in_internal_frame.torque+GP.cross(tau_propeller_in_internal_frame.force));

    const auto rudder_force_in_ned_frame_at_O = rot_from_internal_frame_to_ned*tau_rudder_in_internal_frame.force;
    const auto rudder_torque_in_ned_frame_at_O = rot_from_internal_frame_to_ned*(tau_rudder_in_internal_frame.torque+OP.cross(tau_rudder_in_internal_frame.force));
    const auto propeller_force_in_ned_frame_at_O = rot_from_internal_frame_to_ned*tau_propeller_in_internal_frame.force;
    const auto propeller_torque_in_ned_frame_at_O = rot_from_internal_frame_to_ned*(tau_propeller_in_internal_frame.torque+OP.cross(tau_propeller_in_internal_frame.force));


    WRITE(observer,body_name,name+"_rudder",body_name,"Fx",rudder_force_in_body_frame_at_G(0))
    WRITE(observer,body_name,name+"_rudder",body_name,"Fy",rudder_force_in_body_frame_at_G(1))
	WRITE(observer,body_name,name+"_rudder",body_name,"Fz",rudder_force_in_body_frame_at_G(2))
	WRITE(observer,body_name,name+"_rudder",body_name,"Mx",rudder_torque_in_body_frame_at_G(0))
	WRITE(observer,body_name,name+"_rudder",body_name,"My",rudder_torque_in_body_frame_at_G(1))
	WRITE(observer,body_name,name+"_rudder",body_name,"Mz",rudder_torque_in_body_frame_at_G(2))

	WRITE(observer,body_name,name+"_rudder",name,"Fx",tau_rudder_in_internal_frame.X())
	WRITE(observer,body_name,name+"_rudder",name,"Fy",tau_rudder_in_internal_frame.Y())
	WRITE(observer,body_name,name+"_rudder",name,"Fz",tau_rudder_in_internal_frame.Z())
	WRITE(observer,body_name,name+"_rudder",name,"Mx",tau_rudder_in_internal_frame.K())
	WRITE(observer,body_name,name+"_rudder",name,"My",tau_rudder_in_internal_frame.M())
	WRITE(observer,body_name,name+"_rudder",name,"Mz",tau_rudder_in_internal_frame.N())

	WRITE(observer,body_name,name+"_rudder","NED","Fx",rudder_force_in_ned_frame_at_O(0))
	WRITE(observer,body_name,name+"_rudder","NED","Fy",rudder_force_in_ned_frame_at_O(1))
	WRITE(observer,body_name,name+"_rudder","NED","Fz",rudder_force_in_ned_frame_at_O(2))
	WRITE(observer,body_name,name+"_rudder","NED","Mx",rudder_torque_in_ned_frame_at_O(0))
	WRITE(observer,body_name,name+"_rudder","NED","My",rudder_torque_in_ned_frame_at_O(1))
	WRITE(observer,body_name,name+"_rudder","NED","Mz",rudder_torque_in_ned_frame_at_O(2))

	WRITE(observer,body_name,name+"_propeller",body_name,"Fx",propeller_force_in_body_frame_at_G(0))
	WRITE(observer,body_name,name+"_propeller",body_name,"Fy",propeller_force_in_body_frame_at_G(1))
	WRITE(observer,body_name,name+"_propeller",body_name,"Fz",propeller_force_in_body_frame_at_G(2))
	WRITE(observer,body_name,name+"_propeller",body_name,"Mx",propeller_torque_in_body_frame_at_G(0))
	WRITE(observer,body_name,name+"_propeller",body_name,"My",propeller_torque_in_body_frame_at_G(1))
	WRITE(observer,body_name,name+"_propeller",body_name,"Mz",propeller_torque_in_body_frame_at_G(2))

	WRITE(observer,body_name,name+"_propeller",name,"Fx",tau_propeller_in_internal_frame.X())
	WRITE(observer,body_name,name+"_propeller",name,"Fy",tau_propeller_in_internal_frame.Y())
	WRITE(observer,body_name,name+"_propeller",name,"Fz",tau_propeller_in_internal_frame.Z())
	WRITE(observer,body_name,name+"_propeller",name,"Mx",tau_propeller_in_internal_frame.K())
	WRITE(observer,body_name,name+"_propeller",name,"My",tau_propeller_in_internal_frame.M())
	WRITE(observer,body_name,name+"_propeller",name,"Mz",tau_propeller_in_internal_frame.N())

	WRITE(observer,body_name,name+"_propeller","NED","Fx",propeller_force_in_ned_frame_at_O(0))
	WRITE(observer,body_name,name+"_propeller","NED","Fy",propeller_force_in_ned_frame_at_O(1))
	WRITE(observer,body_name,name+"_propeller","NED","Fz",propeller_force_in_ned_frame_at_O(2))
	WRITE(observer,body_name,name+"_propeller","NED","Mx",propeller_torque_in_ned_frame_at_O(0))
	WRITE(observer,body_name,name+"_propeller","NED","My",propeller_torque_in_ned_frame_at_O(1))
	WRITE(observer,body_name,name+"_propeller","NED","Mz",propeller_torque_in_ned_frame_at_O(2))

	WRITE(observer,body_name,name,body_name,"Fx",propeller_force_in_body_frame_at_G(0))
	WRITE(observer,body_name,name,body_name,"Fy",propeller_force_in_body_frame_at_G(1))
	WRITE(observer,body_name,name,body_name,"Fz",propeller_force_in_body_frame_at_G(2))
	WRITE(observer,body_name,name,body_name,"Mx",propeller_torque_in_body_frame_at_G(0))
	WRITE(observer,body_name,name,body_name,"My",propeller_torque_in_body_frame_at_G(1))
	WRITE(observer,body_name,name,body_name,"Mz",propeller_torque_in_body_frame_at_G(2))

	WRITE(observer,body_name,name,name,"Fx",tau_propeller_in_internal_frame.X())
	WRITE(observer,body_name,name,name,"Fy",tau_propeller_in_internal_frame.Y())
	WRITE(observer,body_name,name,name,"Fz",tau_propeller_in_internal_frame.Z())
	WRITE(observer,body_name,name,name,"Mx",tau_propeller_in_internal_frame.K())
	WRITE(observer,body_name,name,name,"My",tau_propeller_in_internal_frame.M())
	WRITE(observer,body_name,name,name,"Mz",tau_propeller_in_internal_frame.N())

	WRITE(observer,body_name,name,"NED","Fx",propeller_force_in_ned_frame_at_O(0))
	WRITE(observer,body_name,name,"NED","Fy",propeller_force_in_ned_frame_at_O(1))
	WRITE(observer,body_name,name,"NED","Fz",propeller_force_in_ned_frame_at_O(2))
	WRITE(observer,body_name,name,"NED","Mx",propeller_torque_in_ned_frame_at_O(0))
	WRITE(observer,body_name,name,"NED","My",propeller_torque_in_ned_frame_at_O(1))
	WRITE(observer,body_name,name,"NED","Mz",propeller_torque_in_ned_frame_at_O(2))

    extra_observations(observer);
}

void MMGRudderForceModel::extra_observations(Observer& observer) const
{
    observer.write(pimpl->get_F_N(),DataAddressing({"efforts",get_body_name(),get_name(),"extra_observations","F_N"},std::string("F_N")));
}

