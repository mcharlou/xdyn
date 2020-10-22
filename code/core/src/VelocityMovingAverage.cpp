/*
 * VelocityMovingAverage.cpp
 *
 *  Created on: 19 mai 2020
 *      Author: mcharlou2016
 */

#include "BodyStates.hpp"
#include "History.hpp"
#include "InternalErrorException.hpp"

#include "VelocityMovingAverage.hpp"

VelocityMovingAverage::VelocityMovingAverage(double Tmax_):Tmax(Tmax_),U(Tmax_),V(Tmax_),W(Tmax_),P(Tmax_),Q(Tmax_),R(Tmax_){}

void VelocityMovingAverage::update(const BodyStates& states)
{
	double t=states.u.get_current_time();
	double alpha;
	if(states.u.size()>1)
	{
		double dt=states.u.get_last_time_step();
		alpha = 1-exp(-dt/Tmax);
	}
	else alpha = 1;

	U.record(t,alpha*states.u()+(1-alpha)*U());
	V.record(t,alpha*states.v()+(1-alpha)*V());
	W.record(t,alpha*states.w()+(1-alpha)*W());
	P.record(t,alpha*states.p()+(1-alpha)*P());
	Q.record(t,alpha*states.q()+(1-alpha)*Q());
	R.record(t,alpha*states.r()+(1-alpha)*R());
}

void VelocityMovingAverage::update(double t, double u_, double v_, double w_, double p_, double q_, double r_)
{
	double alpha=0;
	if(U.size()>0 && Tmax>0)
	{
		double dt = t - U.get_current_time();
		alpha = 1-exp(-dt/Tmax);
	}
	else
	{
		alpha = 1;
	}
	U.record(t,alpha*u_+(1-alpha)*U());
	V.record(t,alpha*v_+(1-alpha)*V());
	W.record(t,alpha*w_+(1-alpha)*W());
	P.record(t,alpha*p_+(1-alpha)*P());
	Q.record(t,alpha*q_+(1-alpha)*Q());
	R.record(t,alpha*r_+(1-alpha)*R());
}

void VelocityMovingAverage::reset()
{
	U.reset();
	V.reset();
	W.reset();
	P.reset();
	Q.reset();
	R.reset();
}

void VelocityMovingAverage::reset(const BodyStates& states)
{
	double alpha;
	double dt;
	U.reset();
	U.record(states.u[0].first,states.u[0].second);
	for(int i=1;i<(int)states.u.size();i++)
	{
		dt = states.u[i].first - states.u[i-1].first;
		alpha = 1-exp(-dt/Tmax);
		U.record(states.u[i].first,alpha*states.u[i].second+(1-alpha)*U());
	}
	V.reset();
	V.record(states.v[0].first,states.v[0].second);
	for(int i=1;i<(int)states.v.size();i++)
	{
		dt = states.v[i].first - states.v[i-1].first;
		alpha = 1-exp(-dt/Tmax);
		V.record(states.v[i].first,alpha*states.v[i].second+(1-alpha)*V());
	}
	W.reset();
	W.record(states.w[0].first,states.w[0].second);
	for(int i=1;i<(int)states.w.size();i++)
	{
		dt = states.w[i].first - states.w[i-1].first;
		alpha = 1-exp(-dt/Tmax);
		W.record(states.w[i].first,alpha*states.w[i].second+(1-alpha)*W());
	}
	P.reset();
	P.record(states.p[0].first,states.p[0].second);
	for(int i=1;i<(int)states.p.size();i++)
	{
		dt = states.p[i].first - states.p[i-1].first;
		alpha = 1-exp(-dt/Tmax);
		P.record(states.p[i].first,alpha*states.p[i].second+(1-alpha)*P());
	}
	Q.reset();
	Q.record(states.q[0].first,states.q[0].second);
	for(int i=1;i<(int)states.q.size();i++)
	{
		dt = states.q[i].first - states.q[i-1].first;
		alpha = 1-exp(-dt/Tmax);
		Q.record(states.q[i].first,alpha*states.q[i].second+(1-alpha)*Q());
	}
	R.reset();
	R.record(states.r[0].first,states.r[0].second);
	for(int i=1;i<(int)states.r.size();i++)
	{
		dt = states.r[i].first - states.r[i-1].first;
		alpha = 1-exp(-dt/Tmax);
		R.record(states.r[i].first,alpha*states.r[i].second+(1-alpha)*R());
	}
}

const History& VelocityMovingAverage::get(int idx) const
{
	switch(idx)
	{
		case 0:
			return U;
		case 1:
			return V;
		case 2:
			return W;
		case 3:
			return P;
		case 4:
			return Q;
		case 5:
			return R;
		default:
			THROW(__PRETTY_FUNCTION__, InternalErrorException, std::string("Requested speed from body states is out of bounds [0,5]"));
			return U;
	}
}

Eigen::Matrix<double,6,1> VelocityMovingAverage::get_vector() const
{
	Eigen::Matrix<double,6,1> ret;
	ret << U(),V(),W(),P(),Q(),R();
	return ret;
}

Eigen::Vector3d VelocityMovingAverage::get_speed() const
{
	Eigen::Vector3d ret;
	ret << U(),V(),W();
	return ret;
}

