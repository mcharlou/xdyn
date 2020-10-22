/*
 * Sim.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */

#include <string>
#include <vector>

#include "Observer.hpp"
#include "Sim.hpp"
#include "update_kinematics.hpp"
#include "SurfaceElevationInterface.hpp"
#include "YamlWaveModelInput.hpp"
#include "InternalErrorException.hpp"
#include "AbstractController.hpp"

#include <ssc/kinematics.hpp>
#include <ssc/numeric.hpp>

#define SQUARE(x) ((x)*(x))

class Sim::Impl
{
    public:
        Impl(const std::vector<BodyPtr>& bodies_,
             //const std::vector<ListOfForces>& forces_,
             //const std::vector<ListOfControlledForces>& controlled_forces_,
             const EnvironmentAndFrames& env_,
             const StateType& x,
             const ssc::data_source::DataSource& command_listener_) :
                 bodies(bodies_), name2bodyptr(), /*forces(), controlled_forces(),*/ env(env_),
                 _dx_dt(StateType(x.size(),0)), command_listener(command_listener_)/*, sum_of_forces_in_body_frame(),
                 sum_of_forces_in_NED_frame(), coriolis_and_centripetal_in_body_frame(), coriolis_and_centripetal_in_NED_frame()*/
        {
            //size_t i = 0;
            for (auto body:bodies)
            {
                //forces[body->get_name()] = forces_.at(i++);
                //controlled_forces[body->get_name()] = controlled_forces_.at(i++);
                name2bodyptr[body->get_name()] = body;
            }
        }

        /*void feed_sum_of_forces(Observer& observer, const std::string& body_name)
        {
            feed_sum_of_forces(observer, sum_of_forces_in_body_frame[body_name], body_name, body_name);
            feed_sum_of_forces(observer, sum_of_forces_in_NED_frame[body_name], body_name, "NED");
            feed_coriolis_and_centripetal_forces(observer, body_name);
        }

        void feed_sum_of_forces(Observer& observer, ssc::kinematics::UnsafeWrench& W, const std::string& body_name, const std::string& frame)
        {
            observer.write(W.X(),DataAddressing({"efforts",body_name,"sum of forces",frame,"Fx"},std::string("Fx(sum of forces,")+body_name+","+frame+")"));
            observer.write(W.Y(),DataAddressing({"efforts",body_name,"sum of forces",frame,"Fy"},std::string("Fy(sum of forces,")+body_name+","+frame+")"));
            observer.write(W.Z(),DataAddressing({"efforts",body_name,"sum of forces",frame,"Fz"},std::string("Fz(sum of forces,")+body_name+","+frame+")"));
            observer.write(W.K(),DataAddressing({"efforts",body_name,"sum of forces",frame,"Mx"},std::string("Mx(sum of forces,")+body_name+","+frame+")"));
            observer.write(W.M(),DataAddressing({"efforts",body_name,"sum of forces",frame,"My"},std::string("My(sum of forces,")+body_name+","+frame+")"));
            observer.write(W.N(),DataAddressing({"efforts",body_name,"sum of forces",frame,"Mz"},std::string("Mz(sum of forces,")+body_name+","+frame+")"));
        }

        void feed_coriolis_and_centripetal_forces(Observer& observer, const std::string& body_name)
        {
        	observer.write(coriolis_and_centripetal_in_body_frame[body_name].X(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces",body_name,"Fx"},std::string("Fx(coriolis and centripetal forces,")+body_name+","+body_name+")"));
        	observer.write(coriolis_and_centripetal_in_body_frame[body_name].Y(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces",body_name,"Fy"},std::string("Fy(coriolis and centripetal forces,")+body_name+","+body_name+")"));
        	observer.write(coriolis_and_centripetal_in_body_frame[body_name].Z(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces",body_name,"Fz"},std::string("Fz(coriolis and centripetal forces,")+body_name+","+body_name+")"));
        	observer.write(coriolis_and_centripetal_in_body_frame[body_name].K(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces",body_name,"Mx"},std::string("Mx(coriolis and centripetal forces,")+body_name+","+body_name+")"));
        	observer.write(coriolis_and_centripetal_in_body_frame[body_name].M(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces",body_name,"My"},std::string("My(coriolis and centripetal forces,")+body_name+","+body_name+")"));
        	observer.write(coriolis_and_centripetal_in_body_frame[body_name].N(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces",body_name,"Mz"},std::string("Mz(coriolis and centripetal forces,")+body_name+","+body_name+")"));

        	observer.write(coriolis_and_centripetal_in_NED_frame[body_name].X(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces","NED","Fx"},std::string("Fx(coriolis and centripetal forces,")+body_name+","+"NED"+")"));
        	observer.write(coriolis_and_centripetal_in_NED_frame[body_name].Y(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces","NED","Fy"},std::string("Fy(coriolis and centripetal forces,")+body_name+","+"NED"+")"));
        	observer.write(coriolis_and_centripetal_in_NED_frame[body_name].Z(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces","NED","Fz"},std::string("Fz(coriolis and centripetal forces,")+body_name+","+"NED"+")"));
        	observer.write(coriolis_and_centripetal_in_NED_frame[body_name].K(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces","NED","Mx"},std::string("Mx(coriolis and centripetal forces,")+body_name+","+"NED"+")"));
        	observer.write(coriolis_and_centripetal_in_NED_frame[body_name].M(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces","NED","My"},std::string("My(coriolis and centripetal forces,")+body_name+","+"NED"+")"));
        	observer.write(coriolis_and_centripetal_in_NED_frame[body_name].N(),DataAddressing({"efforts",body_name,"coriolis and centripetal forces","NED","Mz"},std::string("Mz(coriolis and centripetal forces,")+body_name+","+"NED"+")"));
        }*/

        void feed_commands_and_signals(Observer& observer)
        {
        	auto signals = command_listener.get_all<double>();
        	for(auto sig:signals)
        	{
        		if(sig.first.substr(0,9)!="NoOutput_" && sig.first!="t")
        		{
        			observer.write(sig.second,DataAddressing({"signals",sig.first},sig.first));
        		}
        	}
        }

        std::vector<BodyPtr> bodies;
        std::map<std::string,BodyPtr> name2bodyptr;
        //std::map<std::string,std::vector<ForcePtr> > forces;
        //std::map<std::string,std::vector<ControllableForcePtr> > controlled_forces;
        EnvironmentAndFrames env;
        StateType _dx_dt;
        ssc::data_source::DataSource command_listener;
        /*std::map<std::string,ssc::kinematics::UnsafeWrench> sum_of_forces_in_body_frame;
        std::map<std::string,ssc::kinematics::UnsafeWrench> sum_of_forces_in_NED_frame;
        std::map<std::string,ssc::kinematics::UnsafeWrench> coriolis_and_centripetal_in_body_frame;
        std::map<std::string,ssc::kinematics::UnsafeWrench> coriolis_and_centripetal_in_NED_frame;*/
};

/*std::map<std::string,std::vector<ForcePtr> > Sim::get_forces() const
{
    return pimpl->forces;
}*/

std::vector<BodyPtr> Sim::get_bodies() const
{
    return pimpl->bodies;
}

EnvironmentAndFrames Sim::get_env() const
{
    return pimpl->env;
}

Sim::Sim(const std::vector<BodyPtr>& bodies,
         //const std::vector<ListOfForces>& forces,
         //const std::vector<ListOfControlledForces>& controlled_forces,
         const EnvironmentAndFrames& env,
         const StateType& x,
         const ssc::data_source::DataSource& command_listener) : state(x), pimpl(new Impl(bodies, /*forces,*/ env, x, command_listener))
{
}

StateType Sim::normalize_quaternions(const StateType& all_states
                                    ) const
{
    StateType normalized = all_states;
    for (size_t i = 0 ; i < pimpl->bodies.size() ; ++i)
    {
        const auto norm = std::hypot(std::hypot(std::hypot(*_QR(normalized,i),*_QI(normalized,i)),*_QJ(normalized,i)),*_QK(normalized,i));
        if (not almost_equal(norm,1.0))
        {
            *_QR(normalized,i) /= norm;
            *_QI(normalized,i) /= norm;
            *_QJ(normalized,i) /= norm;
            *_QK(normalized,i) /= norm;
        }
    }
    return normalized;
}

void Sim::operator()(const StateType& x, StateType& dxdt, double t)
{
    dx_dt(x, dxdt, t);
    state = normalize_quaternions(x);
    pimpl->_dx_dt = dxdt;
}

void Sim::dx_dt(const StateType& x, StateType& dxdt, const double t)
{
    for (auto body: pimpl->bodies)
    {
        body->calculate_state_derivatives(x, dxdt, t, pimpl->env, pimpl->command_listener);
    }
}

void Sim::update_forces(const double t)
{
	for (auto body: pimpl->bodies)
	{
		body->update(pimpl->env,state,t);
		//sum_of_forces(state, body, t);
	}
}

void Sim::update_discrete_states()
{
}

void Sim::update_continuous_states()
{
}

/*ssc::kinematics::UnsafeWrench Sim::sum_of_forces(const StateType& x, const BodyPtr& body, const double t)
{
    const auto states = body->get_states();
    pimpl->coriolis_and_centripetal_in_body_frame[body->get_name()] = ssc::kinematics::UnsafeWrench(body->get_fictitious_forces(x));
    //std::cout << "Fictitious forces: " << pimpl->coriolis_and_centripetal_in_body_frame[body->get_name()] << std::endl;
    pimpl->sum_of_forces_in_body_frame[body->get_name()] = pimpl->coriolis_and_centripetal_in_body_frame[body->get_name()];
    const auto forces = pimpl->forces[body->get_name()];
    for (auto force:forces)
    {
        //force->update(states, t, pimpl->env.k);
        const ssc::kinematics::Wrench tau = force->operator()(states, t, pimpl->env.k, pimpl->command_listener);
        std::cout << "Adding force '" << force->get_name() << "': " << tau << std::endl;
        pimpl->sum_of_forces_in_body_frame[body->get_name()] += tau;
        if (tau.get_frame() != body->get_name())
        {
        	std::cout << "Change of frame for force '" << force->get_name() << "' provided in frame '" << tau.get_frame() << "' by the force model." << std::endl;
            const ssc::kinematics::Transform T = pimpl->env.k->get(tau.get_frame(), body->get_name());
            const auto t = tau.change_frame_but_keep_ref_point(T);
            const ssc::kinematics::UnsafeWrench tau_body(states.G, t.force, t.torque + (t.get_point()-states.G).cross(t.force));
            pimpl->sum_of_forces_in_body_frame[body->get_name()] += tau_body;
        }
        else
        {
            pimpl->sum_of_forces_in_body_frame[body->get_name()] += tau;
        }
    }
    const auto controlled_forces = pimpl->controlled_forces[body->get_name()];
    for (auto force:controlled_forces)
    {
        const ssc::kinematics::Wrench tau = force->operator()(states, t, pimpl->env.k, pimpl->command_listener);
    	std::cout << "Adding force '" << force->get_name() << "': " << tau << std::endl;
        pimpl->sum_of_forces_in_body_frame[body->get_name()] += tau;
    }
    pimpl->sum_of_forces_in_NED_frame[body->get_name()] = ForceModel::project_into_NED_frame(pimpl->sum_of_forces_in_body_frame[body->get_name()],states.get_rot_from_ned_to_body());
    pimpl->coriolis_and_centripetal_in_NED_frame[body->get_name()] = ForceModel::project_into_NED_frame(pimpl->coriolis_and_centripetal_in_body_frame[body->get_name()],states.get_rot_from_ned_to_body());
    return pimpl->sum_of_forces_in_body_frame[body->get_name()];
}*/

ssc::kinematics::PointMatrix Sim::get_waves(const double t//!< Current instant
                                                  ) const
{
    try
    {
        if (pimpl->env.w.get())
        {
            for (size_t i = 0 ; i < pimpl->bodies.size() ; ++i)
            {
                pimpl->bodies[i]->update_kinematics(state,pimpl->env.k);
            }
            return pimpl->env.w->get_waves_on_mesh(pimpl->env.k, t);
        }
    }
    catch (const ssc::kinematics::KinematicsException& e)
    {
        THROW(__PRETTY_FUNCTION__, InternalErrorException, "Error when calculating waves on mesh: the output reference frame does not exist (" << e.get_message() << ")");
    }
    return ssc::kinematics::PointMatrix("NED",0);
}

void Sim::output(const StateType& x, Observer& obs, const double t) const
{
    StateType x_with_forced_states;
    for (auto body: pimpl->bodies)
    {
        x_with_forced_states = body->block_states_if_necessary(x,t);
    }
    const auto normalized_x = normalize_quaternions(x_with_forced_states);
    /*for (auto forces:pimpl->forces)
    {
        for (auto force:forces.second) force->feed(obs);
    }*/
    /*for (auto controlled_forces:pimpl->controlled_forces)
    {
        for (auto force:controlled_forces.second)
        {
            const auto body_name = controlled_forces.first;
            const auto body = pimpl->name2bodyptr[body_name];
            const auto G = body->get_states().G;
            force->feed(obs,pimpl->env.k,G);
        }
    }*/
    for (auto body:pimpl->bodies)
    {
    	body->output(normalized_x, pimpl->_dx_dt, obs, pimpl->env.rot);
        /*auto dF = body->get_delta_F(pimpl->_dx_dt,pimpl->sum_of_forces_in_body_frame[body->get_name()]);
        obs.write((double)dF(0),DataAddressing({"efforts",body->get_name(),"blocked states",body->get_name(),"Fx"},std::string("Fx(blocked states,")+body->get_name()+","+body->get_name()+")"));
        obs.write((double)dF(1),DataAddressing({"efforts",body->get_name(),"blocked states",body->get_name(),"Fy"},std::string("Fy(blocked states,")+body->get_name()+","+body->get_name()+")"));
        obs.write((double)dF(2),DataAddressing({"efforts",body->get_name(),"blocked states",body->get_name(),"Fz"},std::string("Fz(blocked states,")+body->get_name()+","+body->get_name()+")"));
        obs.write((double)dF(3),DataAddressing({"efforts",body->get_name(),"blocked states",body->get_name(),"Mx"},std::string("Mx(blocked states,")+body->get_name()+","+body->get_name()+")"));
        obs.write((double)dF(4),DataAddressing({"efforts",body->get_name(),"blocked states",body->get_name(),"My"},std::string("My(blocked states,")+body->get_name()+","+body->get_name()+")"));
        obs.write((double)dF(5),DataAddressing({"efforts",body->get_name(),"blocked states",body->get_name(),"Mz"},std::string("Mz(blocked states,")+body->get_name()+","+body->get_name()+")"));
        auto U_EMA = body->get_states().low_frequency_velocity.get_vector();
        obs.write((double)U_EMA(0),DataAddressing({"states",body->get_name(),"EMA(U)"},std::string("u_EMA(")+body->get_name()+")"));
        obs.write((double)U_EMA(1),DataAddressing({"states",body->get_name(),"EMA(V)"},std::string("v_EMA(")+body->get_name()+")"));
        obs.write((double)U_EMA(2),DataAddressing({"states",body->get_name(),"EMA(W)"},std::string("w_EMA(")+body->get_name()+")"));
        obs.write((double)U_EMA(3),DataAddressing({"states",body->get_name(),"EMA(P)"},std::string("p_EMA(")+body->get_name()+")"));
        obs.write((double)U_EMA(4),DataAddressing({"states",body->get_name(),"EMA(Q)"},std::string("q_EMA(")+body->get_name()+")"));
        obs.write((double)U_EMA(5),DataAddressing({"states",body->get_name(),"EMA(R)"},std::string("r_EMA(")+body->get_name()+")"));*/
    }
    pimpl->env.feed(obs, t/*, pimpl->bodies, normalized_x*/);
    /*for (auto body:pimpl->bodies)
    {
        pimpl->feed_sum_of_forces(obs, body->get_name());
    }*/
    pimpl->feed_commands_and_signals(obs);
}

void Sim::set_bodystates(const std::vector<State>& states)
{
    if(states.size()!=1 or pimpl->bodies.size()!=1)
    {
        THROW(__PRETTY_FUNCTION__, InternalErrorException, "'states' size must be 1");
    }
    pimpl->bodies.at(0)->set_states_history(states.at(0));
    if (not(states.at(0).x.is_empty()))
    {
        state = states.at(0).get_StateType(states.at(0).x.size()-1);
    }
    // Calling full_update on all forces
    /*for (auto force:pimpl->forces[pimpl->bodies.at(0)->get_name()])
    {
    	force->full_update(pimpl->bodies.at(0)->get_states(),pimpl->env.k);
    }
    for (auto force:pimpl->controlled_forces[pimpl->bodies.at(0)->get_name()])
    {
    	force->full_update(pimpl->bodies.at(0)->get_states(),pimpl->env.k);
    }*/
}

void Sim::set_command_listener(const std::map<std::string, double>& new_commands)
{
    for(const auto c : new_commands)
    {
    	pimpl->command_listener.set(c.first, c.second);
    	/*try
    	{
    		pimpl->command_listener.set(c.first, c.second);
    	}
    	catch(ssc::data_source::DataSourceException&)
    	{
    		pimpl->command_listener.force(c.first, c.second);
    	}*/
    }
}

void Sim::force_commands(const std::map<std::string, double>& commands)
{
    for(const auto c : commands)
    {
    	pimpl->command_listener.force(c.first, c.second);
    }
}

void Sim::reset_history()
{
    for (auto body:pimpl->bodies)
    {
        body->reset_history();
    }
}
