/*
 * EnvironmentAndFrames.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#include <vector>
#include <string>
#include <ssc/kinematics.hpp>
using namespace ssc::kinematics;

#include "InvalidInputException.hpp"
#include "SurfaceElevationInterface.hpp"
#include "Observer.hpp"
#include "Body.hpp"

#include "EnvironmentAndFrames.hpp"

EnvironmentAndFrames::EnvironmentAndFrames() : w(),
											   wind(),
                                               k(KinematicsPtr(new Kinematics())),
                                               rho(0),
                                               nu(0),
                                               g(0),
											   air_density(0),
                                               rot("angle", {"z", "y'", "x''"})
{ // useless code since this constructor takes no argument
    if (rho<0.0)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "rho can not be negative");
    }
    if (nu<0.0)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "nu can not be negative");
    }
    if (g<0.0)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "g can not be negative");
    }
}

void EnvironmentAndFrames::feed(
        Observer& observer, double t/*,
        const std::vector<BodyPtr>& bodies, const StateType& state*/) const
{
    try
    {
        if (w.get())
        {
            /*for (size_t i = 0 ; i < bodies.size() ; ++i)
            {
                bodies[i]->update_kinematics(state,k);
            }*/
            const auto kk = w->get_waves_on_mesh_as_a_grid(k, t);
            if(kk.z.size()!=0)
            {
                const auto address = DataAddressing({"waveElevation"},"waves");
                observer.write(kk, address);
            }
        }
    }
    catch (const ssc::kinematics::KinematicsException& e)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException,
                "In the YAML file (section [environment models/model/output/frame of reference]): the output reference frame is not defined (" << e.get_message() << ")");
    }
}
