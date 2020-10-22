/*
 * GRPCForceModel.hpp
 *
 *  Created on: Jun 17, 2019
 *      Author: cady
 */

#ifndef GRPC_INC_GRPCFORCEMODEL_HPP_
#define GRPC_INC_GRPCFORCEMODEL_HPP_


#include "EnvironmentAndFrames.hpp"
#include "ForceModel.hpp"
#include <ssc/kinematics.hpp>

class GRPCForceModel : public ForceModel
{
    public:
        struct Input
        {
            std::string url;
            std::string name;
            std::string yaml;
        };
        GRPCForceModel(const Input& input, const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static Input parse(const std::string& yaml);
        static std::string model_name();
        double get_Tmax() const;

    private:
        void extra_observations(Observer& observer) const;
        GRPCForceModel(); // Disabled
        class Impl;
        std::shared_ptr<Impl> pimpl;
        GRPCForceModel(const std::shared_ptr<Impl>& pimpl, const std::string& body_name, const EnvironmentAndFrames& env);

};


#endif /* GRPC_INC_GRPCFORCEMODEL_HPP_ */
