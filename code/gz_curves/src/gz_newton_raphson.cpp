/*
 * gz_newton_raphson.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: cady
 */

#include "gz_newton_raphson.hpp"

double delta(const Eigen::Vector3d& X1, const Eigen::Vector3d& X2);
double delta(const Eigen::Vector3d& X1, const Eigen::Vector3d& X2)
{
    return (X1-X2).array().abs().maxCoeff();
}

GZ::State GZ::newton_raphson(const GZ::State& X0, //!< Initial value
                         const GZ::FType& f,      //!< Function calculating the sum of forces
                         const GZ::KComputer& K,  //!< Function calculating the stiffness matrix (f')
                         const size_t max_it,     //!< Maximum number of iterations
                         const double eps         //!< Desired precision
                              )
{
    Eigen::Vector3d Xn = X0;
    Eigen::Vector3d Xn_1 = X0.array() + 2*eps;
    for (size_t i = 0 ; i < max_it ; ++i)
    {
        if (delta(Xn,Xn_1) < eps) return Xn;
        Xn_1 = Xn;
        Xn = Xn_1 - K(Xn_1).lu().solve(f(Xn_1).state);
    }
    return Xn;
}