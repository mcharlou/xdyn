/*
 * HDBParserForTests.cpp
 *
 *  Created on: Dec 2, 2014
 *      Author: cady
 */

#include "HDBParserForTests.hpp"

HDBParserForTests::HDBParserForTests(const std::vector<double>& omega_, const std::vector<double>& Br_) : omega(omega_), Br(Br_)
{
}

std::vector<double> HDBParserForTests::get_radiation_damping_angular_frequencies() const
{
    return omega;
}

std::vector<double> HDBParserForTests::get_radiation_damping_coeff(const size_t i, const size_t j) const
{
    if (i==j) return Br;
              return std::vector<double>(Br.size(),0);
}
