/*
 * HDBData.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: cady
 */

#ifndef HDBDATA_HPP_
#define HDBDATA_HPP_

#include <ssc/macros.hpp>
#include TR1INC(memory)

#include <array>

#include <Eigen/Dense>

#include "TimestampedMatrix.hpp"

class HDBParser;

/** \brief
 *  \details
 *  \addtogroup hdb_interpolators
 *  \ingroup hdb_interpolators
 *  \section ex1 Example
 *  \snippet hdb_interpolators/unit_tests/src/HDBDataTest.cpp HDBDataTest example
 *  \section ex2 Expected output
 *  \snippet hdb_interpolators/unit_tests/src/HDBDataTest.cpp HDBDataTest expected output
 */
class HDBData
{
    public:
        HDBData(const HDBParser& builder);
        Eigen::Matrix<double,6,6> get_added_mass() const;
        Eigen::Matrix<double,6,6> get_added_mass(const double Tp //!< Period at which to interpolate the added mass
                                                ) const; // const doesn't really mean anything here as the members are hidden inside a pimpl
        std::vector<double> get_radiation_damping_angular_frequencies() const;
        std::vector<double> get_radiation_damping_coeff(const size_t i, const size_t j) const;
        std::array<std::vector<std::vector<double> >,6 > get_diffraction_module_tables() const;
        std::array<std::vector<std::vector<double> >,6 > get_diffraction_phase_tables() const;
        std::vector<double> get_diffraction_phase_psis() const;
        std::vector<double> get_diffraction_phase_omegas() const;
        std::vector<double> get_diffraction_module_psis() const;
        std::vector<double> get_diffraction_module_omegas() const;

    private:
        HDBData();
        class Impl;
        TR1(shared_ptr)<Impl> pimpl;
};

#endif /* HDBDATA_HPP_ */