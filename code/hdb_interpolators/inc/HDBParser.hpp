/*
 * HDBParser.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: cady
 */

#ifndef HDBPARSER_HPP_
#define HDBPARSER_HPP_

#include <Eigen/Dense>

#include <ssc/macros.hpp>
#include <memory>

#include "TimestampedMatrix.hpp"

/** \brief
 *  \details
 *  \addtogroup hdb_interpolators
 *  \ingroup hdb_interpolators
 *  \section ex1 Example
 *  \snippet hdb_interpolators/unit_tests/src/HDBParserTest.cpp HDBDataTest example
 *  \section ex2 Expected output
 *  \snippet hdb_interpolators/unit_tests/src/HDBParserTest.cpp HDBDataTest expected output
 */
class HDBParser
{
    public:
        HDBParser(const std::string& data);
        virtual ~HDBParser();
        TimestampedMatrices get_added_mass_array() const;
        TimestampedMatrices get_radiation_damping_array() const;
        RAOData get_diffraction_module() const;
        RAOData get_diffraction_phase() const;
        Eigen::Matrix<double,6,6> get_added_mass() const;
        Eigen::Matrix<double,6,6> get_added_mass(const double Tp //!< Period at which to interpolate the added mass
                                                ) const; // const doesn't really mean anything here as the members are hidden inside a pimpl
        double get_infinite_frequency_added_mass_coeff(const size_t i, const size_t j, std::function<double(double)>& K, double Tmin, double Tmax) const;
        virtual std::vector<double> get_radiation_damping_angular_frequencies() const;
        virtual std::vector<double> get_added_mass_coeff(const size_t i, const size_t j) const;
        virtual std::vector<double> get_radiation_damping_coeff(const size_t i, const size_t j) const;
        std::array<std::vector<std::vector<double> >,6 > get_diffraction_module_tables() const;
        std::array<std::vector<std::vector<double> >,6 > get_diffraction_phase_tables() const;
        std::array<std::vector<std::vector<double> >,6 > get_froude_krylov_module_tables() const;
        std::array<std::vector<std::vector<double> >,6 > get_froude_krylov_phase_tables() const;
        std::vector<double> get_diffraction_phase_psis() const;
        std::vector<double> get_diffraction_phase_periods() const;
        std::vector<double> get_diffraction_module_psis() const;
        std::vector<double> get_diffraction_module_periods() const;
        std::vector<double> get_froude_krylov_phase_psis() const;
        std::vector<double> get_froude_krylov_phase_periods() const;
        std::vector<double> get_froude_krylov_module_psis() const;
        std::vector<double> get_froude_krylov_module_periods() const;

    protected:
        HDBParser();

    private:
        class Impl;
        std::shared_ptr<Impl> pimpl;
};

#endif /* HDBPARSER_HPP_ */
