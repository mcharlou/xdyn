/*
 * WaveModel.hpp
 *
 *  Created on: Aug 1, 2014
 *      Author: cady
 */

#ifndef WAVEMODEL_HPP_
#define WAVEMODEL_HPP_

#include "DiscreteDirectionalWaveSpectrum.hpp"

#include <ssc/kinematics.hpp>
#include <ssc/macros.hpp>
#include <memory>

/** \author cec
 *  \date Aug 1, 2014, 3:15:04 PM
 *  \brief Interface to wave models.
 *  \details Should at least provide elevation.
 *  \ingroup wave_models
 *  \section ex1 Example
 *  \snippet environment_models/unit_tests/src/WaveModelTest.cpp WaveModelTest example
 *  \section ex2 Expected output
 *  \snippet environment_models/unit_tests/src/WaveModelTest.cpp WaveModelTest expected output
 */
class WaveModel
{
    public:
        WaveModel(const DiscreteDirectionalWaveSpectrum& spectrum, const double constant_random_phase);
        WaveModel(const DiscreteDirectionalWaveSpectrum& spectrum, const int random_number_generator_seed);
        virtual ~WaveModel();

        /**  \brief Calculate radiation forces using first order force RAO
          *  \returns Force (or torque), depending on the RAO
          */
        virtual double evaluate_rao(const double x, //!< x-position of the RAO's calculation point in the NED frame (in meters)
                                    const double y, //!< y-position of the RAO's calculation point in the NED frame (in meters)
                                    const double t, //!< Current time instant (in seconds)
                                    const std::vector<double>& rao_module, //!< Module of the RAO
                                    const std::vector<double>& rao_phase //!< Phase of the RAO
                                     ) const = 0;

        /**  \brief Computes the surface elevations at given points.
          *  \returns Elevations of a list of points at a given instant, in meters.
          *  \snippet environment_models/unit_tests/src/WaveModelTest.cpp WaveModelTest method_example
          */
        std::vector<double> get_elevation(const std::vector<double> &x, //!< x-positions in the NED frame (in meters)
                                          const std::vector<double> &y, //!< y-positions in the NED frame (in meters)
                                          const double t                //!< Current time instant (in seconds)
                                         ) const;

        /**  \brief Computes the orbital velocity at given points.
          *  \returns Velocities of the fluid at given points & instant, in m/s
          */
        ssc::kinematics::PointMatrix get_orbital_velocity(const double g,                //!< gravity (in m/s^2)
                                                          const std::vector<double>& x,  //!< x-positions in the NED frame (in meters)
                                                          const std::vector<double>& y,  //!< y-positions in the NED frame (in meters)
                                                          const std::vector<double>& z,  //!< z-positions in the NED frame (in meters)
                                                          const double t,                //!< Current time instant (in seconds)
                                                          const std::vector<double>& eta //!< Wave heights at x,y,t (in meters)
                                                         ) const;

        /**  \brief Computes the dynamic pressure at a given point.
          *  \returns Pressure (in Pa) induced by the waves, at given points in the fluid
          *  \snippet environment_models/unit_tests/src/AiryTest.cpp AiryTest elevation_example
          */
        std::vector<double> get_dynamic_pressure(const double rho,               //!< water density (in kg/m^3)
                                                 const double g,                 //!< gravity (in m/s^2)
                                                 const std::vector<double> &x,   //!< x-positions in the NED frame (in meters)
                                                 const std::vector<double> &y,   //!< y-positions in the NED frame (in meters)
                                                 const std::vector<double> &z,   //!< z-positions in the NED frame (in meters)
                                                 const std::vector<double> &eta, //!< Wave elevations at (x,y) in the NED frame (in meters)
                                                 const double t                  //!< Current time instant (in seconds)
                                                ) const;

        /**  \returns List of angular frequencies for which the spectra will be calculated.
          *  \details Needed by the RAOs (RadiationForceModel)
          */
        std::vector<double> get_omegas() const;

        /**  \returns List of incidences for which the spectra will be calculated.
          *  \details Needed by the RAOs (RadiationForceModel)
          */
        std::vector<double> get_psis() const;

        FlatDiscreteDirectionalWaveSpectrum get_flat_spectrum() const {return flat_spectrum;};
        DiscreteDirectionalWaveSpectrum get_spectrum() const {return spectrum;};

    private:
        WaveModel(); // Disabled
        void check_sizes() const;

        /**  \author cec
          *  \date Aug 1, 2014, 3:24:45 PM
          *  \brief Surface elevation
          *  \returns Elevations of a list of points at a given instant, in meters.
          *  \snippet environment_models/unit_tests/src/WaveModelTest.cpp WaveModelTest method_example
          */
        virtual std::vector<double> elevation(const std::vector<double> &x, //!< x-positions in the NED frame (in meters)
                                              const std::vector<double> &y, //!< y-positions in the NED frame (in meters)
                                              const double t                //!< Current time instant (in seconds)
                                              ) const = 0;

        /**  \author cec
          *  \date Feb 3, 2015, 10:06:45 AM
          *  \brief Orbital velocity
          *  \returns Velocities of the fluid at given points & instant, in m/s
          */
        virtual ssc::kinematics::PointMatrix orbital_velocity(const double g,                 //!< gravity (in m/s^2)
                                                              const std::vector<double>& x,   //!< x-positions in the NED frame (in meters)
                                                              const std::vector<double>& y,   //!< y-positions in the NED frame (in meters)
                                                              const std::vector<double>& z,   //!< z-positions in the NED frame (in meters)
                                                              const double t,                 //!< Current time instant (in seconds)
                                                              const std::vector<double>& eta  //!< Wave heights at x,y,t (in meters)
                                                             ) const = 0;

        /**  \brief Pressure induced by waves
          *  \returns Pressure (in Pa) induced by the waves, at given points in the fluid
          *  \see "Environmental Conditions and Environmental Loads", April 2014, DNV-RP-C205, Det Norske Veritas AS, page 47
          *  \see "Sea Loads on Ships and Offshore Structures", 1990, O.M. Faltinsen, Cambridge Ocean Technology Series, page 16
          *  \see "Hydrodynamique navale : théorie et modèles", 2009, Alain Bovis, Les Presses de l'ENSTA, equation IV.20, page 125
          *  \snippet environment_models/unit_tests/src/AiryTest.cpp AiryTest elevation_example
          */
        virtual std::vector<double> dynamic_pressure(const double rho,               //!< water density (in kg/m^3)
                                                     const double g,                 //!< gravity (in m/s^2)
                                                     const std::vector<double> &x,   //!< x-positions in the NED frame (in meters)
                                                     const std::vector<double> &y,   //!< y-positions in the NED frame (in meters)
                                                     const std::vector<double> &z,   //!< z-positions in the NED frame (in meters)
                                                     const std::vector<double> &eta, //!< Wave elevations at (x,y) in the NED frame (in meters)
                                                     const double t                  //!< Current time instant (in seconds)
                                                    ) const = 0;

    protected:
        DiscreteDirectionalWaveSpectrum spectrum;
        FlatDiscreteDirectionalWaveSpectrum flat_spectrum;
};

typedef std::shared_ptr<WaveModel> WaveModelPtr;

#endif /* WAVEMODEL_HPP_ */
