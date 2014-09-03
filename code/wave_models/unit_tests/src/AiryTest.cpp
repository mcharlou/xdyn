/*
 * AiryTest.cpp
 *
 *  Created on: Aug 1, 2014
 *      Author: cady
 */

#include "AiryTest.hpp"
#include "Airy.hpp"
#include "BretschneiderSpectrum.hpp"
#include "Cos2sDirectionalSpreading.hpp"
#include "DiracSpectralDensity.hpp"
#include "DiracDirectionalSpreading.hpp"
#include "SumOfWaveSpectralDensities.hpp"

#include "discretize.hpp"

#define _USE_MATH_DEFINE
#include <cmath>
#define PI M_PI

AiryTest::AiryTest() : a(DataGenerator(54200))
{
}

AiryTest::~AiryTest()
{
}

void AiryTest::SetUp()
{
}

void AiryTest::TearDown()
{
}

#define EPS 1E-10


TEST_F(AiryTest, single_frequency_single_direction_at_one_point)
{
    //! [AiryTest example]
    const double psi0 = PI/4;
    const double Hs = 3;
    const double Tp = 10;
    const double omega0 = 2*PI/Tp;
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = a.random<double>().greater_than(omega_min);
    const size_t nfreq = a.random<size_t>().between(2,100);
    const DiscreteDirectionalWaveSpectrum A = discretize(DiracSpectralDensity(omega0, Hs), DiracDirectionalSpreading(psi0), omega_min, omega_max, nfreq);
    int random_seed = 0;
    const Airy wave(A, random_seed);

    const double x = a.random<double>();
    const double y = a.random<double>();
    const double phi = 3.4482969340598712549;
    const double k = 4.*PI*PI/Tp/Tp/9.81;
    //! [AiryTest example]
    //! [AiryTest expected output]
    for (double t = 0 ; t < 3*Tp ; t+=0.1)
    {
        ASSERT_NEAR(sqrt(2*Hs)*cos(k*(x*cos(psi0)+y*sin(psi0))-2*PI/Tp*t +phi), wave.elevation(x,y,t), 1E-6);
    }
    //! [AiryTest expected output]
}

TEST_F(AiryTest, two_frequencies_single_direction_at_one_point)
{
    //! [AiryTest example]
    const double psi0 = PI/4;
    const double Hs = 3;
    const double Tp = 10;
    const double omega0 = 2*PI/Tp;
    const double omega_min = 0.01;
    const double omega_max = 10;
    const size_t nfreq = 50;
    const DiscreteDirectionalWaveSpectrum A = discretize(DiracSpectralDensity(omega0, Hs)+DiracSpectralDensity(omega0, Hs), DiracDirectionalSpreading(psi0), omega_min, omega_max, nfreq);
    int random_seed = 0;
    const Airy wave(A, random_seed);

    const double x = a.random<double>();
    const double y = a.random<double>();
    const double phi = 3.4482969340598712549;
    const double k = 4.*PI*PI/Tp/Tp/9.81;
    for (double t = 0 ; t < 3*Tp ; t+=0.1)
    {
        ASSERT_NEAR(2*sqrt(Hs)*cos(k*(x*cos(psi0)+y*sin(psi0))-2*PI/Tp*t +phi), wave.elevation(x,y,t), 1E-6);
    }
}

TEST_F(AiryTest, bug)
{
    const double psi0 = PI;
    const double Hs = 2;
    const double Tp = 7;
    //const double omega0 = 2;
    const double s = 1;
    const double omega_min = 0.1;
    const double omega_max = 6;
    const size_t nfreq = 101;
    const DiscreteDirectionalWaveSpectrum A = discretize(BretschneiderSpectrum(Hs, Tp), Cos2sDirectionalSpreading(psi0, s), omega_min, omega_max, nfreq);
    int random_seed = 0;
    const Airy wave(A, random_seed);

    const double x = 0;
    const double y = 0;
    const double t = 0;

    ASSERT_LT(fabs(wave.elevation(x,y,t)), 5);

}
