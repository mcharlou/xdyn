/*
 * DirectionalWaveSpectrumTest.cpp
 *
 *  Created on: Jul 31, 2014
 *      Author: cady
 */

#define _USE_MATH_DEFINE
#include <cmath>
#define PI M_PI

#include "discretizeTest.hpp"
#include "discretize.hpp"
#include "JonswapSpectrum.hpp"
#include "Cos2sDirectionalSpreading.hpp"
#include "DiracSpectralDensity.hpp"
#include "DiracDirectionalSpreading.hpp"
#include "InvalidInputException.hpp"
#include "Stretching.hpp"
#include "YamlWaveModelInput.hpp"

discretizeTest::discretizeTest() : a(ssc::random_data_generator::DataGenerator(8421))
{
}

discretizeTest::~discretizeTest()
{
}

void discretizeTest::SetUp()
{
}

void discretizeTest::TearDown()
{
}

TEST_F(discretizeTest, example)
{
//! [discretizeTest example]
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const Cos2sDirectionalSpreading D(PI/4, 2);
    YamlStretching y;
    const Stretching s(y);
    const DiscreteDirectionalWaveSpectrum A = discretize(S, D, 0.01, 3, 1000, s);
//! [discretizeTest example]
//! [discretizeTest expected output]
    ASSERT_EQ(1000, A.Dj.size());
    ASSERT_EQ(1000, A.Si.size());
    ASSERT_DOUBLE_EQ(2.99/999., A.domega);
    ASSERT_DOUBLE_EQ(0.002*PI, A.dpsi);
//! [discretizeTest expected output]
}

TEST_F(discretizeTest, Dirac_in_frequency)
{
    const double omega0 = 0.2;
    const double Hs = 3;
    const DiracSpectralDensity S(omega0, Hs);
    const Cos2sDirectionalSpreading D(PI/4, 2);
    YamlStretching y;
    const Stretching s(y);
    const DiscreteDirectionalWaveSpectrum A = discretize(S, D, 0.01, 3, 1000, s);
    ASSERT_EQ(1000, A.Dj.size());
    ASSERT_EQ(1, A.Si.size());
    ASSERT_DOUBLE_EQ(1, A.domega);
    ASSERT_DOUBLE_EQ(0.002*PI, A.dpsi);
    ASSERT_DOUBLE_EQ(Hs*Hs/8, A.Si.front());
}

TEST_F(discretizeTest, Dirac_in_direction)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    YamlStretching y;
    const Stretching s(y);
    const DiscreteDirectionalWaveSpectrum A = discretize(S, D, 0.01, 3, 1000, s);
    ASSERT_EQ(1, A.Dj.size());
    ASSERT_EQ(1000, A.Si.size());
    ASSERT_DOUBLE_EQ(2.99/999, A.domega);
    ASSERT_DOUBLE_EQ(1, A.dpsi);
    ASSERT_DOUBLE_EQ(1, A.Dj.front());
}

TEST_F(discretizeTest, should_throw_if_omega_min_is_negative)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().no().greater_than(0);
    const double omega_max = a.random<double>().greater_than(0);
    const size_t nfreq = a.random<size_t>();
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_omega_min_is_zero)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = 0;
    const double omega_max = a.random<double>().greater_than(0);
    const size_t nfreq = a.random<size_t>();
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_omega_max_is_negative)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = a.random<double>().no().greater_than(0);
    const size_t nfreq = a.random<size_t>();
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_omega_max_is_zero)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = 0;
    const size_t nfreq = a.random<size_t>();
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_omega_max_is_lower_than_omega_min)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = a.random<double>().no().greater_than(omega_min);
    const size_t nfreq = a.random<size_t>();
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_nfreq_is_zero)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = a.random<double>().greater_than(0);
    const size_t nfreq = 0;
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_nfreq_is_one_but_omega_min_is_not_omega_max)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = a.random<double>().greater_than(0);
    const size_t nfreq = 1;
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

TEST_F(discretizeTest, should_throw_if_omega_min_equals_omega_max_but_nfreq_is_not_one)
{
    const double Hs = 3;
    const double Tp = 4;
    const double gamma = 1.4;
    const JonswapSpectrum S(Hs, Tp, gamma);
    const DiracDirectionalSpreading D(PI/4);
    const double omega_min = a.random<double>().greater_than(0);
    const double omega_max = omega_min;
    const size_t nfreq = a.random<size_t>().but_not(1);
    YamlStretching y;
    const Stretching s(y);
    ASSERT_THROW(discretize(S, D, omega_min, omega_max, nfreq, s), InvalidInputException);
}

/**
 * \brief This test checks that the flatten and filter functions work correctly
 * on the following spectrum discretization
 *
 * \code
 *    |  3  2  4
 *  ------------
 *  1 |  3  2  4
 *  5 | 15 10 20
 *  4 | 12  8 16
 *  3 |  9  6 12
 * \endcode
 *
 * The following Python code was used to generated threshold input for the
 * filter function
 *
 * \code{.py}
 * import numpy as np
 *  S = np.array([3,2,4])
 * D = np.array([1,5,4,3])
 * f1,f2 = np.meshgrid(S,D)
 * M = f1*f2
 * pct = np.cumsum(np.flip(np.sort(np.hstack(M)),axis=0)) / M.sum()
 * # Evaluate the sorted contribution of each component
 * filter_ratio = np.floor(100*pct)
 * \endcode
 */
TEST_F(discretizeTest, filter)
{
    DiscreteDirectionalWaveSpectrum d;
    d.Si = a.random_vector_of<double>().of_size(3);
    d.Dj = a.random_vector_of<double>().of_size(4);
    d.k = a.random_vector_of<double>().of_size(3);
    d.omega = a.random_vector_of<double>().of_size(3);
    d.phase = std::vector<std::vector<double> >(3,std::vector<double>(4,0));
    d.psi = a.random_vector_of<double>().of_size(4);
    d.domega = 1.0;
    d.dpsi = 1.0;

    d.omega[0] = 0;
    d.omega[1] = 1;
    d.omega[2] = 2;
    d.psi[0] = 10;
    d.psi[1] = 11;
    d.psi[2] = 12;
    d.psi[3] = 13;

    d.Dj[0] = 1;
    d.Dj[1] = 5;
    d.Dj[2] = 4;
    d.Dj[3] = 3;

    d.Si[0] = 3;
    d.Si[1] = 2;
    d.Si[2] = 4;

    // Si.Dj     =   20    16    15    12    12     9     8
    // for (i,j) = (2,1) (2,2) (0,1) (0,2) (2,3) (0,3) (1,2)

    const auto domega = [](const size_t ){return 1;};
    const auto dpsi = [](const size_t ){return 1;};

    FlatDiscreteDirectionalWaveSpectrum s_ori = flatten(d);
    EXPECT_EQ(12, s_ori.a.size());
    EXPECT_EQ(12, s_ori.omega.size());
    EXPECT_EQ(12, s_ori.psi.size());
    EXPECT_EQ(12, s_ori.cos_psi.size());
    EXPECT_EQ(12, s_ori.sin_psi.size());
    EXPECT_EQ(12, s_ori.k.size());
    EXPECT_EQ(0, s_ori.phase.size());

    FlatDiscreteDirectionalWaveSpectrum s = filter(s_ori, 0.0);
    EXPECT_EQ(0, s.a.size());

    s = filter(s_ori, 0.17);
    EXPECT_EQ(1, s.a.size());
    size_t i = 2;
    size_t j = 1;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[0]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[0]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[0]);



    s = filter(s_ori, 0.30);
    EXPECT_EQ(2, s.a.size());
    i = 2;
    j = 2;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[1]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[1]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[1]);

    s = filter(s_ori, 0.43);
    EXPECT_EQ(3, s.a.size());
    i = 0;
    j = 1;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[2]);
    EXPECT_DOUBLE_EQ(0, s.omega[2]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[2]);

    s = filter(s_ori, 0.53);
    EXPECT_EQ(4, s.a.size());
    i = 0;
    j = 2;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[3]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[3]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[3]);

    s = filter(s_ori, 0.64);
    EXPECT_EQ(5, s.a.size());
    i = 2;
    j = 3;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[4]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[4]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[4]);

    s = filter(s_ori, 0.72);
    EXPECT_EQ(6, s.a.size());
    i = 1;
    j = 1;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[5]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[5]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[5]);

    s = filter(s_ori, 0.80);
    EXPECT_EQ(7, s.a.size());
    i = 0;
    j = 3;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[6]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[6]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[6]);

    s = filter(s_ori, 0.87);
    EXPECT_EQ(8, s.a.size());
    i = 1;
    j = 2;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[7]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[7]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[7]);

    s = filter(s_ori, 0.92);
    EXPECT_EQ(9, s.a.size());
    i = 1;
    j = 3;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[8]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[8]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[8]);

    s = filter(s_ori, 0.95);
    EXPECT_EQ(10, s.a.size());
    i = 2;
    j = 0;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[9]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[9]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[9]);

    s = filter(s_ori, 0.98);
    EXPECT_EQ(11, s.a.size());
    i = 0;
    j = 0;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[10]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[10]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[10]);

    s = filter(s_ori, 1.0);
    EXPECT_EQ(12, s.a.size());
    i = 1;
    j = 0;
    EXPECT_DOUBLE_EQ(sqrt(2*domega(i)*dpsi(j)*d.Si[i]*d.Dj[j]), s.a[11]);
    EXPECT_DOUBLE_EQ(d.omega[i], s.omega[11]);
    EXPECT_DOUBLE_EQ(d.psi[j], s.psi[11]);
}

TEST_F(discretizeTest, dynamic_pressure_factor)
{
    //! [discretizeTest dynamic_pressure_factor example]
    YamlStretching y;
    const Stretching s(y);
    ASSERT_DOUBLE_EQ(exp(-3), dynamic_pressure_factor(1,3,2,s));
    ASSERT_DOUBLE_EQ(exp(-10), dynamic_pressure_factor(2,5,4,s));
    ASSERT_DOUBLE_EQ((exp(1)+exp(-1))/(exp(3)+exp(-3)), dynamic_pressure_factor(1,2,3,-4,s));
    ASSERT_DOUBLE_EQ((exp(0.14)+exp(-0.14))/(exp(0.08)+exp(-0.08)), dynamic_pressure_factor(0.2,-0.3,0.4,-0.5,s));
    //! [discretizeTest dynamic_pressure_factor example]
}
