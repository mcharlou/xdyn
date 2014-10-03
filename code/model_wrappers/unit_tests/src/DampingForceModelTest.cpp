/*
 * DampingForceModelTest.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: jacquenot
 */

#include "DampingForceModel.hpp"
#include "DampingForceModelTest.hpp"
#include "generate_body_for_tests.hpp"
#include <ssc/kinematics.hpp>

#define BODY "body 1"

DampingForceModelTest::DampingForceModelTest() : a(ssc::random_data_generator::DataGenerator(666))
{
}

DampingForceModelTest::~DampingForceModelTest()
{
}

void DampingForceModelTest::SetUp()
{
}

void DampingForceModelTest::TearDown()
{
}

TEST_F(DampingForceModelTest, example_with_null_velocities)
{
//! [DampingForceModelTest example]
    DampingForceModel F;
    Body b = get_body(BODY);
    const ssc::kinematics::Wrench f = F(b,0.0);
//! [DampingForceModelTest example]
//! [DampingForceModelTest expected output]
    ASSERT_EQ(BODY, f.get_frame());
    ASSERT_DOUBLE_EQ(0, f.K());
    ASSERT_DOUBLE_EQ(0, f.M());
    ASSERT_DOUBLE_EQ(0, f.N());
    ASSERT_DOUBLE_EQ(0, f.X());
    ASSERT_DOUBLE_EQ(0, f.Y());
    ASSERT_DOUBLE_EQ(0, f.Z());
//! [DampingForceModelTest expected output]
}

TEST_F(DampingForceModelTest, example_with_random_positive_velocities_and_identity_damping_matrix)
{
    const double EPS = 1e-11;
    DampingForceModel F;
    double u,v,w,p,q,r;
    Body b = get_body(BODY);
    b.m = a.random<double>();
    for (size_t i=0;i<100;++i)
    {
        b.u = u = a.random<double>().greater_than(0.0);
        b.v = v = a.random<double>().greater_than(0.0);
        b.w = w = a.random<double>().greater_than(0.0);
        b.p = p = a.random<double>().greater_than(0.0);
        b.q = q = a.random<double>().greater_than(0.0);
        b.r = r = a.random<double>().greater_than(0.0);
        const ssc::kinematics::Wrench f = F(b,a.random<double>());
        ASSERT_EQ(BODY, f.get_frame());
        ASSERT_NEAR(u*u, f.X(),EPS);
        ASSERT_NEAR(v*v, f.Y(),EPS);
        ASSERT_NEAR(w*w, f.Z(),EPS);
        ASSERT_NEAR(p*p, f.K(),EPS);
        ASSERT_NEAR(q*q, f.M(),EPS);
        ASSERT_NEAR(r*r, f.N(),EPS);
    }
}

TEST_F(DampingForceModelTest, example_with_dense_damping_matrix)
{
    const double EPS = 1e-11;
    Eigen::Matrix<double,6,6> d;
    DampingForceModel F;
    double u,v,w,p,q,r;
    double uu,vv,ww,pp,qq,rr;
    Body b = get_body(BODY);
    b.m = a.random<double>();
    d<<  2,   3,   5,   7,  11,  13,
        17,  19,  23,  29,  31,  37,
        41,  43,  47,  53,  59,  61,
        67,  71,  73,  79,  83,  89,
        97, 101, 103, 107, 109, 113,
       127, 131, 137, 139, 149, 151;
    b.damping = MatrixPtr(new Eigen::Matrix<double,6,6>(d));
    for (size_t i=0;i<100;++i)
    {
        b.u = u = a.random<double>().between(-100.0,+100.0);
        b.v = v = a.random<double>().between(-100.0,+100.0);
        b.w = w = a.random<double>().between(-100.0,+100.0);
        b.p = p = a.random<double>().between(-100.0,+100.0);
        b.q = q = a.random<double>().between(-100.0,+100.0);
        b.r = r = a.random<double>().between(-100.0,+100.0);
        uu = fabs(u)*u;
        vv = fabs(v)*v;
        ww = fabs(w)*w;
        pp = fabs(p)*p;
        qq = fabs(q)*q;
        rr = fabs(r)*r;
        const ssc::kinematics::Wrench f = F(b,a.random<double>());
        ASSERT_EQ(BODY, f.get_frame());
        for (size_t j=0;j<3;++j)
        {
            const size_t k = j+3;
            ASSERT_NEAR(d(j,0)*uu+d(j,1)*vv+d(j,2)*ww+d(j,3)*pp+d(j,4)*qq+d(j,5)*rr,f.force[j],EPS)<<"test:"<<i << " row:"<<j;
            ASSERT_NEAR(d(k,0)*uu+d(k,1)*vv+d(k,2)*ww+d(k,3)*pp+d(k,4)*qq+d(k,5)*rr,f.torque[j],EPS)<<"test:"<<i << " row:"<<k;
        }
    }
}
