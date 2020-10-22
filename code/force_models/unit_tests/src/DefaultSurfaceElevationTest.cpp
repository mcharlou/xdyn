/*
 * HydrostaticForceTest.cpp
 *
 *  Created on: 31 mars 2014
 *      Author: cec
 */

#include <ssc/kinematics.hpp>

#include "DefaultSurfaceElevation.hpp"

#include "DefaultSurfaceElevationTest.hpp"

DefaultSurfaceElevationTest::DefaultSurfaceElevationTest() : a(ssc::random_data_generator::DataGenerator(5466123))
{
}

DefaultSurfaceElevationTest::~DefaultSurfaceElevationTest()
{
}

void DefaultSurfaceElevationTest::SetUp()
{
}

void DefaultSurfaceElevationTest::TearDown()
{
}

TEST_F(DefaultSurfaceElevationTest, example)
{
//! [DefaultWaveModelTest example]
	ssc::kinematics::KinematicsPtr k(new ssc::kinematics::Kinematics());
    const ssc::kinematics::Point P("NED", 0, 0, -20);
    ssc::kinematics::PointMatrixPtr mesh(new ssc::kinematics::PointMatrix(P.v,"NED"));
    DefaultSurfaceElevation w(10, mesh);
//! [DefaultWaveModelTest example]
//! [DefaultWaveModelTest expected output]
    w.update_surface_elevation(mesh, k, (double)a.random<double>());
    ASSERT_DOUBLE_EQ(-30, w.get_relative_wave_height().front());
//! [DefaultWaveModelTest expected output]
}

TEST_F(DefaultSurfaceElevationTest, more_interesting_example)
{
	ssc::kinematics::KinematicsPtr k(new ssc::kinematics::Kinematics());
    const ssc::kinematics::Point P("NED", 7, -100, 1795);
    ssc::kinematics::PointMatrixPtr mesh(new ssc::kinematics::PointMatrix(P.v,"NED"));
    DefaultSurfaceElevation w(7, mesh);

    const ssc::kinematics::Transform bodyTned(ssc::kinematics::Point("NED", 4, 5, 9), "body");
    k->add(bodyTned);
    w.update_surface_elevation(mesh,k, a.random<double>());
    ASSERT_DOUBLE_EQ(1788, w.get_relative_wave_height().front());
}

TEST_F(DefaultSurfaceElevationTest, more_interesting_example_with_rotation_180p_around_Y)
{
	ssc::kinematics::KinematicsPtr k(new ssc::kinematics::Kinematics());
    const ssc::kinematics::Point P("body", 0, 0, 1);
    const ssc::kinematics::Point Q("NED", 7, -100, 1795);
    ssc::kinematics::PointMatrixPtr mesh(new ssc::kinematics::PointMatrix(Q.v,"NED"));
    DefaultSurfaceElevation w(7, mesh);
    ssc::kinematics::RotationMatrix rot;
    rot(0,0)=-1.0;
    rot(0,1)=0.0;
    rot(0,2)=0.0;
    rot(1,0)=0.0;
    rot(1,1)=1.0;
    rot(1,2)=0.0;
    rot(2,0)=0.0;
    rot(2,1)=0.0;
    rot(2,2)=-1.0;
    const ssc::kinematics::Transform bodyTned(ssc::kinematics::Point("NED", 4, 5, 9), rot, "body");
    k->add(bodyTned);
    w.update_surface_elevation(mesh,k, a.random<double>());
    ASSERT_DOUBLE_EQ(1788, w.get_relative_wave_height().front());
}

TEST_F(DefaultSurfaceElevationTest, more_interesting_example_with_rotation_with_rotation_090p_around_Y)
{
	ssc::kinematics::KinematicsPtr k(new ssc::kinematics::Kinematics());
    const ssc::kinematics::Point Q("NED", 7, -100, 1795);
    ssc::kinematics::PointMatrixPtr mesh(new ssc::kinematics::PointMatrix(Q.v,"NED"));
    DefaultSurfaceElevation w(7, mesh);
    ssc::kinematics::RotationMatrix rot;
    rot(0,0)= 0.0;
    rot(0,1)= 0.0;
    rot(0,2)= 1.0;
    rot(1,0)= 0.0;
    rot(1,1)= 1.0;
    rot(1,2)= 0.0;
    rot(2,0)=-1.0;
    rot(2,1)= 0.0;
    rot(2,2)= 0.0;
    const ssc::kinematics::Transform bodyTned(ssc::kinematics::Point("NED", 4, 5, 9), rot, "body");
    k->add(bodyTned);
    w.update_surface_elevation(mesh,k, a.random<double>());
    ASSERT_DOUBLE_EQ(1788, w.get_relative_wave_height().front());
}

TEST_F(DefaultSurfaceElevationTest, more_interesting_example_with_rotation)
{
	ssc::kinematics::KinematicsPtr k(new ssc::kinematics::Kinematics());
    const ssc::kinematics::Point Q("NED", 7, -100, 1795);
    ssc::kinematics::PointMatrixPtr mesh(new ssc::kinematics::PointMatrix(Q.v,"NED"));
    DefaultSurfaceElevation w(7, mesh);
    ssc::kinematics::RotationMatrix rot;
    rot(0,0)=-1.0;
    rot(0,1)=0.0;
    rot(0,2)=0.0;
    rot(1,0)=0.0;
    rot(1,1)=1.0;
    rot(1,2)=0.0;
    rot(2,0)=0.0;
    rot(2,1)=0.0;
    rot(2,2)=-1.0;
    const ssc::kinematics::Transform bodyTned(ssc::kinematics::Point("NED", 4, 5, 9), rot, "body");
    k->add(bodyTned);
    w.update_surface_elevation(mesh,k, a.random<double>());
    ASSERT_DOUBLE_EQ(1788, w.get_relative_wave_height().front());
}

TEST_F(DefaultSurfaceElevationTest, can_compute_relative_wave_height_for_PointMatrix)
{
    //! [DefaultWaveModelTest example]
    ssc::kinematics::PointMatrix M("body", 5);
    M.m(0,0) = 12;
    M.m(1,0) = -11;
    M.m(2,0) = 10;
    M.m(0,1) = -19;
    M.m(1,1) = 81;
    M.m(2,1) = 71;
    M.m(0,2) = 16;
    M.m(1,2) = -51;
    M.m(2,2) = 14;
    M.m(0,3) = 31;
    M.m(1,3) = -12;
    M.m(2,3) = -11;
    M.m(0,4) = 10;
    M.m(1,4) = -81;
    M.m(2,4) = -91;

    ssc::kinematics::KinematicsPtr k(new ssc::kinematics::Kinematics());
    ssc::kinematics::PointMatrixPtr mesh;
    DefaultSurfaceElevation w(888, mesh);
    const ssc::kinematics::Transform T(ssc::kinematics::Point("NED", 1, 20, 300), "body");
    k->add(T);
    ssc::kinematics::PointMatrixPtr Mptr(new ssc::kinematics::PointMatrix(M));
    w.update_surface_elevation(Mptr, k, a.random<double>());
    const std::vector<double> delta = w.get_relative_wave_height();
    //! [DefaultWaveModelTest example]
    //! [DefaultWaveModelTest expected output]
    ASSERT_EQ(5, delta.size());
    ASSERT_DOUBLE_EQ(-888+300+10, delta.at(0));
    ASSERT_DOUBLE_EQ(-888+300+71, delta.at(1));
    ASSERT_DOUBLE_EQ(-888+300+14, delta.at(2));
    ASSERT_DOUBLE_EQ(-888+300-11, delta.at(3));
    ASSERT_DOUBLE_EQ(-888+300-91, delta.at(4));
    //! [DefaultWaveModelTest expected output]
}
