/*
 * HydrostaticForceTest.cpp
 *
 *  Created on: 31 mars 2014
 *      Author: maroff
 */

#include "DefaultWaveModelTest.hpp"
#include "DefaultWaveModel.hpp"
#include "Point.hpp"

DefaultWaveModelTest::DefaultWaveModelTest() : a(DataGenerator(5466123))
{
}

DefaultWaveModelTest::~DefaultWaveModelTest()
{
}

void DefaultWaveModelTest::SetUp()
{
}

void DefaultWaveModelTest::TearDown()
{
}

TEST_F(DefaultWaveModelTest, example)
{
//! [DefaultWaveModelTest example]
    const Point P("A", 1, 2, 3);
    const DefaultWaveModel w(3);

//! [DefaultWaveModelTest example]
//! [DefaultWaveModelTest expected output]
//! [DefaultWaveModelTest expected output]
}
