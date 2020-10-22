#include <sstream>

#include "yaml_data.hpp"



std::string test_data::yml_bug_3230()
{
    std::stringstream ss;
    ss << "rotations convention: [psi, theta', phi'']\n"
       << "\n"
       << "environmental constants:\n"
       << "    g: {value: 9.81, unit: m/s^2}\n"
       << "    rho: {value: 1025, unit: kg/m^3}\n"
       << "    nu: {value: 1.19e-6, unit: m^2/s}\n"
	   << "    air density: {value: 1.225, unit: kg/m^2}\n"
       << "\n"
       << "# Fixed frame: NED\n"
       << "bodies: # All bodies have NED as parent frame\n"
       << "  - name: ship\n"
       << "    position of body frame relative to mesh:\n"
       << "        frame: mesh\n"
       << "        x: {value: 61.005, unit: m}\n"
       << "        y: {value: 0, unit: m}\n"
       << "        z: {value: -7.32, unit: m}\n"
       << "        phi: {value: 0, unit: rad}\n"
       << "        theta: {value: 0, unit: rad}\n"
       << "        psi: {value: 0, unit: rad}\n"
       << "    dynamics:\n"
       << "        hydrodynamic forces calculation point in body frame:\n"
       << "            x: {value: 0, unit: m}\n"
       << "            y: {value: 0, unit: m}\n"
       << "            z: {value: 3.92, unit: m}\n"
       << "        centre of inertia:\n"
       << "            frame: ship\n"
       << "            x: {value: 0, unit: m}\n"
       << "            y: {value: 0, unit: m}\n"
       << "            z: {value: 0, unit: m} \n"
       << "        rigid body inertia matrix at the center of gravity and projected in the body frame:\n"
       << "            row 1: [6.05e6,0,0,0,0,0]\n"
       << "            row 2: [0,6.05e6,0,0,0,0]\n"
       << "            row 3: [0,0,6.05e6,0,0,0]\n"
       << "            row 4: [0,0,0,3.07e8,0,0]\n"
       << "            row 5: [0,0,0,0,6.40e9,0]\n"
       << "            row 6: [0,0,0,0,0,6.40e9]\n"
       << "        added mass matrix at the center of gravity and projected in the body frame:\n"
       << "            frame: ship\n"
       << "            # T min Aqua+ (3.5s)\n"
       << "            row 1: [ 5.814E+04, 0.000E+00, -2.021E+05,  0.000E+00, -1.467E+07,  0.000E+00]\n"
       << "            row 2: [ 0.000E+00, 1.533E+06,  0.000E+00,  1.056E+05,  0.000E+00,  1.871E+07]\n"
       << "            row 3: [-2.021E+05, 0.000E+00,  7.830E+06,  0.000E+00,  7.468E+07,  0.000E+00]\n"
       << "            row 4: [ 0.000E+00, 1.056E+05,  0.000E+00,  3.170E+07,  0.000E+00, -1.402E+08]\n"
       << "            row 5: [-1.467E+07, 0.000E+00,  7.468E+07,  0.000E+00,  6.398E+09,  0.000E+00]\n"
       << "            row 6: [ 0.000E+00, 1.871E+07,  0.000E+00, -1.402E+08,  0.000E+00,  2.105E+09]\n"
       << "    mesh: ship_2008.stl\n"
       << "    initial position of body frame relative to NED:\n"
       << "        frame: NED\n"
       << "        x: {value: 0, unit: m}\n"
       << "        y: {value: 0, unit: m}\n"
       << "        z: {value: -2.18, unit: m}\n"
       << "        phi: {value: 0, unit: deg}\n"
       << "        theta: {value: 0.013, unit: deg}\n"
       << "        psi: {value: 0, unit: deg}\n"
       << "    initial velocity of body frame relative to NED:\n"
       << "        frame: ship\n"
       << "        u: {value: 5, unit: m/s}\n"
       << "        v: {value: 0, unit: m/s}\n"
       << "        w: {value: 1.13e-3, unit: m/s}\n"
       << "        p: {value: 0, unit: rad/s}\n"
       << "        q: {value: 0, unit: rad/s}\n"
       << "        r: {value: 0, unit: rad/s}\n"
       << "    blocked dof:\n"
       << "        from YAML:\n"
       << "            - state: u\n"
       << "              t: [0, 10000]\n"
       << "              value: [5, 5]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: v\n"
       << "              t: [0, 10000]\n"
       << "              value: [0, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: w\n"
       << "              t: [0, 10000]\n"
       << "              value: [0, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: p\n"
       << "              t: [0, 10000]\n"
       << "              value: [0, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: q\n"
       << "              t: [0, 10000]\n"
       << "              value: [0, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: r\n"
       << "              t: [0, 10000]\n"
       << "              value: [0, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "    external forces:\n"
       << "      - model: diffraction\n"
       << "        hdb: ship_2008.HDB\n"
       << "        calculation point in body frame:\n"
       << "            x: {value: 0, unit: m}\n"
       << "            y: {value: 0, unit: m}\n"
       << "            z: {value: 0, unit: m}\n"
       << "        mirror for 180 to 360: true\n"
       << "environment models:\n"
       << "  # - model: no waves\n"
       << "    # constant sea elevation in NED frame: {value: 0, unit: m}\n"
       << "  - model: waves\n"
       << "    discretization:\n"
       << "       n: 1\n"
       << "       omega min: {value: 0.1, unit: rad/s}\n"
       << "       omega max: {value: 6, unit: rad/s}\n"
       << "       energy fraction: 0.999\n"
       << "    spectra:\n"
       << "      - model: airy\n"
       << "        depth: {value: 100000, unit: m}\n"
       << "        seed of the random data generator: 0\n"
       << "        stretching:\n"
       << "           delta: 0\n"
       << "           h: {unit: m, value: 100000}\n"
       << "        directional spreading:\n"
       << "           type: dirac\n"
       << "           waves propagating to: {value: 180, unit: deg}\n"
       << "        spectral density:\n"
       << "           type: dirac\n"
       << "           omega0: {value: 0.62831853, unit: rad/s}\n"
       << "           Hs: {value: 2., unit: m}\n"
       << "    output:\n"
       << "        frame of reference: NED(ship)\n"
       << "        mesh:\n"
       << "            xmin: {value: 0, unit: m}\n"
       << "            xmax: {value: 0, unit: m}\n"
       << "            nx: 1\n"
       << "            ymin: {value: 0, unit: m}\n"
       << "            ymax: {value: 0, unit: m}\n"
       << "            ny: 1\n"
       << "output:\n"
       << "  - format: map\n"
       << "    data: ['Fx(diffraction,ship,ship)', 'Fy(diffraction,ship,ship)', 'Fz(diffraction,ship,ship)', 'Mx(diffraction,ship,ship)', 'My(diffraction,ship,ship)', 'Mz(diffraction,ship,ship)']\n";
    return ss.str();
}
