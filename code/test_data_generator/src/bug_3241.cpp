#include <sstream>

#include "yaml_data.hpp"

std::string test_data::bug_3241()
{
    std::stringstream ss;
    ss << "rotations convention: [psi, theta', phi'']\n"
       << "\n"
       << "environmental constants:\n"
       << "    g: {value: 9.81, unit: m/s^2}\n"
       << "    rho: {value: 1000, unit: kg/m^3}\n"
       << "    nu: {value: 1.14e-6, unit: m^2/s}\n"
       << "\n"
       << "# Fixed frame: NED\n"
       << "bodies: # All bodies have NED as parent frame\n"
       << "  - name: dtmb\n"
       << "    position of body frame relative to mesh:\n"
       << "        frame: mesh\n"
       << "        x: {value: 1.509, unit: m}\n"
       << "        y: {value: 0, unit: m}\n"
       << "        z: {value: -0.160, unit: m}\n"
       << "        phi: {value: 0, unit: rad}\n"
       << "        theta: {value: 0, unit: rad}\n"
       << "        psi: {value: 0, unit: rad}\n"
       << "    dynamics:\n"
       << "        hydrodynamic forces calculation point in body frame:\n"
       << "            x: {value: 0, unit: m}\n"
       << "            y: {value: 0, unit: m}\n"
       << "            z: {value: 0.072, unit: m}\n"
       << "        centre of inertia:\n"
       << "            frame: dtmb\n"
       << "            x: {value: 0, unit: m}\n"
       << "            y: {value: 0, unit: m}\n"
       << "            z: {value: 0, unit: m} \n"
       << "        rigid body inertia matrix at the center of gravity and projected in the body frame:\n"
       << "            row 1: [80.555,0,0,0,0,0]\n"
       << "            row 2: [0,80.555,0,0,0,0]\n"
       << "            row 3: [0,0,80.555,0,0,0]\n"
       << "            row 4: [0,0,0,1.845,0,0]\n"
       << "            row 5: [0,0,0,0,46.77,0]\n"
       << "            row 6: [0,0,0,0,0,46.77]\n"
       << "        added mass matrix at the center of gravity and projected in the body frame:\n"
       << "            # T min Aqua+ (0.51s)\n"
       << "            row 1: [ 8.663E-01, 0.000E+00, -1.911E+00,  0.000E+00, -4.063E+00,  0.000E+00]\n"
       << "            row 2: [ 0.000E+00, 2.096E+01,  0.000E+00,  2.351E-01,  0.000E+00,  5.225E+00]\n"
       << "            row 3: [-1.911E+00, 0.000E+00,  9.340E+01,  0.000E+00,  1.391E+01,  0.000E+00]\n"
       << "            row 4: [ 0.000E+00, 2.351E-01,  0.000E+00,  1.718E-01,  0.000E+00, -9.904E-01]\n"
       << "            row 5: [-4.063E+00, 0.000E+00,  1.391E+01,  0.000E+00,  4.053E+01,  0.000E+00]\n"
       << "            row 6: [ 0.000E+00, 5.225E+00,  0.000E+00, -9.904E-01,  0.000E+00,  1.484E+01]\n"
       << "    mesh: dtmb_IIHR.stl\n"
       << "    initial position of body frame relative to NED:\n"
       << "        frame: NED\n"
       << "        x: {value: 0, unit: m}\n"
       << "        y: {value: 0, unit: m}\n"
       << "        z: {value: -0.0357, unit: m}\n"
       << "        phi: {value: 0, unit: deg}\n"
       << "        theta: {value: 0.027, unit: deg}\n"
       << "        psi: {value: 0, unit: deg}\n"
       << "    initial velocity of body frame relative to NED:\n"
       << "        frame: dtmb\n"
       << "        u: {value: 1.531, unit: m/s}\n"
       << "        v: {value: 0, unit: m/s}\n"
       << "        w: {value: 0.000721, unit: m/s}\n"
       << "        p: {value: 0, unit: rad/s}\n"
       << "        q: {value: 0, unit: rad/s}\n"
       << "        r: {value: 0, unit: rad/s}\n"
       << "    blocked dof:\n"
       << "        from YAML:\n"
       << "            - state: u\n"
       << "              t: [0, 5, 10, 15]\n"
       << "              value: [1.531, 1, 1.5, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: v\n"
       << "              t: [0, 10000]\n"
       << "              value: [0, 0]\n"
       << "              interpolation: piecewise constant\n"
       << "            - state: w\n"
       << "              t: [0, 10000]\n"
       << "              value: [0.000721, 0.000721]\n"
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
       << "      - model: gravity\n"
       << "      - model: non-linear hydrostatic (exact)\n"
       << "      - model: non-linear Froude-Krylov\n"
       << "      - model: linear damping\n"
       << "        damping matrix at the center of gravity projected in the body frame:\n"
       << "            row 1: [ 0, 0,      0,      0,       0, 0]\n"
       << "            row 2: [ 0, 0,      0,      0,       0, 0]\n"
       << "            row 3: [ 0, 0, 5.83e2,      0,       0, 0]\n"
       << "            row 4: [ 0, 0,      0, 2.19e-1,       0, 0]\n"
       << "            row 5: [ 0, 0,      0,      0, 3.06e2, 0]\n"
       << "            row 6: [ 0, 0,      0,      0,       0, 0]\n"
       << "      - model: quadratic damping\n"
       << "        damping matrix at the center of gravity projected in the body frame:\n"
       << "            row 1: [ 0, 0, 0,      0, 0, 0]\n"
       << "            row 2: [ 0, 0, 0,      0, 0, 0]\n"
       << "            row 3: [ 0, 0, 0,      0, 0, 0]\n"
       << "            row 4: [ 0, 0, 0, 2.35e-1, 0, 0]\n"
       << "            row 5: [ 0, 0, 0,      0, 0, 0]\n"
       << "            row 6: [ 0, 0, 0,      0, 0, 0]\n"
       << "      - model: resistance curve\n"
       << "        speed: {unit: m/s, values: [-0.14, 0.00, 0.14, 0.27, 0.41, 0.55, 0.68, 0.82, 0.96, 1.09, 1.23, 1.37, 1.50, 1.64, 1.78, 1.91, 2.05, 2.19, 2.32, 2.46]}\n"
       << "        resistance: {unit: N, values: [-0.08, 0.00, 0.08, 0.30, 0.68, 1.21, 1.89, 2.72, 3.71, 4.84, 6.13, 7.57, 9.16, 10.90, 12.79, 14.83, 17.03, 19.37, 21.87, 24.52]}\n"
       << "environment models:\n"
       << "  - model: no waves\n"
       << "    constant sea elevation in NED frame: {value: 0, unit: m}\n"
       << "output:\n"
       << "  - format: map\n"
       << "    data: [u(dtmb)]";
    return ss.str();
}
