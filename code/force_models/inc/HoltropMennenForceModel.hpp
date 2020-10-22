/*
 * HoltropMennenForceModel.hpp
 *
 *  Created on: 16 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_HOLTROPMENNENFORCEMODEL_HPP_
#define FORCE_MODELS_INC_HOLTROPMENNENFORCEMODEL_HPP_

#include <functional>

#include "ForceModelAtH.hpp"

class HoltropMennenForceModel : public ForceModelAtH
{
public:
  struct Yaml
  {
    Yaml();
    double Lwl;
    double Lpp;
    double B;
    double Ta;
    double Tf;
    double Vol;
    double lcb;
    double S;
    double Abt;
    double hb;
    double Cm;
    double Cwp;
    double At;
    double Sapp;
    double Cstern;
    double form_coeff_app;
    bool apply_on_ship_speed_direction;
  };
  HoltropMennenForceModel(const Yaml& data, const std::string& body_name, const EnvironmentAndFrames& env);
  Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
  static Yaml parse(const std::string& yaml);
  static std::string model_name();

private:
  double Rf(const BodyStates& states, const EnvironmentAndFrames& env) const;
  double Rapp(const BodyStates& states, const EnvironmentAndFrames& env) const;
  double Rw(const BodyStates& states, const EnvironmentAndFrames& env) const;
  double Rb(const BodyStates& states, const EnvironmentAndFrames& env) const;
  double Rtr(const BodyStates& states, const EnvironmentAndFrames& env) const;
  double Ra(const BodyStates& states, const EnvironmentAndFrames& env) const;
  bool apply_on_ship_speed_direction;
  const double d;
  const double L;
  const double Lpp;
  const double B;
  const double c7;
  const double Ta;
  const double Tf;
  const double c4;
  const double T;
  const double m3;
  const double Vol;
  const double c15;
  const double lcb;
  const double S;
  const double Abt;
  const double hb;
  const double Pb;
  const double c3;
  const double c2;
  double Ca;
  const double Cm;
  const double c17;
  const double Cp;
  const double c16;
  const double m1;
  const double Lr;
  const double lambda;
  const double Cwp;
  const double iE;
  const double c1;
  const double At;
  const double c5;
  const double Sapp;
  const double Cstern;
  const double c14;
  const double form_coeff_hull;
  const double form_coeff_app;
  const double Cb;
  const std::function<double(double, double)> Rw_a;
  const std::function<double(double, double)> Rw_b;
};

#endif /* FORCE_MODELS_INC_HOLTROPMENNENFORCEMODEL_HPP_ */
