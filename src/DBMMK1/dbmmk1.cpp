#include "dbmmk1.hpp"

#include <math.h>

using namespace DBMMK1;

mirena::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, MEASURE_DIM> DBMMK1::Model::build_ekf()
{
    // Use this model as both a Measure and Control model
    std::shared_ptr<Model> model = std::make_shared<Model>();
    return mirena::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, MEASURE_DIM>(model, model);
}

X DBMMK1::Model::predict_state(const X &previous_state, const U &control)
{
    // Preparation, and pre-calculation of Fy1 & Fy2
    X prediction;
    StateAcessor x(previous_state);
    ControlAcessor u(control);
    double Fy1 = _params.k_f * (((x.v() + _params.l_f * x.omega()) / (x.u())) - u.delta());
    double Fy2 = _params.k_r * (((x.v() + _params.l_r * x.omega()) / (x.u())));

    // Obtention of the actual prediction
    /* p_x   */ prediction(0) = x.p_x() + u.dt() * (x.u() * cos(x.omega()) - x.v() * sin(x.omega()));
    /* p_y   */ prediction(1) = x.p_y() + u.dt() * (x.u() * sin(x.omega()) - x.v() * cos(x.omega()));
    /* phi   */ prediction(2) = x.phi() + u.dt() * (x.omega());
    /* u     */ prediction(3) = x.u() + u.dt() * (u.a() + x.v() * x.omega() - Fy1 * sin(u.delta() / _params.m));
    /* v     */ prediction(4) = x.v() + u.dt() * (-x.u() * x.omega() + (Fy1 * cos(u.delta()) + Fy2) / _params.m);
    /* omega */ prediction(5) = x.omega() + u.dt() * (_params.l_f * Fy1 * cos(u.delta()) - _params.l_r * Fy2) / _params.I_z;

    return prediction;
}

EMatrix<double, STATE_DIM, STATE_DIM> DBMMK1::Model::get_state_jacobian(const X &state, const U &control)
{
    // Preparation
    EMatrix<double, STATE_DIM, STATE_DIM> J = EMatrix<double, STATE_DIM, STATE_DIM>::Zero();
    const EMatrix<double, STATE_DIM, STATE_DIM> I = EMatrix<double, STATE_DIM, STATE_DIM>::Identity();
    StateAcessor x(state);
    ControlAcessor u(control);

    // Pre-Calculation of all the relevant partial derivatives of Fy1 and Fy2
    double Fy1_du = -_params.k_f * (x.v() + _params.l_f * x.omega()) / (x.u() * x.u());
    double Fy1_dv = _params.k_f / (x.u());
    double Fy1_dw = _params.k_f * (_params.l_f) / (x.u());
    double Fy2_du = -_params.k_r * (x.v() + _params.l_r * x.omega()) / (x.u() * x.u());
    double Fy2_dv = _params.k_r / (x.u());
    double Fy2_dw = _params.k_r * (_params.l_r) / (x.u());

    // Calculate the jacobian
    // Row 0
    J(0, 2) = -x.u() * sin(x.phi()) - x.v() * cos(x.phi());
    J(0, 3) = cos(x.phi());
    J(0, 4) = -sin(x.phi());

    // Row 1
    J(1, 2) = x.u() * cos(x.phi()) - x.v() * sin(x.phi());
    J(1, 3) = sin(x.phi());
    J(1, 4) = cos(x.phi());

    // Row 2
    J(2, 5) = 1.0;

    // Row 3:
    J(3, 3) = -(Fy1_du * sin(u.delta()))/ _params.m; 
    J(3, 4) = x.omega() - (Fy1_dv * sin(u.delta()))/_params.m;
    J(3, 5) = x.v() - (Fy1_dw * sin(u.delta()))/_params.m;

    // Row 4:
    J(4, 3) = -x.omega() + (Fy1_du * cos(u.delta()) + Fy2_du)/_params.m;
    J(4, 4) = (Fy1_dv * cos(u.delta()) + Fy2_dv)/_params.m;
    J(4, 5) = -x.u() + (Fy1_dw * cos(u.delta()) + Fy2_dw)/_params.m;

    // Row 5:
    J(5, 3) = (_params.l_f * Fy1_du * cos(u.delta()) - _params.l_r * Fy2_du)/_params.I_z;
    J(5, 4) = (_params.l_f * Fy1_dv * cos(u.delta()) - _params.l_r * Fy2_dv)/_params.I_z;
    J(5, 5) = (_params.l_f * Fy1_dw * cos(u.delta()) - _params.l_r * Fy2_dw)/_params.I_z;

    return I + u.dt() * J;
}

//
// NOTE: Probably the two functions below could be optimized by reusing the state prediction and jacobian, but tbh im too lazy to implement cachein.
// Just let that idea sink in...
//

Z DBMMK1::Model::predict_measure(const X &state)
{
    return EMatrix<double, MEASURE_DIM, 1>();
}

EMatrix<double, MEASURE_DIM, STATE_DIM> DBMMK1::Model::get_measure_jacobian(const X &state)
{
    return EMatrix<double, MEASURE_DIM, STATE_DIM>();
}
