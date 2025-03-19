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
    /* p_x   */ prediction(0) = x.p_x() + u.dt()*(x.u()*cos(x.omega()) - x.v()*sin(x.omega()));
    /* p_y   */ prediction(1) = x.p_y() + u.dt()*(x.u()*sin(x.omega()) - x.v()*cos(x.omega()));
    /* phi   */ prediction(2) = x.phi() + u.dt()*(x.omega());
    /* u     */ prediction(3) = x.u() + u.dt()*(u.a() + x.v()*x.omega() - Fy1*sin(u.delta()/_params.m));
    /* v     */ prediction(4) = x.v() + u.dt()*(- x.u()*x.omega() + (Fy1*cos(u.delta()) + Fy2)/_params.m);
    /* omega */ prediction(5) = x.omega() + u.dt()*(_params.l_f*Fy1*cos(u.delta()) - _params.l_r*Fy2)/_params.I_z;

    return prediction;
}

EMatrix<double, STATE_DIM, STATE_DIM> DBMMK1::Model::get_state_jacobian(const X &state, const U &control)
{
    return EMatrix<double, STATE_DIM, STATE_DIM>();
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
