#include "dbmmk1.hpp"

using namespace DBMMK1;

mirena::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, MEASURE_DIM> DBMMK1::Model::build_ekf()
{
    // Use this model as both a Measure and Control model
    std::shared_ptr<Model> model = std::make_shared<Model>();
    return mirena::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, MEASURE_DIM>(model, model);
}

EMatrix<double, STATE_DIM, 1> DBMMK1::Model::predict_state(const EMatrix<double, STATE_DIM, 1> &previous_state, const EMatrix<double, CONTROL_DIM, 1> &control)
{
    return EMatrix<double, STATE_DIM, 1>();
}

EMatrix<double, STATE_DIM, STATE_DIM> DBMMK1::Model::get_state_jacobian(const EMatrix<double, STATE_DIM, 1> &state, const EMatrix<double, CONTROL_DIM, 1> &control)
{
    return EMatrix<double, STATE_DIM, STATE_DIM>();
}

EMatrix<double, MEASURE_DIM, 1> DBMMK1::Model::predict_measure(const EMatrix<double, STATE_DIM, 1> &state)
{
    return EMatrix<double, MEASURE_DIM, 1>();
}

EMatrix<double, MEASURE_DIM, STATE_DIM> DBMMK1::Model::get_measure_jacobian(const EMatrix<double, STATE_DIM, 1> &state)
{
    return EMatrix<double, MEASURE_DIM, STATE_DIM>();
}

void DBMMK1::Model::get_Fy1(const X &state, const U &control, double &output)
{
}

void DBMMK1::Model::get_Fy2(const X &state, const U &control, double &output)
{
}

void DBMMK1::Model::get_Fy1_partials(const X &state, const U &control, double output[3])
{
}

void DBMMK1::Model::get_Fy2_partials(const X &state, const U &control, double output[3])
{
}
