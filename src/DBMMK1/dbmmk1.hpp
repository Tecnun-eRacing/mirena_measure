#ifndef dbmmk1_hpp
#define dbmmk1_hpp

#include "EKF/extended_kalman_filter.hpp"

namespace DBMMK1 
{
    static constexpr int STATE_DIM = 6;
    static constexpr int CONTROL_DIM = 3;
    static constexpr int MEASURE_DIM = 4;
    
    // Implementation of the Dynamic Bicycle Model Mark 1
    // Ready to be used in the EKM as both the measure and prediction model
    class Model: public mirena::StatePredictor<STATE_DIM, CONTROL_DIM>, public mirena::MeasurePredictor<STATE_DIM, MEASURE_DIM>
    {
    public:
    // Predict next state based on current state + control input. Corresponds the "f" in theory
    EMatrix<double, STATE_DIM, 1> predict_state(
        const EMatrix<double, STATE_DIM, 1> &previous_state,
        const EMatrix<double, CONTROL_DIM, 1> &control) override;

    // Get jacobian evaluated at the given state assuming the control vector is constant. Corresponds to "F" in theory
    EMatrix<double, STATE_DIM, STATE_DIM> get_state_jacobian(
        const EMatrix<double, STATE_DIM, 1> &state,
        const EMatrix<double, CONTROL_DIM, 1> &control) override;

    // Predict next measure based on current state. Corresponds to "h" in theory
    EMatrix<double, MEASURE_DIM, 1> predict_measure(
        const EMatrix<double, STATE_DIM, 1> &state) override;
    
    // Get jacobian evaluated at the given state. Corresponds to "H" in theory
    EMatrix<double, MEASURE_DIM, STATE_DIM> get_measure_jacobian(
        const EMatrix<double, STATE_DIM, 1> &state) override;
    };

    // Build a Extended Kalman Filter using this model
    static mirena::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, MEASURE_DIM> build_ekf();

} // namespace mirena

#endif