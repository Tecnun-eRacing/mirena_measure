#ifndef dbmmk1_hpp
#define dbmmk1_hpp

#include "EKF/extended_kalman_filter.hpp"

namespace DBMMK1 
{
    static constexpr int STATE_DIM = 6;
    static constexpr int CONTROL_DIM = 3;
    static constexpr int MEASURE_DIM = 4;

    typedef EMatrix<double, STATE_DIM, 1> X;
    typedef EMatrix<double, CONTROL_DIM, 1> U;
    typedef EMatrix<double, MEASURE_DIM, 1> Z;

    struct Parameters{
        double l_f; /* Distance from centrer of mass to front axis */
        double k_f; /* Front axis equivalent sideslip stiffness */
        double l_r; /* Distance from centrer of mass to rear axis */
        double k_r; /* Real axis equivalent sideslip stiffness */
        double I_z; /* Yaw inertial of vehicle body */
        double m; /* Mass of the vehicle */
    };
    
    // Implementation of the Dynamic Bicycle Model Mark 1
    // Ready to be used in the EKM as both the measure and prediction model
    class Model: public mirena::StatePredictor<STATE_DIM, CONTROL_DIM>, public mirena::MeasurePredictor<STATE_DIM, MEASURE_DIM>
    {
    public:
    const Parameters _params;

    /////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTORS & FACTORIES
    /////////////////////////////////////////////////////////////////////////////////
 
    Model() = delete; /* The model must be parametrized */
    Model(Parameters params) : _params(params) {}

    // Build a Extended Kalman Filter using this model
    static mirena::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM, MEASURE_DIM> build_ekf();

    /////////////////////////////////////////////////////////////////////////////////
    // INTERFACES
    /////////////////////////////////////////////////////////////////////////////////
    
    // Predict next state based on current state + control input. Corresponds the "f" in theory
    X predict_state(
        const X &previous_state,
        const U &control) override;

    // Get jacobian evaluated at the given state assuming the control vector is constant. Corresponds to "F" in theory
    EMatrix<double, STATE_DIM, STATE_DIM> get_state_jacobian(
        const X &state,
        const U &control) override;

    // Predict next measure based on current state. Corresponds to "h" in theory
    EMatrix<double, MEASURE_DIM, 1> predict_measure(
        const X &state) override;
    
    // Get jacobian evaluated at the given state. Corresponds to "H" in theory
    EMatrix<double, MEASURE_DIM, STATE_DIM> get_measure_jacobian(
        const X &state) override;

    /////////////////////////////////////////////////////////////////////////////////
    // UTILS 
    /////////////////////////////////////////////////////////////////////////////////

    void get_Fy1(const X &state, const U &control, double &output);
    void get_Fy2(const X &state, const U &control, double &output);
    void get_Fy1_partials(const X &state, const U &control, double output[3]);
    void get_Fy2_partials(const X &state, const U &control, double output[3]);
    };

} // namespace mirena

#endif