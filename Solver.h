#pragma once
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <numeric>
#include <fstream>
#include <sstream>
#include <map>

using namespace std;
// --- System Base Class ---
// y' = f(y, theta, t)

class System {
public:
    // Core ODE function: y' = f(y, theta, t)
    // t is included to handle time-dependent systems like the Magneto-Mechanical Oscillator
    virtual vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const = 0;
    virtual ~System() = default;
};

// --- Child class of the main System class: Problem 25: Magneto-Mechanical Oscillator (2D) ---
class MagnetoMechanical : public System {
public:
    // y = [x, v], theta = [gamma, omega0, alpha, c]
    vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const override;
};

// --- Child class of the main System class: Problem 21: Satellite Attitude Dynamics (3D) ---
class SatelliteAttitude : public System {
public:
    // y = [wx, wy, wz], theta = [Ix, Iy, Iz]
    vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const override;
};

// --- Child class of the main System class: Problem 8: Two-Loop RLC Circuit (4D) ---
class RLC_Circuit : public System {
public:
    // y = [i1, i2, q1, q2], theta = [L1, R1, C1, M, L2, R2, C2] + Vin(t)
    // Assuming V_in(t) is a parameter, can treat as constant for simplicity
    // theta = [L1, R1, C1, M, L2, R2, C2, Vin]
    vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const override;
};


// --- Integrator Base Class ---

class Integrator {
public:
    // Calculates y_n+1 from y_n
    virtual vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const = 0;
    virtual ~Integrator() = default;
};
//Child class of the main Integrator class: overrides the method step
class ForwardEuler : public Integrator {
public:
    vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const override;
};

//Child class of the main Integrator class: overrides the method step
class RungeKutta4 : public Integrator {
public:
    vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const override;
};

// --- Solver Class ---

class Solver {
//Private attributes and method takeInput
private:
    unique_ptr<System> m_system;
    unique_ptr<Integrator> m_integrator;
    vector<double> m_initial_conditions;
    vector<double> m_parameters;
    double m_T_end;
    double m_dt;
    string m_system_name;
    string m_integrator_name;

    // Helper to read and parse the input file
    void takeInput(const string& filename);
//Public run method that can be accessed anywhere 
public:
    Solver() = default;
    void run(const string input_filename, const string output_filename);
};