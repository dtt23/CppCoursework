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
// --- 1. System Base Class and Derived Systems ---
// y' = f(y, theta, t)

class System {
public:
    // Core ODE function: y' = f(y, theta, t)
    // Note: t is included to handle time-dependent systems like the Magneto-Mechanical Oscillator (Prob 25).
    virtual vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const = 0;
    virtual ~System() = default;
};

// --- Problem 25: Magneto-Mechanical Oscillator (2D) ---
class MagnetoMechanical : public System {
public:
    // y = [x, v], theta = [gamma, omega0, alpha, c]
    vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const override;
};

// --- Problem 21: Satellite Attitude Dynamics (3D) ---
class SatelliteAttitude : public System {
public:
    // y = [wx, wy, wz], theta = [Ix, Iy, Iz]
    vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const override;
};

// --- Problem 8: Two-Loop RLC Circuit (4D) ---
class RLC_Circuit : public System {
public:
    // y = [i1, i2, q1, q2], theta = [L1, R1, C1, M, L2, R2, C2] + Vin(t)
    // Assuming V_in(t) is a parameter, let's treat it as constant V_in for simplicity unless specified otherwise.
    // Let theta = [L1, R1, C1, M, L2, R2, C2, Vin]
    vector<double> f(const vector<double>& y, const vector<double>& theta, double t) const override;
};


// --- 2. Integrator Base Class and Derived Integrators ---

class Integrator {
public:
    // Calculates y_n+1 from y_n
    virtual vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const = 0;
    virtual ~Integrator() = default;
};

class ForwardEuler : public Integrator {
public:
    vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const override;
};

class RungeKutta4 : public Integrator {
public:
    vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const override;
};

// --- 3. Solver Class (Orchestration and I/O) ---

class Solver {
private:
    unique_ptr<System> system_;
    unique_ptr<Integrator> integrator_;
    vector<double> initial_conditions_;
    vector<double> parameters_;
    double T_end_;
    double dt_;
    string system_name_;
    string integrator_name_;

    // Helper to read and parse the input file
    void parseInput(const string& filename);

public:
    Solver() = default;
    void run(const string input_filename, const string output_filename);
};