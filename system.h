//The #ifndef SYSTEM_H and #define SYSTEM_H lines prevent the contents of the solver.h header file from being included and processed multiple times during compilation.
#ifndef SYSTEM_H
#define SYSTEM_H

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <numeric>
#include <fstream>
#include <sstream>
#include <map>

using namespace std;
// System Base Class
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

#endif // SYSTEM_H