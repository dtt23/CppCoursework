//The #ifndef INTEGRATOR_H and #define INTEGRATOR_H lines prevent the contents of the solver.h header file from being included and processed multiple times during compilation.
#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <numeric>
#include <fstream>
#include <sstream>
#include <map>

#include "system.h"

using namespace std;

// Integrator Base Class
class Integrator {
public:
    // Advance the system state by one time step: compute y_{n+1} from y_n
    virtual vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const = 0;
    virtual ~Integrator() = default;
};
//Child class of the main Integrator class: overrides the method step
class ForwardEuler : public Integrator {
public:
    // Perform one integration step using the Forward Euler method
    vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const override;
};

//Child class of the main Integrator class: overrides the method step
class RungeKutta4 : public Integrator {
public:
    // Perform one integration step using classical fourth-order Runge–Kutta (RK4)
    vector<double> step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const override;
};

#endif // INTEGRATOR_H