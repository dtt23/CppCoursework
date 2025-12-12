#include "integrator.h"
#include <iostream>

using namespace std;

// Integrator

// Forward Euler Scheme
// Method step within the ForwardEuler class that takes in the parameters and calculates the next y value via the Forward Euler method
vector<double> ForwardEuler::step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const {
    vector<double> dydt = system.f(y_n, theta, t_n);
    vector<double> y_n_plus_1 = y_n;
    
    for (size_t i = 0; i < y_n.size(); i++) {
        y_n_plus_1[i] += dt * dydt[i];
    }
    return y_n_plus_1;
}

// Runge-Kutta 4th Order Scheme
// Method step within the RungeKutta4 class that takes in the parameters and calculates the next y value via the RungeKutta4 method
vector<double> RungeKutta4::step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const {
    size_t N = y_n.size();
    
    //Function for vector addition/scaling
    auto add_scaled = [&](const vector<double>& a, const vector<double>& b, double scale) {
        vector<double> result(N);
        for (size_t i = 0; i < N; i++) {
            result[i] = a[i] + scale * b[i];
        }
        return result;
    };
    
    // K1 = f(y_n, t_n)
    vector<double> k1 = system.f(y_n, theta, t_n);
    
    // K2 = f(y_n + dt/2 * K1, t_n + dt/2)
    vector<double> y_temp_2 = add_scaled(y_n, k1, dt / 2.0);
    vector<double> k2 = system.f(y_temp_2, theta, t_n + dt / 2.0);
    
    // K3 = f(y_n + dt/2 * K2, t_n + dt/2)
    vector<double> y_temp_3 = add_scaled(y_n, k2, dt / 2.0);
    vector<double> k3 = system.f(y_temp_3, theta, t_n + dt / 2.0);
    
    // K4 = f(y_n + dt * K3, t_n + dt)
    vector<double> y_temp_4 = add_scaled(y_n, k3, dt);
    vector<double> k4 = system.f(y_temp_4, theta, t_n + dt);
    
    // y_n+1 = y_n + dt/6 * (K1 + 2*K2 + 2*K3 + K4)
    vector<double> y_n_plus_1 = y_n;
    for (size_t i = 0; i < N; i++) {
        y_n_plus_1[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    
    return y_n_plus_1;
}