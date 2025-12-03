#include "system.h"
#include <iostream>

using namespace std;
// --- Systems ---
// Problem 25: Magneto-Mechanical Oscillator
// Method f within the MagnetoMechanical class that takes in the parameters and calculates the derivates wrt time
vector<double> MagnetoMechanical::f(const vector<double>& y, const vector<double>& theta, double t) const {
    double x = y[0];
    double v = y[1];
    
    // theta = [gamma, omega0, alpha, c]
    double gamma = theta[0];
    double omega0_sq = theta[1] * theta[1]; // Using omega0^2
    double alpha = theta[2];
    double c = theta[3];
    
    double dx_dt = v;
    double dv_dt = -gamma * v - omega0_sq * x + alpha * sin(x - c * t);
    
    return {dx_dt, dv_dt};
}

// Problem 21: Satellite Attitude Dynamics
// Method f within the SatelliteAttitude class that takes in the parameters and calculates the derivates wrt time
vector<double> SatelliteAttitude::f(const vector<double>& y, const vector<double>& theta, double t) const {
    double wx = y[0];
    double wy = y[1];
    double wz = y[2];
    
    // theta = [Ix, Iy, Iz]
    double Ix = theta[0];
    double Iy = theta[1];
    double Iz = theta[2];
    
    double dwx_dt = (Iy - Iz) * wy * wz / Ix;
    double dwy_dt = (Iz - Ix) * wz * wx / Iy;
    double dwz_dt = (Ix - Iy) * wx * wy / Iz;
    
    return {dwx_dt, dwy_dt, dwz_dt};
}

// Problem 8: Two-Loop RLC Circuit
// Method f within the RLC_Circuit class that takes in the parameters and calculates the derivates wrt time
vector<double> RLC_Circuit::f(const vector<double>& y, const vector<double>& theta, double t) const {
    double i1 = y[0];
    double i2 = y[1];
    double q1 = y[2];
    double q2 = y[3];
    
    // theta = [L1, R1, C1, M, L2, R2, C2, Vin]
    double L1 = theta[0];
    double R1 = theta[1];
    double C1 = theta[2];
    double M = theta[3];
    double L2 = theta[4];
    double R2 = theta[5];
    double C2 = theta[6];
    double Vin = theta[7]; // Assuming constant input voltage
    
    // Solving the linear system for di1/dt and di2/dt
    // [ L1  M ] [ di1/dt ] = [ Vin - R1*i1 - q1/C1 ]
    // [ M   L2] [ di2/dt ] = [ - R2*i2 - q2/C2     ]
    double det = L1 * L2 - M * M;
    
    double RHS1 = Vin - R1 * i1 - q1 / C1;
    double RHS2 = -R2 * i2 - q2 / C2;
    
    double di1_dt = (L2 * RHS1 - M * RHS2) / det;
    double di2_dt = (L1 * RHS2 - M * RHS1) / det;
    
    double dq1_dt = i1;
    double dq2_dt = i2;
    
    return {di1_dt, di2_dt, dq1_dt, dq2_dt};
}