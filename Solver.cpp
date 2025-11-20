#include "Solver.h"
#include <iostream>

using namespace std;
// --- System Implementations ---

// Problem 25: Magneto-Mechanical Oscillator [cite: 87]
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

// Problem 21: Satellite Attitude Dynamics (Torque-Free) [cite: 77]
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

// Problem 9: Bungee Jumper with Nonlinear Damping [cite: 35]
vector<double> BungeeJumper::f(const vector<double>& y, const vector<double>& theta, double t) const {
    double y_pos = y[0];
    double v = y[1];
    
    // theta = [g, k, m, L0, c]
    double g = theta[0];
    double k = theta[1];
    double m = theta[2];
    double L0 = theta[3];
    double c = theta[4];
    
    double dy_dt = v;
    
    double F_spring = -k / m * max(0.0, y_pos - L0); // Spring force
    double F_drag = -c / m * abs(v) * v;          // Quadratic drag
    
    double dv_dt = g + F_spring + F_drag;
    
    return {dy_dt, dv_dt};
}

// Problem 8: Two-Loop RLC Circuit [cite: 32]
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

// --- Integrator Implementations ---

// Forward Euler Scheme
vector<double> ForwardEuler::step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const {
    vector<double> dydt = system.f(y_n, theta, t_n);
    vector<double> y_n_plus_1 = y_n;
    
    for (size_t i = 0; i < y_n.size(); ++i) {
        y_n_plus_1[i] += dt * dydt[i];
    }
    return y_n_plus_1;
}

// Runge-Kutta 4th Order Scheme
vector<double> RungeKutta4::step(const System& system, const vector<double>& y_n, double t_n, double dt, const vector<double>& theta) const {
    size_t N = y_n.size();
    
    // Helper function for vector addition/scaling
    auto add_scaled = [&](const vector<double>& a, const vector<double>& b, double scale) {
        vector<double> result(N);
        for (size_t i = 0; i < N; ++i) {
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
    for (size_t i = 0; i < N; ++i) {
        y_n_plus_1[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    
    return y_n_plus_1;
}

// --- Solver Implementations ---

void Solver::parseInput(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Could not open parameters.txt");
    }

    string line;
    map<string, string> keywords;

    // Read the first line [ODE] [TimeScheme] [T] [dt]
    if (getline(file, line)) {
        stringstream ss(line);
        ss >> system_name_ >> integrator_name_ >> T_end_ >> dt_;
    } else {
        throw runtime_error("Input file is empty.");
    }

    // Process keywords and create objects
    if (system_name_ == "MAGNETO_MECH") {
        system_ = make_unique<MagnetoMechanical>();
    } else if (system_name_ == "SAT_ATTITUDE") {
        system_ = make_unique<SatelliteAttitude>();
    } else if (system_name_ == "BUNGEE_JUMPER") {
        system_ = make_unique<BungeeJumper>();
    } else if (system_name_ == "RLC_CIRCUIT") {
        system_ = make_unique<RLC_Circuit>();
    } else {
        throw runtime_error("Unknown ODE system: " + system_name_);
    }

    if (integrator_name_ == "FORWARD_EULER") {
        integrator_ = make_unique<ForwardEuler>();
    } else if (integrator_name_ == "RK4") {
        integrator_ = make_unique<RungeKutta4>();
    } else {
        throw runtime_error("Unknown Time Scheme: " + integrator_name_);
    }

    // Read second line (Parameters)
    if (getline(file, line)) {
        stringstream ss(line);
        double param;
        while (ss >> param) {
            parameters_.push_back(param);
        }
    } else {
        throw runtime_error("Missing parameter line in input file.");
    }

    // Read third line (Initial Conditions)
    if (getline(file, line)) {
        stringstream ss(line);
        double ic;
        while (ss >> ic) {
            initial_conditions_.push_back(ic);
        }
    } else {
        throw runtime_error("Missing initial conditions line in input file.");
    }
}

void Solver::run(const string input_filename, const string output_filename) {
    try {
        parseInput(input_filename);
    } catch (const exception& e) {
        cerr << "Error during setup: " << e.what() << endl;
        return;
    }

    ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        cerr << "Error: Could not write " << output_filename << endl;
        return;
    }

    // Time integration setup
    double t = 0.0;
    vector<double> y_current = initial_conditions_;
    
    // Main Time-Stepping Loop
    while (t <= T_end_) {
        // Output current state: [t] [x0_t] [x1_t] ...
        outfile << t;
        for (double val : y_current) {
            outfile << " " << val;
        }
        outfile << "\n";

        // Step the solution forward
        y_current = integrator_->step(*system_, y_current, t, dt_, parameters_);

        // Update time
        t += dt_;
    }

    outfile.close();
    cout << "Simulation complete. Results written to " << output_filename << endl;
}