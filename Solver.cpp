#include "Solver.h"
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

// --- Integrator ---

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

// --- Solver ---
// Method parseInput within the Solver class that handles the input file, error checks with validation of the values and assigns the Solver attributes appropriately
void Solver::takeInput(const string& filename) {
    //Opens the file
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Cannot open parameters.txt");
    }

    string line;

    // Read the first line [ODE] [TimeScheme] [T] [dt]
    //Stores all the values in appropriate private attributes of the Solver class
    if (getline(file, line)) {
        stringstream ss(line);
        ss >> m_system_name >> m_integrator_name >> m_T_end >> m_dt;
        cout << endl;
        cout << "The system you are using is: " << m_system_name << endl;
        cout << "The integrator you are using is: " << m_integrator_name << endl;
        cout << "The time that it is being run for is between 0 and " << m_T_end << " seconds." << endl;
        cout << "It is being stepped by " << m_dt << " seconds." << endl;
        cout << endl;
    } else {
        throw runtime_error("Input file is empty");
    }

    // Process keywords and create objects
    if (m_system_name == "MAGNETO_MECH") {
        m_system = make_unique<MagnetoMechanical>();
    } else if (m_system_name == "SAT_ATTITUDE") {
        m_system = make_unique<SatelliteAttitude>();
    }else if (m_system_name == "RLC_CIRCUIT") {
        m_system = make_unique<RLC_Circuit>();
    } else {
        throw runtime_error("Unknown ODE system: " + m_system_name);
    }

    if (m_integrator_name == "FORWARD_EULER") {
        m_integrator = make_unique<ForwardEuler>();
    } else if (m_integrator_name == "RK4") {
        m_integrator = make_unique<RungeKutta4>();
    } else {
        throw runtime_error("Unknown Time Scheme: " + m_integrator_name);
    }

    // Read second line (Parameters)
    if (getline(file, line)) {
        stringstream ss(line);
        double param;
        while (ss >> param) {
            m_parameters.push_back(param);
        }
    } else {
        throw runtime_error("Missing parameter line in input file");
    }

    // Read third line (Initial Conditions)
    if (getline(file, line)) {
        stringstream ss(line);
        double ic;
        while (ss >> ic) {
            m_initial_conditions.push_back(ic);
        }
    } else {
        throw runtime_error("Missing initial conditions line in input file");
    }
}

// Method run within the Solver class that handles the calculation of the next y values that can be plotted and writes to the output file
void Solver::run(const string input_filename, const string output_filename) {
    //Runs the whole code by first parsing the input
    try {
        takeInput(input_filename);
    } catch (const exception& e) {
        cerr << "Error during setup: " << e.what() << endl;
        return;
    }

    //Checks if output file is writable
    ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        cerr << "Error: Could not write " << output_filename << endl;
        return;
    }

    // Time integration setup
    double t = 0.0;
    vector<double> y_current = m_initial_conditions;
    
    // Main Time-Stepping Loop
    while (t <= m_T_end) {
        // Output current state: [t] [x0_t] [x1_t] ..
        outfile << t;
        for (double val : y_current) {
            outfile << " " << val;
        }
        outfile << "\n";

        // Step the solution forward
        y_current = m_integrator->step(*m_system, y_current, t, m_dt, m_parameters);

        // Update time by dt given in the attributes
        t += m_dt;
    }

    //Closes the file to release resources
    outfile.close();
    cout << "Simulation is complete. The output is written to " << output_filename << endl;
}