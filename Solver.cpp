#include "solver.h"
#include <iostream>

using namespace std;

// Solver

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