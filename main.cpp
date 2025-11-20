#include "Solver.h"
#include <iostream>
#include <fstream>

using namespace std;
// Helper function to create a dummy parameters.txt file for testing
void create_example_input(const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Could not create example input file!" << endl;
        return;
    }
    
    // Example: Satellite Attitude Dynamics (Problem 21) using RK4
    // [ODE] [TimeScheme] [T] [dt]
    // SAT_ATTITUDE RK4 100.0 0.01 
    
    // [param1] [param2] [param3] ... (Ix, Iy, Iz)
    // 10.0 12.0 8.0 
    
    // [ic0] [ic1] [ic2] ... (omega_x0, omega_y0, omega_z0)
    // 0.1 0.0 0.5 
    
    file << "SAT_ATTITUDE RK4 100.0 0.01\n";
    file << "10.0 12.0 8.0\n"; 
    file << "0.1 0.0 0.5\n"; 
    
    file.close();
    cout << "Created example 'parameters.txt' for Satellite Attitude (RK4)." << endl;
}

int main() {
    string input_file = "parameters.txt";
    string output_file = "output.txt";

    // 1. Create a test input file
    create_example_input(input_file);
    
    // 2. Instantiate and run the solver
    Solver solver;
    solver.run(input_file, output_file);

    return 0;
}