// Necessary includes to run the program
#include "Solver.h"
#include <iostream>
#include <fstream>

using namespace std;

// Main function that is run which defines input and output function, initialises the solver object and passes in the input and output files
// This main function specifically has all the objects needed to reproduce the graphs in the report
int main() {
    string input_file;
    string output_file;
    
    // Instantiate the Solver
    Solver solver; 
    // Magneto-Mechanical Oscillator input file with RK4
    // Input files are named according to their ODE and integrator and can be renamed into parameters.txt and changed below - can reproduce the graphs in the report
    input_file = "MAG_MECH_RK4.txt";

    // Output file name can be changed below and will appear in same location
    output_file = "output_magRK3.txt";
    solver.run(input_file, output_file);

    return 0;
}