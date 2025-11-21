#include "Solver.h"
#include "Solver.cpp"
#include <iostream>
#include <fstream>

using namespace std;

//Main function that is run which defines input and output function, initialises the solver object and passes in the input and output files
int main() {
    string input_file = "parameters.txt";
    string output_file = "output.txt";
    
    // 2. Instantiate and run the solver
    Solver solver;
    solver.run(input_file, output_file);

    return 0;
}