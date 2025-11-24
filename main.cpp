//Necessary includes to run the program
#include "Solver.h"
#include "Solver.cpp"
#include <iostream>
#include <fstream>

using namespace std;

//Main function that is run which defines input and output function, initialises the solver object and passes in the input and output files
int main() {
    //Stores the name of the input and output file
    string input_file = "parameters.txt";
    string output_file = "output_magRK.txt";
    
    // 2. Instantiate and run the solver in which the output is then saved to that speicfic file
    Solver solver;
    solver.run(input_file, output_file);

    return 0;
}