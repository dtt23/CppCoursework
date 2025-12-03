//Necessary includes to run the program
#include "Solver.h"
#include <iostream>
#include <fstream>

using namespace std;

//Main function that is run which defines input and output function, initialises the solver object and passes in the input and output files
//This main function specifically has all the objects needed to reproduce the graphs in the report
int main() {
    string input_file;
    string output_file;
    
    //Instantiate the Solver
    Solver MMORK4; 
    // 1.1. Magneto-Mechanical Oscillator (RK4)
    input_file = "MAG_MECH_RK4.txt";
    output_file = "output_magRK3.txt";
    MMORK4.run(input_file, output_file);

    Solver MMOFE; 
    // 1.2. Magneto-Mechanical Oscillator (FE)
    input_file = "MAG_MECH_FE.txt";
    output_file = "output_magFE3.txt";
    MMOFE.run(input_file, output_file);

    Solver RLCRK4; 
    // 2.1. RLC Circuit (RK4)
    input_file = "RLC_RK4.txt";
    output_file = "output_rlcRK4.txt";
    RLCRK4.run(input_file, output_file);
    
    Solver RLCFE; 
    // 2.2. RLC Circuit (FE)
    input_file = "RLC_RK4.txt";
    output_file = "output_rlcFE.txt";
    RLCFE.run(input_file, output_file);

    Solver SATRK4; 
    // 3.1. Satellite Attitude (RK4)
    input_file = "SAT_RK4.txt";
    output_file = "output_satRK4.txt";
    SATRK4.run(input_file, output_file);
    
    Solver SATFE; 
    // 3.2. Satellite Attitude (FE)
    input_file = "SAT_FE.txt";
    output_file = "output_satFE.txt";
    SATFE.run(input_file, output_file);  

    return 0;
}