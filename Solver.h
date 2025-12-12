#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <numeric>
#include <fstream>
#include <sstream>
#include <map>

#include "system.h"
#include "integrator.h"

using namespace std;

// Solver Class Definition
class Solver {
//Private attributes and method takeInput
private:
    // Polymorphic system and integrator selected at runtime
    unique_ptr<System> m_system;
    unique_ptr<Integrator> m_integrator;
    vector<double> m_initial_conditions; // Initial state values
    vector<double> m_parameters; // System-specific parameters
    double m_T_end; // Simulation end time
    double m_dt; // Time step size
    string m_system_name;  // Identifier for ODE system selection
    string m_integrator_name; // Identifier for time integrator selection

    // Helper to read and parse the input file
    void takeInput(const string& filename);
//Public run method that can be accessed anywhere 
public:
    Solver() = default;
    void run(const string input_filename, const string output_filename);
};