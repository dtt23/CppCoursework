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
    unique_ptr<System> m_system;
    unique_ptr<Integrator> m_integrator;
    vector<double> m_initial_conditions;
    vector<double> m_parameters;
    double m_T_end;
    double m_dt;
    string m_system_name;
    string m_integrator_name;

    // Helper to read and parse the input file
    void takeInput(const string& filename);
//Public run method that can be accessed anywhere 
public:
    Solver() = default;
    void run(const string input_filename, const string output_filename);
};