#include "translateToCPP.h"
#include <iostream>
// #include <string>
#include <sstream>
#include <vector>

using namespace std;

/*
 * Goal(s): 
 * 
 * - Convert a string of instructions into a vector using ',' as the delimiter.
 * - Create a dictionary that calls functions based on the instructions in the vector
 *       - each instruction should correspond to a function and call it from the dictionary
 */

// Temporary main
int main(int argc, char **argv) {
    string input = "f"; // Placeholder
    string temp; // Place holder for each isntruction
    vector<string> instructions;
    stringstream ss(input);

    while(ss >> temp) {
        
    }
}