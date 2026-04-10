#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <fstream>


using namespace std;

void writeLineToCSV(std::string filename, vector<double> inputs, std::string delimiter = ","){
    fstream fout;
    fout.open(filename, ios::out | ios::app);

    // Set fixed-point notation and maximum precision for doubles
    fout << std::fixed;
    fout.precision(3); // You can adjust precision as needed

    for (int i = 0; i < inputs.size() - 1; i++) {
        fout << inputs[i] << delimiter;
    }

    fout << inputs.back() << "\n";
}