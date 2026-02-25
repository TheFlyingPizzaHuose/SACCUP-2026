#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <fstream>


using namespace std;

void writeLineToCSV(std::string filename, vector<double> inputs){
    fstream fout;

    fout.open(filename, ios::out | ios::app);

    for (int i = 0; i<inputs.size(); i++){
        fout << inputs[i]<< ",";
    }
    fout << "\n";

}