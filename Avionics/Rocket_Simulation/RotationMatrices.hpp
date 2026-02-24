#include "../KalmanFiltering/MatrixMath.hpp"
#include <cmath>

vector<double> euler2quat(double phi, double theta, double psi){
    double q0 = cos(psi/2)*cos(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);
    double q1 = cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2);
    double q2 = cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2);
    double q3 = sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2);

    vector<double> q = {q0,q1,q2,q3};
    return q;
};

vector<vector<double>> quat2DCM(double q0, double q1, double q2, double q3){
    vector<vector<double>> R = {{2*pow(q0,2)+2*pow(q1,2)-1, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2},
                                {2*q1*q2+2*q0*q3, 2*pow(q0,2)+2*pow(q2,2)-1, 2*q2*q3-2*q0*q1},
                                {2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 2*pow(q0,2)+2*pow(q3,2)-1}};
    return R;               
};

vector<double> rotateQuaternionRate(vector<double> q, vector<vector<double>> omega){
    vector<vector<double>> M1 = {{-q[1], -q[2], -q[3]},
                                 {q[0], -q[3], q[2]},
                                 {q[3], q[0], -q[1]},
                                 {-q[2], q[1], q[0]}};
    vector<vector<double>> result = mat_mul(matrixTimesScaler(0.5, M1), omega);

    vector<double> output = {result[0][0], result[1][0], result[2][0], result[3][0]};
    return output;
}