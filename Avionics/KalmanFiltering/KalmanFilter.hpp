#include "MatrixMath.hpp"
#include <cmath>

class KalmanFilter{
    public:
    vector<vector<double>> F; // State Transition Funciton
    vector<vector<double>> B; // Control Input Transition Function
    vector<vector<double>> P; // Covariance Matrix
    vector<vector<double>> Q; // Process Noise Covariance
    vector<vector<double>> H; // Measurement Function
    vector<vector<double>> R; // Measurement Noise Covariance
    vector<vector<double>> S; //
    vector<vector<double>> K; // Kalman Gains

    vector<vector<double>> x; // states

    KalmanFilter(vector<vector<double>> F1, vector<vector<double>> P1, vector<vector<double>> Q1, vector<vector<double>> H1, vector<vector<double>> R1, vector<vector<double>> B1 = vector<vector<double>>(1,vector<double>(1,0))){
        F = F1;
        B = B1;
        P = P1;
        Q = Q1;
        H = H1;
        R = R1;
    };

    void predict(vector<vector<double>> u = vector<vector<double>>(1,vector<double>(1,0))){
        // cout << "Calculating x\n";
        if(B.size() == F.size()){
            x = mat_add(mat_mul(F,x), mat_mul(B,u));
        }
        else{
            x = mat_mul(F,x);
        }
        // cout << "Calculating p\n";
        P = mat_add(mat_mul(F,mat_mul(P,transpose(F))),Q);
    };

    void update(vector<vector<double>> z){
        vector<vector<double>> y(mat_add(z,negative(mat_mul(H,x))));
        K = mat_mul(P,mat_mul(transpose(H), inverseMatrix(mat_add(mat_mul(H,mat_mul(P,transpose(H))),R))));
        x = mat_add(x,mat_mul(K,y));
        P = mat_mul(mat_add(eye(P.size()),negative(mat_mul(K,H))),P);
    };

    void printKF(){
        cout << "x: " << x[0][1] << endl;
        printMatrix(x);
        cout << "\nP: \n";
        printMatrix(P);
    };
};

vector<vector<double>> calculateQ(int dim, double dt){
    return vector<vector<double>>({{0.25*pow(dt,4), 0.5*pow(dt,3)},{0.5*pow(dt,3),pow(dt,2)}});
}

// class UKF{

// }