#include "MatrixMath.hpp"
#include <cmath>
#include <functional>

class ExtendedKalmanFilter{
    public:
    vector<vector<double>> F; // State Transition Funciton
    vector<vector<double>> B; // Control Input Transition Function
    vector<vector<double>> P; // Covariance Matrix
    vector<vector<double>> Q; // Process Noise Covariance
    vector<vector<double>> H; // Measurement Function
    vector<vector<double>> R; // Measurement Noise Covariance
    vector<vector<double>> S; //
    vector<vector<double>> K; // Kalman Gains

    function<vector<vector<double>>(const vector<vector<double>> state_, const double CD_, const double thrust_, const double area_, const double rho_, const double mass_, const double g_, const double dt_)> Jf;
    function<vector<vector<double>>(uint sensor_id_, vector<vector<double>> states_)> Jh;
    function<vector<vector<double>>(uint sensor_id_, vector<vector<double>> states_)> calcHx;
    function<vector<vector<double>>(vector<vector<double>> states_, const double CD_, const double thrust_, const double area_, const double rho_, const double mass_, const double g_, const double dt_)> calcFx;

    function<vector<vector<double>>(const vector<vector<double>> state_, const double dt_)> Q_calc;

    function<vector<vector<double>>(const uint sensor_id_)> calcR;

    vector<vector<double>> x; // states

    // ExtendedKalmanFilter(function<vector<vector<double>>(const vector<vector<double>> state_, const double CD_, const double thrust_, const double area_, const double rho_, const double mass_, const double g_, const double dt_)> Jf_, vector<vector<double>> F1, vector<vector<double>> P1, vector<vector<double>> Q1, vector<vector<double>> H1, vector<vector<double>> R1, vector<vector<double>> B1 = vector<vector<double>>(1,vector<double>(1,0))){
    //     Jf = Jf_;
    //     F = F1;
    //     B = B1;
    //     P = P1;
    //     Q = Q1;
    //     H = H1;
    //     R = R1;
    // };

    void predict(double thrust, double cd, double area, double rho, double mass, double g, double dt){
        // cout << "Calculating x\n";//<<
        // cout << "In Predict\n";
        // cout << "F is \n" << F.size(); //<< "x" << F[0].size() << " x is " << x.size() << "x" << x[0].size();
        F = Jf(x, cd, thrust, area, rho, mass, g, dt);
        Q = Q_calc(x, dt);

        x = calcFx(x, cd, thrust, area, rho, mass, g, dt);


        // cout << "Calculating x\n";
        // if(B.size() == F.size()){
        //     x = mat_add(mat_mul(F,x), matrixTimesScaler(thrust, B));
        // }
        // else{
        //     x = mat_mul(F,x);
        // }
        // cout << "Calculating P\n";
        P = mat_add(mat_mul(F,mat_mul(P,transpose(F))),Q);
    };

    void update(uint sensor_id_, vector<vector<double>> z){
        vector<vector<double>> Hx = calcHx(sensor_id_, x);
        cout << "H(x):\n";
        printMatrix(Hx);
        cout << "z: \n";
        printMatrix(z);
        H = Jh(sensor_id_, x);
        R = calcR(sensor_id_);
        vector<vector<double>> y(mat_add(z,negative(Hx)));
        cout << "y: \n";
        printMatrix(y);
        K = mat_mul(mat_mul(P,transpose(H)), inverseMatrix(mat_add(mat_mul(H,mat_mul(P,transpose(H))),R)));
        x = mat_add(x,mat_mul(K,y));
        cout << "Ky: \n";
        printMatrix(mat_mul(K,y));

        // for (int i = 0; i<K.size(); i++){
        //     for (int j = 0; j<K[0].size(); j++){
        //         if (K[i][j] > 1 || K[i][j] < 0){
        //             cout << "INVALID KALMAN GAIN AT INDEX " << i << "," << j << endl;
        //             cout << "INVALID GAIN = " << K[i][j] << endl;
        //             cout << "Current Q:\n";
        //             printMatrix(Q);
        //             cout << "Current Kalman Gains\n";
        //             printMatrix(K);
        //             printKF();
        //             throw runtime_error("INVALID KALMAN GAINS");
        //         }
        //     }
        // }
        P = mat_mul(mat_add(eye(P.size()),negative(mat_mul(K,H))),P);
    };

    void printKF(){
        cout << "x: \n";
        printMatrix(x);
        cout << "\nP: \n";
        printMatrix(P);
    };
};

vector<vector<double>> calculateQ2D(int dim, double dt){
    return vector<vector<double>>({{0.25*pow(dt,4), 0.5*pow(dt,3)},{0.5*pow(dt,3),pow(dt,2)}});
}

// class UKF{

// }