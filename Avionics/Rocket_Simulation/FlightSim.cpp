#include "RotationMatrices.hpp"
#include <tuple>

double gravity = -9.80665;
// double gravity = 0;

vector<vector<double>> calculateBodyForces(vector<double> quat, double drag, double thrust, double mass){

    // Calculate the forces acting on the aircraft in the body frame
    vector<vector<double>> R = quat2DCM(quat[0],quat[1],quat[2],quat[3]);
    vector<vector<double>> g_w = {{0},{0},{gravity*mass}};
    vector<vector<double>> g = mat_mul(transpose(R),g_w);

    vector<vector<double>> d = {{0},{0},{-drag}};
    vector<vector<double>> t = {{0},{0},{thrust}};

    vector<vector<double>> F_b = mat_add(g,mat_add(d,t));
    return F_b;
}

vector<vector<double>> calculateAcceleration(vector<vector<double>> F_b, double m, vector<vector<double>> omega_b, vector<vector<double>> V_b){
    
    vector<vector<double>> a_rot = {{omega_b[2][0]*V_b[1][0]-omega_b[1][0]*V_b[2][0]},
                                    {omega_b[0][0]*V_b[2][0]-omega_b[2][0]*V_b[0][0]},
                                    {omega_b[1][0]*V_b[0][0]-omega_b[0][0]*V_b[1][0]}};

    vector<vector<double>> a_b = mat_add(matrixTimesScaler(1/m, F_b),a_rot);

    return a_b;
}

vector<vector<double>> calculateRotationalAcceleration(vector<vector<double>> I, vector<vector<double>> omega_b, vector<vector<double>> M_b){
    vector<vector<double>> rot_accel = mat_mul(inverseMatrix(I),mat_add(crossProduct(negative(omega_b),mat_mul(I,omega_b)),M_b));
    return rot_accel;
}

tuple<vector<vector<double>>, vector<vector<double>>, vector<double>, vector<vector<double>>> step(double dt, vector<double> quat, double drag, double thrust, double m, vector<vector<double>> omega_b0, vector<vector<double>> V_b, vector<vector<double>> I, vector<vector<double>> M_b, vector<vector<double>> x_w0){
    vector<vector<double>> Fb = calculateBodyForces(quat,drag,thrust, m);
    vector<vector<double>> a_b = calculateAcceleration(Fb, m, omega_b0, V_b);
    vector<vector<double>> rot_accel = calculateRotationalAcceleration(I,omega_b0, M_b);
    

    vector<vector<double>> R_wb = quat2DCM(quat[0],quat[1],quat[2],quat[3]);

    vector<vector<double>> V_w0 = mat_mul(R_wb,V_b);
    // cout << "Rotation Matrix: \n" << R[0][0] << " " << R[0][1] << " " << R[0][2] << endl;
    // cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << endl;
    // cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << endl; 
    // cout << "V_b = " << V_b[2][0] << " V_w = " << V_w0[2][0] << endl;
    // cout << " Vx: " << V_b[0][0] << " Vy: " << V_b[1][0] << " Vz: " << V_b[2][0] << " Vwx: " << V_w0[0][0] << " Vwy: " << V_w0[1][0] << " Vwz: " << V_w0[2][0] << endl;
    // cout << "Speed Body: " << magnitude(vector<double>{V_b[0][0],V_b[1][0],V_b[2][0]}) << " Speed World: " << magnitude(vector<double>{V_w0[0][0],V_w0[1][0],V_w0[2][0]}) << endl;
    vector<vector<double>> x_new = mat_add(x_w0,matrixTimesScaler(dt,V_w0));

    vector<double> q_rate = rotateQuaternionRate(quat,omega_b0);
    
    vector<double> q_new = vector_add(quat, vectorTimesScaler(dt, q_rate));

    if (magnitude(q_new) > 1.00000001 || magnitude(q_new)<(1-0.00000001)){
        // cout << "QUATERNION NOT VALID\n";
        q_new = vectorTimesScaler(1/magnitude(q_new), q_new);
    }

    vector<vector<double>> omega_new = mat_add(omega_b0, matrixTimesScaler(dt, rot_accel));


    V_b = mat_add(V_b, matrixTimesScaler(dt, a_b));
    // vector<vector<double>> V_w = mat_mul(R,V_b);

    tuple<vector<vector<double>>, vector<vector<double>>, vector<double>, vector<vector<double>>> result = {V_b, x_new, q_new, omega_new};


    return result;

}


int main(){
    // set initial conditions

    vector<vector<double>> vb {{0},{0},{280}};
    vector<vector<double>> x0 {{0},{0},{0}};
    vector<double> rot_angles {0,0,0}; // Rotation angles in Radians
    vector<double> q = euler2quat(rot_angles[0], rot_angles[1], rot_angles[2]);

    vector<vector<double>> omega_b {{0.1},{0},{0}}; // initial rotational rates rad/s
    double mass = 7; // kg
    vector<vector<double>> I = eye(3);
    vector<vector<double>> M_b = {{0},{0},{0}};
    double thrust = 0;
    double drag = 0;
    double dt = 0.01;
    double time = 0;
    tuple<vector<vector<double>>, vector<vector<double>>, vector<double>, vector<vector<double>>> output = {vb, x0, q, omega_b};


    for (int i = 0; i < 57/dt; i++){
        time += dt;
        output = step(dt, q, drag, thrust, mass, omega_b, vb, I, M_b, x0);
        vb = get<0>(output);
        x0 = get<1>(output);
        q = get<2>(output);
        omega_b = get<3>(output);
        if (i%100==0){
            cout << "V_B at t = " << time << ": " << " Vx: " << vb[0][0] << " Vy: " << vb[1][0] << " Vz: " << vb[2][0] << " m/s x: " << x0[0][0] << " y: " << x0[1][0] << " z: " << x0[2][0] <<"m\n";
        }
        

    }




    return 0;
}