#include "RotationMatrices.hpp"
#include <tuple>
#include <functional>

// RK4 helper for vector<double>
vector<double> rk4_step_vec(const vector<double>& y, double dt, const std::function<vector<double>(const vector<double>&)>& dydt) {
    vector<double> k1 = dydt(y);
    vector<double> y2 = vector_add(y, vectorTimesScaler(dt/2.0, k1));
    vector<double> k2 = dydt(y2);
    vector<double> y3 = vector_add(y, vectorTimesScaler(dt/2.0, k2));
    vector<double> k3 = dydt(y3);
    vector<double> y4 = vector_add(y, vectorTimesScaler(dt, k3));
    vector<double> k4 = dydt(y4);
    vector<double> result(y.size());
    for (size_t i = 0; i < y.size(); ++i) {
        result[i] = y[i] + dt/6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
    return result;
}

// RK4 helper for vector<vector<double>>
vector<vector<double>> rk4_step_mat(const vector<vector<double>>& y, double dt, const std::function<vector<vector<double>>(const vector<vector<double>>&)> &dydt) {
    vector<vector<double>> k1 = dydt(y);
    vector<vector<double>> y2 = mat_add(y, matrixTimesScaler(dt/2.0, k1));
    vector<vector<double>> k2 = dydt(y2);
    vector<vector<double>> y3 = mat_add(y, matrixTimesScaler(dt/2.0, k2));
    vector<vector<double>> k3 = dydt(y3);
    vector<vector<double>> y4 = mat_add(y, matrixTimesScaler(dt, k3));
    vector<vector<double>> k4 = dydt(y4);
    vector<vector<double>> result = y;
    for (size_t i = 0; i < y.size(); ++i) {
        for (size_t j = 0; j < y[0].size(); ++j) {
            result[i][j] += dt/6.0 * (k1[i][j] + 2*k2[i][j] + 2*k3[i][j] + k4[i][j]);
        }
    }
    return result;
}

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

// Main simulation loop using RK4
int main() {
    // set initial conditions
    vector<vector<double>> vb {{0},{0},{280}};
    vector<vector<double>> x0 {{0},{0},{0}};
    vector<double> rot_angles {0,0,0}; // Rotation angles in Radians
    vector<double> q = euler2quat(rot_angles[0], rot_angles[1], rot_angles[2]);
    vector<vector<double>> omega_b {{0},{0},{0}}; // initial rotational rates rad/s
    double mass = 7; // kg
    vector<vector<double>> I = eye(3);
    vector<vector<double>> M_b = {{0},{0},{0}};
    double thrust = 0;
    double drag = 0;
    double dt = 0.01;
    double time = 0;

    for (int i = 0; i < 57/dt; i++) {
        time += dt;
        // RK4 for vb
        auto vb_dydt = [&](const vector<vector<double>>& vb_) {
            vector<vector<double>> Fb = calculateBodyForces(q, drag, thrust, mass);
            return calculateAcceleration(Fb, mass, omega_b, vb_);
        };
        vb = rk4_step_mat(vb, dt, vb_dydt);

        // RK4 for omega_b
        auto omega_dydt = [&](const vector<vector<double>>& omega_) {
            return calculateRotationalAcceleration(I, omega_, M_b);
        };
        omega_b = rk4_step_mat(omega_b, dt, omega_dydt);

        // RK4 for quaternion
        auto quat_dydt = [&](const vector<double>& q_) {
            return rotateQuaternionRate(q_, omega_b);
        };
        q = rk4_step_vec(q, dt, quat_dydt);
        if (magnitude(q) > 1.00000001 || magnitude(q)<(1-0.00000001)) {
            q = vectorTimesScaler(1/magnitude(q), q);
        }

        // RK4 for position
        auto x_dydt = [&](const vector<vector<double>>& x_) {
            vector<vector<double>> R_wb = quat2DCM(q[0],q[1],q[2],q[3]);
            vector<vector<double>> V_w0 = mat_mul(R_wb, vb);
            return V_w0;
        };
        x0 = rk4_step_mat(x0, dt, x_dydt);

        if (i%100==0) {
            cout << "V_B at t = " << time << ": " << " Vx: " << vb[0][0] << " Vy: " << vb[1][0] << " Vz: " << vb[2][0] << " m/s x: " << x0[0][0] << " y: " << x0[1][0] << " z: " << x0[2][0] <<"m\n";
        }
    }
    return 0;
}
