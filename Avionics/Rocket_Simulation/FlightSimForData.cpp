#include "RotationMatrices.hpp"
#include "../csvWriter.hpp"

#include <tuple>
#include <functional>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <random>

struct DragEntry {
    double mach;
    double cd;
    double cnalpha;
};

// Parse CSV file and return drag table
std::vector<DragEntry> parseDragCSV(const std::string& filename) {
    std::vector<DragEntry> table;
    std::ifstream file(filename);
    std::string line;
    // Skip header
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cells;
        while (std::getline(ss, cell, ',')) {
            cells.push_back(cell);
        }
        if (cells.size() >= 13) {
            try {
                double mach = std::stod(cells[0]);
                double cd = std::stod(cells[2]);
                double cnalpha = std::stod(cells[12]);
                table.push_back({mach, cd, cnalpha});
                if (mach > 3) break;
            } catch (...) {
                // skip invalid rows
            }
        }
    }
    return table;
}

// Linear interpolation for drag lookup
std::pair<double, double> lookupDrag(double mach, const std::vector<DragEntry>& table) {
    if (table.empty()) {
        return {0, 0};
    }
    // If mach is out of bounds, clamp to ends
    if (mach <= table.front().mach) {
        return {table.front().cd, table.front().cnalpha};
    }
    if (mach >= table.back().mach) {
        return {table.back().cd, table.back().cnalpha};
    }
    // Find interval
    for (size_t i = 1; i < table.size(); ++i) {
        if (mach < table[i].mach) {
            double m0 = table[i-1].mach;
            double m1 = table[i].mach;
            double cd0 = table[i-1].cd;
            double cd1 = table[i].cd;
            double cn0 = table[i-1].cnalpha;
            double cn1 = table[i].cnalpha;
            double t = (mach - m0) / (m1 - m0);
            double cd = cd0 + t * (cd1 - cd0);
            double cnalpha = cn0 + t * (cn1 - cn0);
            return {cd, cnalpha};
        }
    }
    // fallback
    return {table.back().cd, table.back().cnalpha};
}

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

vector<vector<double>> importDrag(string filename){
    vector<vector<double>> dragData;

    ifstream file(filename);

    string line;
    while (getline(file, line)){
        stringstream ss(line);
        string cell;
        vector<double> row;

        while (getline(ss,cell,',')){
            try {
                // Handle potential quotes and convert to double
                // Basic quote handling: if the cell is quoted, remove them
                if (!cell.empty() && cell.front() == '"' && cell.back() == '"') {
                    cell = cell.substr(1, cell.length() - 2);
                }
                // cout << cell<<endl;
                // cout << (std::stod(cell)) << endl;

                row.push_back(std::stod(cell)); // Convert string to double
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid argument for stod: " << cell << std::endl;
                // Handle error or push a default value
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range for stod: " << cell << std::endl;
                // Handle error
            }
        }
        // cout << row[0];
        if (row.size() > 0){
            dragData.push_back(row);
            if (row[0] > 3) break;
        }
    }
    return dragData;
}

double calculateSoS(double gamma, double R, double T){
    double SoS = sqrt(gamma*R*T);
    return SoS;
}

double calculateMachNumber(double V, double SoS){
    double M = V/SoS;
    return M;
}

// Standard atmosphere constants (metric)
const double T0 = 288.15; // K
const double P0 = 101325.0; // Pa
const double rho0 = 1.225; // kg/m^3
const double L = -0.0065; // K/m (lapse rate)
const double R_air = 287.05; // J/(kg·K)
const double g0 = 9.80665; // m/s^2

auto getStandardAtmosphere = [](double h) -> std::pair<double, double> {
    double T = 288.15 + (-0.0065) * h;
    double P = 101325.0 * pow(T / 288.15, -g0 / (R_air * -0.0065));
    double rho = P / (R_air * T);
    return {T, rho};
};

// Main simulation loop using RK4
int main() {
        // Load thrust curve from North_StarV2_Sim.eng
        vector<double> thrust_time = {0.001,0.011,0.021,0.031,0.041,0.051,0.061,0.071,0.081,0.091,0.101,0.111,0.121,0.131,0.141,0.151,0.161,0.171,0.181,0.191,0.201,0.211,0.221,0.231,0.241,0.251,0.261,0.271,0.281,0.291,0.301,0.311,0.321,0.331,0.341,0.351,0.361,0.371,0.381,0.391,0.401,0.411,0.421,0.431,0.441,0.451,0.461,0.471,0.481,0.491,0.501,0.511,0.521,0.531,0.541,0.551,0.561,0.571,0.581,0.591,0.601,0.611,0.621,0.631,0.641,0.651,0.661,0.671,0.681,0.691,0.701,0.711,0.721,0.731,0.741,0.751,0.761,0.771,0.781,0.791,0.801,0.811,0.821,0.831,0.841,0.851,0.861,0.871,0.881,0.891,0.901,0.911,0.921,0.931,0.941,0.951,0.961,0.971,0.981,0.991,1.001,1.011,1.021,1.031,1.041,1.051,1.061,1.071,1.081,1.091,1.101,1.111,1.121,1.131,1.141,1.151,1.161,1.171,1.181,1.191,1.201,1.211,1.221,1.231,1.241,1.251,1.261,1.271,1.281,1.291,1.301,1.311,1.321,1.331,1.341,1.351,1.361,1.371,1.381,1.391,1.401,1.411,1.421,1.431,1.441,1.451,1.461,1.471,1.481,1.491,1.501,1.511,1.521,1.531,1.541,1.551,1.561,1.571,1.581,1.591,1.601,1.611,1.621,1.631,1.641,1.651,1.661,1.671,1.681,1.691,1.701,1.711,1.721,1.731,1.741,1.751,1.761,1.771,1.781,1.791,1.801,1.811,1.821,1.831,1.841,1.851,1.861,1.871,1.881,1.891,1.901,1.911,1.921,1.931,1.941,1.951,1.961,1.971,1.981,1.991,2.001,2.011,2.021,2.031,2.041,2.051,2.061,2.071,2.081,2.091,2.101,2.111,2.121,2.131,2.141,2.151,2.161,2.171,2.181,2.191,2.201,2.211,2.221,2.231,2.241,2.251,2.261,2.271,2.281,2.291,2.301,2.311,2.321,2.331,2.341,2.351,2.361,2.371,2.381,2.391,2.401,2.411,2.421,2.431,2.441,2.451,2.461,2.471,2.481,2.491,2.501,2.511,2.521,2.531,2.541,2.551,2.561,2.571,2.581,2.591,2.601,2.611,2.621,2.631,2.641,2.651,2.661,2.671,2.681,2.691,2.701,2.711,2.721,2.731,2.741,2.751,2.761,2.771,2.781,2.791,2.801,2.811,2.821,2.831,2.841,2.851,2.861,2.871,2.881,2.891,2.901,2.911,2.921,2.931,2.941,2.951,2.961,2.971,2.981,2.991,3.001,3.011,3.021,3.031,3.041,3.051,3.061,3.071,3.081,3.091,3.101,3.111,3.121,3.131,3.141,3.151,3.161,3.171,3.181,3.191,3.201,3.211,3.221,3.231,3.241,3.251,3.261,3.271,3.281,3.291,3.301,3.311,3.321,3.331,3.341,3.351,3.361,3.371,3.381,3.391,3.401,3.411,3.421,3.431,3.441,3.451,3.461,3.471,3.481,3.491,3.501,3.511,3.521,3.531,3.541,3.551,3.561,3.571,3.581,3.591,3.601,3.611,3.621,3.631,3.641,3.651,3.661,3.671,3.681,3.691,3.701,3.711,3.721,3.731,3.741,3.751,3.761,3.771,3.781,3.791,3.801,3.811,3.821,3.831,3.841,3.851,3.861,3.871,3.881,3.891,3.901,3.911,3.921,3.931,3.941,3.951,3.961,3.971,3.981,3.991,4.001,4.011,4.021,4.031,4.041,4.051,4.061,4.071,4.081,4.091,4.101,4.111,4.121,4.131,4.141,4.151,4.161,4.171,4.181,4.191,4.201,4.211,4.221,4.231,4.241,4.251,4.261,4.271,4.281,4.291,4.301,4.311,4.321,4.331,4.341,4.351,4.361,4.371,4.381,4.391,4.401,4.411,4.421,4.431,4.441,4.451,4.461,4.471,4.481,4.491,4.501,4.511,4.521,4.531,4.541,4.551,4.561,4.571,4.581,4.591,4.601,4.611,4.621,4.631,4.641,4.651,4.661,4.671,4.681,4.691,4.701,4.711,4.721,4.731,4.741,4.751,4.761,4.771,4.781,4.791,4.801,4.811,4.821,4.831,4.841,4.851,4.861,4.871,4.881,4.891,4.901,4.911,4.921,4.931,4.941,4.951,4.961,4.971,4.981,4.991,5.001,5.011,5.021,5.031,5.041,5.051,5.061,5.071,5.081,5.091,5.101,5.111,5.121,5.131,5.141,5.151,5.161,5.171,5.181,5.191,5.201,5.211,5.221,5.231,5.241,5.251,5.261,5.271,5.281,5.291,5.301,5.311,5.321,5.331,5.341,5.351,5.361,5.371,5.381,5.391,5.401,5.411,5.421,5.431,5.441,5.451,5.461,5.471,5.481,5.491,5.501,5.511,5.521,5.531,5.541,5.551,5.561,5.571,5.581,5.591,5.601,5.611,5.621,5.631,5.641,5.651,5.661,5.671,5.681,5.691,5.701,5.711,5.721,5.731,5.741,5.751,5.761,5.771,5.781,5.791,5.801,5.811,5.821,5.831,5.841,5.851,5.861,5.871,5.881,5.891,5.901,5.911,5.921,5.931,5.941,5.951,5.961,5.971,5.981,5.991,6.001,6.011,6.021,6.031,6.041,6.051,6.061,6.071,6.081,6.091,6.101,6.111,6.121,6.131,6.141,6.151,6.161,6.171,6.181,6.191,6.201,6.211,6.221,6.231,6.241,6.251,6.261,6.271,6.281,6.291,6.301,6.311,6.321,6.331,6.341,6.351,6.361,6.371,6.381,6.391,6.401,6.411,6.421,6.431,6.441,6.451,6.461,6.471,6.481,6.491,6.501,6.511,6.521,6.531,6.541,6.551,6.561,6.571,6.581,6.591,6.601,6.611,6.621,6.631,6.641,6.651,6.661,6.671,6.681,6.691,6.701,6.711,6.721,6.731,6.741,6.751,6.761,6.771,6.781,6.791,6.801,6.811,6.821,6.831,6.841,6.851,6.861,6.871,6.881,6.891,6.901,6.911,6.921,6.931,6.941,6.951,6.961,6.971,6.981,6.991,7.001,7.011,7.021,7.031,7.041,7.051,7.061,7.071,7.081,7.091,7.101,7.111,7.121,7.131,7.141,7.151,7.161,7.171,7.181,7.191,7.201,7.211,7.221,7.231,7.241,7.251,7.261,7.271,7.281,7.291,7.301,7.311,7.321,7.331,7.341,7.351,7.361,7.371,7.381,7.391,7.401,7.411,7.421,7.431,7.441,7.451,7.461,7.471,7.481,7.491,7.501,7.511,7.521,7.531,7.541,7.551,7.561,7.571,7.581,7.591,7.601,7.611,7.621,7.631,7.641,7.651,7.661,7.671,7.681,7.691,7.701,7.711,7.721,7.731,7.741,7.751,7.761,7.771,7.781,7.791,7.801,7.811,7.821,7.831,7.841,7.851,7.861,7.871,7.881,7.891,7.901,7.911,7.921,7.931,7.941,7.951,7.961,7.971,7.981,7.991,8.001,8.011,8.021,8.031,8.041,8.051,8.061,8.071,8.081,8.091,8.101,8.111,8.121,8.131,8.141,8.151,8.161,8.171,8.181,8.191,8.201,8.211,8.221,8.231,8.241,8.251,8.261,8.271,8.281,8.291,8.301,8.311,8.321,8.331,8.341,8.351,8.361,8.371,8.381,8.391,8.401,8.411,8.421,8.431,8.441,8.451,8.461,8.471,8.481,8.491,8.501,8.511,8.521,8.531,8.541,8.551,8.561,8.571,8.581,8.591,8.601,8.611,8.621,8.631,8.641,8.651,8.661,8.671,8.681,8.691,8.701,8.711,8.721,8.731,8.741,8.751,8.761,8.771,8.781,8.791,8.801,8.811,8.821,8.831,8.841,8.851,8.861,8.871,8.881,8.891,8.901,8.911,8.921,8.931,8.941,8.951,8.961,8.971,8.981,8.991,9.001,9.011,9.021,9.031,9.041,9.051,9.061,9.071,9.081,9.091,9.101,9.111,9.121,9.131,9.141,9.151,9.161,9.171,9.181,9.191,9.201,9.211,9.221,9.231,9.241,9.251,9.261,9.271,9.281,9.291,9.301,9.311,9.321,9.331,9.341,9.351,9.361,9.371,9.381,9.391,9.401,9.411,9.421,9.431,9.441,9.451,9.461,9.471,9.481,9.491,9.501,9.511,9.521,9.531,9.541,9.551,9.561,9.571,9.581,9.591,9.601,9.611,9.621,9.631,9.641,9.651,9.661,9.671,9.681,9.691,9.701,9.711,9.721,9.731,9.741,9.751,9.761};
        vector<double> thrust_data = {31,381,697,987,1250,1491,1711,1911,2093,2258,2408,2544,2668,2779,2880,2971,3053,3126,3193,3252,3305,3352,3394,3431,3464,3493,3518,3541,3560,3577,3591,3603,3613,3622,3629,3634,3638,3641,3643,3644,3644,3644,3643,3641,3639,3636,3633,3630,3626,3622,3617,3613,3608,3603,3598,3593,3587,3582,3576,3571,3565,3559,3553,3547,3541,3536,3530,3524,3518,3512,3506,3500,3494,3488,3482,3476,3470,3464,3458,3452,3446,3440,3435,3429,3423,3417,3412,3406,3400,3395,3389,3383,3378,3372,3367,3361,3356,3350,3345,3339,3334,3329,3323,3318,3313,3308,3302,3297,3292,3287,3282,3277,3272,3267,3262,3257,3252,3247,3242,3237,3232,3227,3223,3218,3213,3208,3203,3199,3194,3189,3185,3180,3176,3171,3166,3162,3157,3153,3148,3144,3140,3135,3131,3126,3122,3117,3113,3108,3104,3100,3095,3091,3086,3082,3078,3073,3069,3065,3061,3056,3052,3048,3044,3040,3035,3031,3027,3023,3019,3015,3011,3007,3003,2999,2995,2991,2987,2983,2979,2975,2971,2967,2963,2959,2955,2952,2948,2944,2940,2936,2933,2929,2925,2921,2918,2914,2910,2907,2903,2899,2896,2892,2889,2885,2882,2878,2875,2871,2868,2864,2861,2857,2854,2850,2846,2842,2838,2834,2830,2826,2823,2819,2815,2811,2807,2803,2799,2795,2791,2787,2783,2779,2776,2772,2768,2764,2760,2757,2753,2749,2745,2742,2738,2734,2731,2727,2724,2720,2716,2713,2709,2706,2702,2699,2695,2692,2689,2685,2682,2678,2675,2672,2668,2665,2662,2659,2655,2652,2649,2646,2643,2639,2636,2633,2630,2627,2624,2621,2618,2614,2611,2608,2605,2602,2599,2596,2593,2590,2587,2584,2581,2579,2576,2573,2570,2567,2564,2561,2558,2556,2553,2550,2547,2544,2542,2539,2536,2533,2530,2528,2525,2522,2520,2517,2514,2511,2509,2506,2503,2501,2498,2495,2493,2490,2488,2485,2482,2480,2477,2475,2472,2470,2467,2464,2462,2459,2457,2454,2452,2449,2447,2444,2442,2440,2437,2435,2432,2430,2428,2425,2423,2420,2418,2416,2413,2411,2409,2406,2404,2402,2399,2397,2395,2393,2390,2388,2386,2384,2381,2379,2377,2375,2372,2370,2368,2366,2364,2362,2359,2357,2355,2353,2351,2349,2347,2344,2342,2340,2338,2336,2334,2332,2330,2328,2326,2324,2321,2319,2317,2315,2313,2311,2309,2307,2305,2303,2301,2299,2297,2295,2293,2291,2289,2287,2285,2283,2282,2280,2278,2276,2274,2272,2270,2268,2266,2264,2262,2261,2259,2257,2255,2253,2251,2249,2248,2246,2244,2242,2240,2238,2237,2235,2233,2231,2229,2228,2226,2224,2222,2221,2219,2217,2215,2214,2212,2210,2208,2207,2205,2203,2202,2200,2198,2196,2195,2193,2191,2190,2188,2186,2185,2183,2181,2180,2178,2176,2175,2173,2172,2170,2168,2167,2165,2163,2162,2160,2159,2157,2155,2154,2152,2151,2149,2148,2146,2144,2143,2141,2140,2138,2137,2135,2134,2132,2131,2129,2127,2126,2124,2123,2121,2120,2118,2117,2115,2114,2112,2111,2109,2108,2107,2105,2104,2102,2101,2099,2098,2096,2095,2093,2092,2090,2089,2087,2086,2085,2083,2082,2080,2079,2077,2076,2075,2073,2072,2070,2069,2068,2066,2065,2063,2062,2061,2059,2058,2057,2055,2054,2052,2051,2050,2048,2047,2046,2044,2043,2042,2040,2039,2038,2036,2035,2034,2032,2031,2030,2028,2027,2026,2024,2023,2022,2021,2019,2018,2017,2015,2014,2013,2012,2010,2009,2008,2007,2005,2004,2003,2001,2000,1999,1998,1997,1995,1994,1993,1992,1990,1989,1988,1987,1985,1984,1983,1982,1981,1979,1978,1977,1976,1975,1973,1972,1971,1970,1969,1967,1966,1965,1964,1963,1962,1960,1959,1958,1957,1956,1955,1953,1952,1951,1950,1949,1948,1947,1945,1944,1943,1942,1941,1940,1939,1938,1936,1935,1934,1933,1932,1931,1930,1929,1927,1926,1925,1924,1923,1922,1921,1920,1919,1918,1917,1915,1914,1913,1912,1911,1910,1909,1908,1907,1906,1905,1904,1903,1902,1901,1900,1898,1897,1896,1895,1894,1893,1892,1891,1890,1889,1888,1887,1886,1885,1884,1883,1882,1881,1880,1879,1878,1877,1876,1875,1874,1873,1872,1871,1870,1869,1868,1867,1866,1865,1864,1863,1862,1861,1860,1859,1858,1857,1856,1855,1854,1853,1852,1851,1850,1849,1848,1847,1846,1845,1844,1843,1842,1770,1699,1634,1573,1517,1465,1416,1369,1326,1285,1246,1210,1175,1143,1112,1083,1055,1029,1004,980,958,936,916,896,878,860,843,827,812,797,783,770,757,744,732,721,710,699,689,679,670,661,652,643,635,627,619,612,605,598,591,584,578,571,565,559,553,548,542,537,531,526,521,516,511,507,502,497,493,488,484,480,476,471,467,463,459,455,451,447,444,440,436,432,429,425,421,418,414,411,407,404,400,397,394,390,387,384,381,377,374,371,368,365,362,359,356,353,350,347,344,341,338,336,333,330,327,325,322,319,316,314,311,309,306,304,301,299,296,294,291,289,286,284,282,279,277,275,272,270,268,265,263,261,259,257,255,252,250,248,246,244,242,240,238,236,234,232,230,228,226,224,222,220,218,217,215,213,211,209,207,206,204,202,200,199,197,195,193,192,190,188,187,185,184,182,180,179,177,176,174,173,171,170,168,167,165,164,162,161,159,158,156,155,154,152,151,149,148,147,145,144,143,141,140,139,138,136,135,134,133,131,130,129,128,126,125,124,123,122,121,119,118,117,116,115,114,113,112,110,109,108,107,106,105,104,103,102,101,100,99,98,97,97,96,95,94,93,92,91,90,89,88,88,87,86,85,84,83,83,82,81,80,79,79,78,77,76,76,75,74,73,73,72,71,70,70,69,68,68,67,66,65,65,64,63,63,62,61,61,60,59,59,58,58,57,56,56,55,54,54,53,53,52,51,51,50,50,49,48,48,47,47,46,46,45,45,44,43,43,42,42,41,41,40,40,39,39,38,38,37,37,36,36,35,35,34,34,33,33,32,32,31,31,30,30,30,29,29,28,28,27,27,26,26,26,25,25,24,24,23,23,23,22,22,21,21,21,20,20,19,19,19,18,18,17,17,17,16,16,16,15,15,15,14,14,13,13,13,12,12,12,11,11,11,10,10,10,9,9,9,8,8,8,7,7,7,6,6,6,5,5,5,4,4,4,4,3,3,3,2,2,2,2,1};
        vector<DragEntry> dragData = parseDragCSV("../CD Test.CSV");
        // cout << dragData[1].size();
        // Linear interpolation function for thrust
        auto interpolate_thrust = [&](double t) {
            if (t <= thrust_time.front()) return thrust_data.front();
            if (t >= thrust_time.back()) return thrust_data.back();
            for (size_t i = 1; i < thrust_time.size(); ++i) {
                if (t < thrust_time[i]) {
                    double t0 = thrust_time[i-1];
                    double t1 = thrust_time[i];
                    double f0 = thrust_data[i-1];
                    double f1 = thrust_data[i];
                    return f0 + (f1-f0)*(t-t0)/(t1-t0);
                }
            }
            return thrust_data.back();
        };
    // set initial conditions
    vector<vector<double>> vb {{0},{0},{0}};
    vector<vector<double>> x0 {{0},{0},{0}};
    // vector<double> rot_angles {4*3.1415926535/180,0,0}; // Rotation angles in Radians
    vector<double> rot_angles {0,0,0}; // Rotation angles in Radians
    vector<double> q = euler2quat(rot_angles[0], rot_angles[1], rot_angles[2]);
    vector<vector<double>> omega_b {{0},{0},{0}}; // initial rotational rates rad/s
    double mass = 49.7; // kg
    vector<vector<double>> I = eye(3);
    vector<vector<double>> M_b = {{0},{0},{0}};
    double thrust = 0;
    double drag = 0;
    double CNa, CD;
    double dt = 0.01;
    double time = 0;
    pair<double, double> CD_CNa;
    double rho = 1.19489;
    double gamma = 1.4;
    double T = 20+273.15;
    double R = 286;
    double A = 0.0182441469;
    double mach = 0;
    double SoS = calculateSoS(gamma, R, T);


    // Sensor output setup
    #include <random>
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise_acc(0.0, 0.02); // 0.02g noise
    std::normal_distribution<> noise_gyro(0.0, 0.01); // 0.01 rad/s noise
    std::normal_distribution<> noise_press(0.0, 1.0); // 1 Pa noise
    std::normal_distribution<> noise_alt(0.0, 0.1); // 0.1 m noise
    std::uniform_int_distribution<> jitter(-500, 500); // +/- 500us jitter

    // Sensor output intervals (us)
    int acc_interval = 16667; // 60Hz
    int gyro_interval = 16667; // 60Hz
    int press_interval = 16667; // 60Hz
    int alt_interval = 16667; // 60Hz

    int64_t sim_time_us = 0;
    int64_t next_acc = 0, next_gyro = 0, next_press = 0, next_alt = 0;

    std::string sensor_file = "SimulatedSensors.txt";
    // Clear file at start
    std::ofstream clearfile(sensor_file, std::ios::trunc);
    clearfile.close();

    for (int i = 0; i < 57/dt; i++) {
        time += dt;
        sim_time_us = static_cast<int64_t>(time * 1e6);

        // Update thrust from .eng curve
        thrust = interpolate_thrust(time);

        // Update T and rho based on altitude (height = x0[2][0])
        double height = x0[2][0]+848;
        auto [T_atm, rho_atm] = getStandardAtmosphere(height);
        T = T_atm;
        rho = rho_atm;
        R = R_air;

        SoS = calculateSoS(gamma, R, T);
        mach = calculateMachNumber(magnitude({vb[0][0], vb[1][0],vb[2][0]}), SoS);
        CD_CNa = lookupDrag(mach,dragData);
        CD = CD_CNa.first;
        CNa = CD_CNa.second;
        if (vb[2][0] > 0){
            drag = 0.5*rho*A*pow(magnitude({vb[0][0], vb[1][0],vb[2][0]}),2);
        }
        else{
            drag = 0;
        }
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
        if (x0[2][0] <0){
            x0[2][0] =0;
        }
        if (i%100==0) {
            cout << "V_B at t = " << time << ": " << " Vx: " << vb[0][0] << " Vy: " << vb[1][0] << " Vz: " << vb[2][0] << " m/s x: " << x0[0][0] << " y: " << x0[1][0] << " z: " << x0[2][0] <<"m\n";
        }
        if (mass > 42){
            mass -= 1.83333*dt;
        }
        vector<double> outputCSV = {time, x0[0][0], x0[1][0], x0[2][0], vb[0][0], vb[1][0], vb[2][0], q[0], q[1], q[2], q[3], omega_b[0][0], omega_b[1][0], omega_b[2][0]};
        writeLineToCSV("SimTrajectory.csv", outputCSV);

        // Simulate sensor readings at 60Hz with jitter and noise, format: timestamp|sensor_id|data...
        // 2: baro, 3: accel, 5: gyro, 6: altimeter
        // Baro (pressure sensor)
        if (sim_time_us >= next_press) {
            int64_t ts = sim_time_us + jitter(gen);
            double temperature = T; // in Kelvin
            double pressure = 101325.0 * pow(T / 288.15, -g0 / (R_air * -0.0065)) + noise_press(gen); // simple baro model 101325.0 * pow(T / 288.15, -g0 / (R_air * -0.0065))
            std::vector<double> row = {double(ts), 2, pressure, temperature};
            writeLineToCSV(sensor_file, row, "|");
            next_press += press_interval;
        }
        else if (sim_time_us >= next_acc) { // Accel (3-axis)
            int64_t ts = sim_time_us + jitter(gen);
            // Calculate acceleration in body frame (including gravity offset)
            // a_b = dvb/dt + [0,0,g] (in body frame)
            // Use the most recent RK4 step for acceleration
            vector<vector<double>> Fb = calculateBodyForces(q, drag, thrust, mass);
            vector<vector<double>> acc_b = calculateAcceleration(Fb, mass, omega_b, vb); // 3x1
            // Add gravity offset in body frame
            // Compute gravity vector in world frame
            std::vector<std::vector<double>> g_w = {{0},{0},{-gravity}};
            // Compute rotation matrix from quaternion (q)
            std::vector<std::vector<double>> R = quat2DCM(q[0], q[1], q[2], q[3]);
            // Transform gravity to body frame
            std::vector<std::vector<double>> g_b = mat_mul(transpose(R), g_w);
            double ax = acc_b[0][0] + g_b[0][0] + noise_acc(gen);
            double ay = acc_b[1][0] + g_b[1][0] + noise_acc(gen);
            double az = acc_b[2][0] + g_b[2][0] + noise_acc(gen);
            // Output roll, pitch, yaw rates (body frame)
            double roll_rate = omega_b[0][0]+ noise_gyro(gen);
            double pitch_rate = omega_b[1][0]+ noise_gyro(gen);
            double yaw_rate = omega_b[2][0]+ noise_gyro(gen);
            std::vector<double> row = {double(ts), 3, ax, ay, az, roll_rate, pitch_rate, yaw_rate};
            // std::vector<double> row = {double(ts), 5, ax, ay, az};
            writeLineToCSV(sensor_file, row, "|");
            next_acc += acc_interval;
        }
        else if (sim_time_us >= next_gyro) { // Gyro (3-axis)
            int64_t ts = sim_time_us + jitter(gen);
            double gx = omega_b[0][0] + noise_gyro(gen);
            double gy = omega_b[1][0] + noise_gyro(gen);
            double gz = omega_b[2][0] + noise_gyro(gen);
            std::vector<double> row = {double(ts), 5, gx, gy, gz};
            writeLineToCSV(sensor_file, row, "|");
            next_gyro += gyro_interval;
        }
        // // Altimeter
        // if (sim_time_us >= next_alt) {
        //     int64_t ts = sim_time_us + jitter(gen);
        //     double alt = x0[2][0] + noise_alt(gen);
        //     std::vector<double> row = {double(ts), 6, -1.0, alt};
        //     writeLineToCSV(sensor_file, row, "|");
        //     next_alt += alt_interval;
        // }
    }
    return 0;
}
