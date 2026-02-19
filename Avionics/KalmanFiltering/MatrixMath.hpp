/*  
Saint Louis University Rocket Propulsion Laboratory (SLURPL)
Avionics Main Code
Authors: Michael Brady

This library is designed to assist with Kalman filtering for Rocket Altitude*/
#include <vector>
#include <stdexcept>
#include <iostream>
using namespace std;


vector<vector<double>> mat_mul(vector<vector<double>> A, vector<vector<double>> B){
    if (A.empty() || B.empty() || A[0].size() != B.size()){
        throw runtime_error("Matrix sizes do not match");
    }

    vector<vector<double>> result(A.size(), vector<double>(B[0].size(), 0.0));

    for (size_t i = 0; i < A.size(); i++) {
        for (size_t j = 0; j < B[0].size(); j++) {
            for (size_t k = 0; k < B.size(); k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
};

vector<vector<double>> mat_add(vector<vector<double>> A, vector<vector<double>> B){
    if (A.empty() || B.empty() || A.size() != B.size() || A[0].size() != B[0].size()){
        throw runtime_error("Matrix sizes do not match");
    }

    vector<vector<double>> result(A.size(), vector<double>(A[0].size(), 0.0));

    for (size_t i = 0; i < A.size(); i++){
        for (size_t j = 0; j < A[0].size(); j++){
            result[i][j] = A[i][j] + B[i][j];
        }
    }
    return result;
};

vector<vector<double>> transpose(vector<vector<double>> A){
    vector<vector<double>> result(A[0].size(), vector<double>(A.size(), 0.0));

    for (size_t i = 0; i < A.size(); i++){
        for (size_t j = 0; j < A[0].size(); j++){
            result[j][i] = A[i][j];
        }
    }
    return result;
};

vector<vector<double>> negative(vector<vector<double>> A){
    for (size_t i = 0; i < A.size(); i++){
        for (size_t j = 0; j < A[0].size(); j++){
            A[i][j] = -1*A[i][j];
        }
    }
    return A;
};

vector<vector<double>> eye(int n){
    vector<vector<double>> I(n, vector<double>(n, 0.0));
    for (int i = 0; i<n; i++){
        I[i][i] = 1;
    }
    return I;
};

vector<double> row_add(int n, vector<double> row){
    for (size_t i = 0; i < row.size(); i++){
        row[i] += n;
    }
    return row;
};


// Function to find the inverse of a matrix using Gauss-Jordan elimination
vector<vector<double>> inverseMatrix(vector<vector<double>> A) {
    int n = A.size();
    if (A[0].size() != n) {
        throw runtime_error("Matrix must be square.");
    }

    // Create an augmented matrix [A | I]
    vector<vector<double>> augmentedMatrix(n, vector<double>(2 * n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            augmentedMatrix[i][j] = A[i][j];
        }
        augmentedMatrix[i][i + n] = 1.0;
    }

    // Apply Gauss-Jordan elimination
    for (int i = 0; i < n; ++i) {
        // Find pivot
        int pivotRow = i;
        for (int k = i + 1; k < n; ++k) {
            if (abs(augmentedMatrix[k][i]) > abs(augmentedMatrix[pivotRow][i])) {
                pivotRow = k;
            }
        }
        swap(augmentedMatrix[i], augmentedMatrix[pivotRow]);

        // Check for singularity
        if (augmentedMatrix[i][i] == 0.0) {
            throw runtime_error("Matrix is singular and cannot be inverted.");
        }

        // Normalize the pivot row
        double pivot = augmentedMatrix[i][i];
        for (int k = i; k < 2 * n; ++k) {
            augmentedMatrix[i][k] /= pivot;
        }

        // Eliminate other rows
        for (int k = 0; k < n; ++k) {
            if (k != i) {
                double factor = augmentedMatrix[k][i];
                for (int j = 0; j < 2 * n; ++j) {
                    augmentedMatrix[k][j] -= factor * augmentedMatrix[i][j];
                }
            }
        }
    }

    // Extract the inverse matrix from the right half [I | A^-1]
    vector<vector<double>> inverse(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            inverse[i][j] = augmentedMatrix[i][j + n];
        }
    }
    return inverse;
};

void printMatrix(vector<vector<double>> A){
    int m = A.size();
    int n = A[0].size();
    for (int i = 0; i < m; i++){
        for (int j = 0; j < n; j++){
            cout<< A[i][j] << " ";
        }
        cout<<endl;
    }
}