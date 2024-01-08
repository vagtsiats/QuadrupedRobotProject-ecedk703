#include <BasicLinearAlgebra.h>
using namespace BLA;

const int num_joints = 3;

// DH Parameters
float a[num_joints] = {0.3, 0.7, 0.5};
float alpha[num_joints] = {PI/2, 0, 0};
float d[num_joints] = {0, 0, 0};
float theta[num_joints] = {0, 0, 0};

void printMatrix(Matrix<4, 4> matrix);
Matrix<4,4> dhTransform(float a, float alpha, float d, float theta);
Matrix<4,4> forwardKinematics(float a[], float alpha[], float d[], float theta[]);

void setup() {
    Serial.begin(9600);

    Matrix<4,4> final_transform = forwardKinematics(a, alpha, d, theta);

    printMatrix(final_transform);
}

void loop() {
    // Your loop code here
}

void printMatrix(Matrix<4, 4> matrix) {
  for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.print(matrix(i, j));
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
    Serial.println();
}

Matrix<4,4> dhTransform(float a, float alpha, float d, float theta) {
    Matrix<4,4> T = {cos(theta), -sin(theta), 0, a, sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha), sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha), 0, 0, 0, 1};
    return T;
}

Matrix<4,4> forwardKinematics(float a[], float alpha[], float d[], float theta[]) {
    Matrix<4,4> final_transform = dhTransform(a[0], alpha[0], d[0], theta[0]);

    for (int i = 1; i < num_joints; i++) {
      final_transform = final_transform * dhTransform(a[i], alpha[i], d[i], theta[i]);
    }

    return final_transform;
}
