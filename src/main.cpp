#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

// put function declarations here:
void printArray(MatrixXd a);

void setup() {
  Serial.begin(9600);

    MatrixXd matrix  {3, 3};

    VectorXd vec{3};
    vec << 1, 2, 3;
    //Create Matrix from collumn vectors
    matrix << vec, vec, vec;
  //Custom print Function
  printArray(matrix);

}
void loop() {
  // put your main code here, to run repeatedly:
}
void printArray(MatrixXd a){
   for (int i = 0; i < a.rows(); i++) {
        for (int j = 0; j < a.cols(); j++) {
      Serial.print(a(i,j));
      Serial.print("\t");
    }
    Serial.println();
  } 
}