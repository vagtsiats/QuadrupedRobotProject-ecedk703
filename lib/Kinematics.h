#include <BasicLinearAlgebra.h>

#define num_joints 4


class Kinematics {
private:
    BLA::Vector<num_joints> a;
    BLA::Vector<num_joints> alpha;
    BLA::Vector<num_joints> d;
    BLA::Vector<num_joints> theta;

public:
    Kinematics(BLA::Vector<num_joints> set_a, BLA::Vector<num_joints> set_alpha) {
        a = set_a;
        alpha = set_alpha;
        d = {0, 0, 0};
        theta = {0, 0, 0};
    }

    BLA::Matrix<4, 4> dhTransform(float d, float theta) {
        BLA::Matrix<4, 4> T = {cos(theta), -sin(theta), 0, a, sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha), sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha), 0, 0, 0, 1};
        return T;
    }

    BLA::Matrix<4, 4> forwardKinematics(BLA::Vector<num_joints> d, BLA::Vector<num_joints> theta) {
        BLA::Matrix<4, 4> final_transform = dhTransform(a[0], alpha[0], d[0], theta[0]);

        for (int i = 1; i < num_joints; i++)
        {
            final_transform = final_transform * dhTransform(a[i], alpha[i], d[i], theta[i]);
        }

        return final_transform;
    }

    // TODO: See if its worth doing it abstract. If not hard code
    BLA::Matrix<2, 3> inverseKinematics(BLA::Matrix<4, 4> homogeneous_matrix) {
        BLA::Matrix<2, 3> result;
        BLA::Vector<num_joints> d = {0, 0, 0};
        BLA::Vector<num_joints> theta = {0, 0, 0};

        for(int iteration = 0; iteration < 100; ++iteration) {
            BLA::Matrix<4, 4> current_transform = forwardKinematics(a, alpha, d, theta); // See how close you are

            BLA::Matrix<4, 4> error = homogeneous_matrix - current_transform; // Compute error

            if(IsAcceptable(error)) { // Check error
                break;
            }

            BLA::Matrix<num_joints, num_joints> J = computeJacobian(d, theta); // Compute J

            theta = theta + Jpinv(J) * getPosition(current_transform); // Fix theta
        }

        
        for(int i = 0; i < num_joints; ++i) {
            result(0, i) = d(i);
            result(1, i) = theta(i);
        }

        return result;
    }

    // TODO!!!
    BLA::Vector<6> differentialKinematics(BLA:Vector<3> theta, BLA::Vector<num_joints> jointVelocities) {
        BLA::Matrix<6, num_joints> J = computeJacobian({0, 0, 0}, theta);
        BLA::Matrix<6> endEffectorVelocity = J * jointVelocities;
        return endEffectorVelocity;
    }

    BLA::Vector<num_joints> inverseDifferentialKinematics(BLA:Vector<3> theta, BLA::Vector<6> desiredEndEffectorVelocity) {
        BLA::Matrix<6, num_joints> J = computeJacobian({0, 0, 0}, theta);
        BLA::Matrix<num_joints> jointVelocities = Jpinv(J) * desiredEndEffectorVelocity;
        return jointVelocities;
    }

    BLA::Matrix<6, num_joints> computeJacobian(BLA::Vector<num_joints> d, BLA::Vector<num_joints> theta) {
        BLA::Matrix<6, num_joints> J;

        BLA::Matrix<4, 4> T01 = dhTransform(a[0], alpha[0], d[0], theta[0]);
        BLA::Matrix<4, 4> T02 = T01*dhTransform(a[1], alpha[1], d[1], theta[1]);
        BLA::Matrix<4, 4> T03 = T02*dhTransform(a[2], alpha[2], d[2], theta[2]);

        BLA::Vector<3> z0 = {0,0,1};
        BLA::Vector<3> z1 = T01.Submatrix<3,1>(0,2);
        BLA::Vector<3> z2 = T02.Submatrix<3,1>(0,2);

        BLA::Vector<3> p1 = T01.Submatrix<3,1>(0,3);
        BLA::Vector<3> p2 = T02.Submatrix<3,1>(0,3);
        BLA::Vector<3> pe = T03.Submatrix<3,1>(0,3);
        
        BLA::Matrix<3, 3> cp1 = crossProduct(z0, pe);
        BLA::Matrix<3, 3> cp2 = crossProduct(z1, pe - p1);
        BLA::Matrix<3, 3> cp3 = crossProduct(z2, pe - p2);

        BLA::Matrix<6, num_joints> J = {cp1, cp2, cp3} && {z0, z1, z2};

        return J;
    }

    BLA::Matrix<num_joints, 6> Jpinv(BLA::Matrix<9, 3> J) {
        return Inverse(~J * J) * (~J);
    }

    int IsAcceptable(BLA::Matrix<4, 4> error) {
        int threshold = 0.1;
        BLA::Matrix<4, 4> temp = zeros(4, 4);
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                if std::abs(error(i, j) < threshold){
                    temp(i, j) = 1;
                }
            }
        }
        int result = 1;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                result *= temp(i, j);
            }
        }
        return result;
    }

    BLA::Matrix<3, 3> crossProduct(BLA::Matrix<3> v1, BLA::Matrix<3> v2) {
        BLA::Matrix<3> result;
        result(0) = v1(1)*v2(2) - v1(2)*v2(1);
        result(1) = v1(2)*v2(0) - v1(0)*v2(2);
        result(2) = v1(0)*v2(1) - v1(1)*v2(0);
        return result;
    }

    BLA::Vector<3> getPosition(BLA::Matrix<4, 4> homogeneous_matrix) {
        BLA::Vector<3> position;
        for(int i = 0; i < 3; ++i) {
            position(i) = homogeneous_matrix(i, 3);
        }
        return position;
    }

    BLA::Matrix<3, 3> getOrientation(BLA::Matrix<4, 4> homogeneous_matrix) {
        BLA::Matrix<3, 3> orientation;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                orientation(i, j) = homogeneous_matrix(i, j);
            }
        }
        return orientation;
    }
