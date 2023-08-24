#include "main.h"
#include <cmath>

class KalmanFilter2D {
private:
    float x, y, theta;
    float P[3][3];

public:
    KalmanFilter2D(float initial_x, float initial_y, float initial_theta) : x(initial_x), y(initial_y), theta(initial_theta) {
        P[0][0] = P[1][1] = P[2][2] = 1;
        P[0][1] = P[0][2] = P[1][0] = P[1][2] = P[2][0] = P[2][1] = 0;
    }

    void update(float measurement_x, float measurement_y, float measurement_theta) {
        float Q[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
        float R[3][3] = {{0.05, 0, 0}, {0, 0.05, 0}, {0, 0, 0.05}};
        float F[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        float H[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        float x_pred = x;
        float y_pred = y;
        float theta_pred = theta;
        float P_pred[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P_pred[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j] + Q[i][j];
            }
        }
        float innovation[3];
        innovation[0] = measurement_x - x_pred;
        innovation[1] = measurement_y - y_pred;
        innovation[2] = measurement_theta - theta_pred;
        float S[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                S[i][j] = H[i][0] * P_pred[0][j] + H[i][1] * P_pred[1][j] + H[i][2] * P_pred[2][j] + R[i][j];
            }
        }
        float detS = S[0][0] * (S[1][1] * S[2][2] - S[2][1] * S[1][2]) - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        if (fabs(detS) < 1e-6) { return; }
        float invS[3][3];
        invS[0][0] = (S[1][1] * S[2][2] - S[2][1] * S[1][2]) / detS;
        invS[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / detS;
        invS[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / detS;
        invS[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) / detS;
        invS[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / detS;
        invS[1][2] = (S[1][0] * S[0][2] - S[0][0] * S[1][2]) / detS;
        invS[2][0] = (S[1][0] * S[2][1] - S[2][0] * S[1][1]) / detS;
        invS[2][1] = (S[2][0] * S[0][1] - S[0][0] * S[2][1]) / detS;
        invS[2][2] = (S[0][0] * S[1][1] - S[1][0] * S[0][1]) / detS;
        float K[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K[i][j] = P_pred[i][0] * H[0][j] + P_pred[i][1] * H[1][j] + P_pred[i][2] * H[2][j];
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K[i][j] = K[i][0] * invS[0][j] + K[i][1] * invS[1][j] + K[i][2] * invS[2][j];
            }
        }
        x = x_pred + K[0][0] * innovation[0] + K[0][1] * innovation[1] + K[0][2] * innovation[2];
        y = y_pred + K[1][0] * innovation[0] + K[1][1] * innovation[1] + K[1][2] * innovation[2];
        theta = theta_pred + K[2][0] * innovation[0] + K[2][1] * innovation[1] + K[2][2] * innovation[2];
        float I_KH[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                I_KH[i][j] -= K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j];
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P[i][j] = I_KH[i][0] * P_pred[0][j] + I_KH[i][1] * P_pred[1][j] + I_KH[i][2] * P_pred[2][j];
            }
        }
    }

    float getX() { return x; }
    float getY() { return y; }
    float getTheta() { return theta; }
};

class PIDController {
private:
    float kp, ki, kd;
    float integral, previousError;

public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), integral(0), previousError(0) {}

    float calculate(float target, float current) {
        float error = target - current;
        integral += error;
        float derivative = error - previousError;
        float output = kp * error + ki * integral + kd * derivative;
        previousError = error;
        return output;
    }
};

void driveRobot(float leftControl, float rightControl) {
    pros::Motor leftMotor1(1);
    pros::Motor leftMotor2(2);
    pros::Motor rightMotor1(3);
    pros::Motor rightMotor2(4);
    leftMotor1.move(leftControl);
    leftMotor2.move(leftControl);
    rightMotor1.move(rightControl);
    rightMotor2.move(rightControl);
}

void stopRobot() {
    driveRobot(0, 0);
}

float getVerticalTrackerReading() {
    pros::ADIEncoder verticalTracker1(1, 2, false);
    pros::ADIEncoder verticalTracker2(3, 4, false);
    return (verticalTracker1.get_value() + verticalTracker2.get_value()) / 2.0;
}

float getHorizontalTrackerReading() {
    pros::ADIEncoder horizontalTracker(5, 6, false);
    return horizontalTracker.get_value();
}

float getIMUReading() {
    pros::Imu imuSensor(21);
    return imuSensor.get_rotation();
}

void moveToTarget(KalmanFilter2D &filter, float targetX, float targetY, float targetTheta, float tolerance) {
    PIDController pidX(1.0, 0.0, 0.1);
    PIDController pidY(1.0, 0.0, 0.1);
    PIDController pidTheta(1.0, 0.0, 0.1);
    float errorX, errorY, errorTheta;

    do {
        float currentX = filter.getX();
        float currentY = filter.getY();
        float currentTheta = filter.getTheta();
        float controlInputX = pidX.calculate(targetX, currentX);
        float controlInputY = pidY.calculate(targetY, currentY);
        float controlInputTheta = pidTheta.calculate(targetTheta, currentTheta);
        float leftControl = controlInputX + controlInputTheta;
        float rightControl = controlInputX - controlInputTheta;
        driveRobot(leftControl, rightControl);
        float measurement_x = getVerticalTrackerReading();
        float measurement_y = getHorizontalTrackerReading();
        float measurement_theta = getIMUReading();
        filter.update(measurement_x, measurement_y, measurement_theta);
        errorX = targetX - currentX;
        errorY = targetY - currentY;
        errorTheta = targetTheta - currentTheta;
        pros::delay(10);
    } while (fabs(errorX) > tolerance || fabs(errorY) > tolerance || fabs(errorTheta) > tolerance);
}

void odometryLoop() {
    KalmanFilter2D filter(0, 0, 0);

    while (true) {
        float measurement_x = getVerticalTrackerReading();
        float measurement_y = getHorizontalTrackerReading();
        float measurement_theta = getIMUReading();
        filter.update(measurement_x, measurement_y, measurement_theta);
        pros::delay(10);
    }
}

void updateOdometry() {
    pros::Task odometryTask(odometryLoop);
    float targetPoints[][3] = { {100, 100, 3.14159/2}, {200, 100, 0}, {200, 200, 3.14159} };
    int numPoints = sizeof(targetPoints) / sizeof(targetPoints[0]);
    KalmanFilter2D filter(0, 0, 0);

    for (int i = 0; i < numPoints; i++) {
        moveToTarget(filter, targetPoints[i][0], targetPoints[i][1], targetPoints[i][2], 5.0);
    }

    stopRobot();
}
