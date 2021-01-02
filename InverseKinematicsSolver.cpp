#include "Arduino.h";
#include "InverseKinematicsSolver.h";

#define PI 3.14159265358979
#define NUM_SOLUTIONS 10
#define DEFAULT_ANGLE 90

InverseKinematicsSolver::InverseKinematicsSolver(double armLength, double forearmLength, double wristLength){
  L1 = armLength;
  L2 = forearmLength;
  L3 = wristLength;
}

bool InverseKinematicsSolver::solve3D(double x, double y, double z) {
  double theta = atan2(y, x);
  double r = sqrt(pow(x, 2) + pow(y, 2));
  bool ok = solve2D(r, z);
  R0 = 180 - radiansToDegrees(theta);
  if (ok) {
    printSuccessStatus(x, y, z);
    return true;
  } else {
    R0 = DEFAULT_ANGLE;
    R1 = DEFAULT_ANGLE;
    R2 = DEFAULT_ANGLE;
    R3 = DEFAULT_ANGLE;
    printErrorStatus(x, y, z);
    return false;
  }
}

bool InverseKinematicsSolver::solve2D(double r, double z) {
  double phi = atan2(z, r);
  double c = sqrt(pow(r, 2) + pow(z, 2));
  
  double minimumArmLength = sqrt(pow(L1, 2) + pow(L2, 2));
  double maximumArmLength = L1 + L2;
  double step = (maximumArmLength - minimumArmLength) / NUM_SOLUTIONS;
  
  double Q1S;
  double Q1L;
  double Q2;
  double Q3S;
  double Q3L;
  double A;
  for (int i = 0; i < NUM_SOLUTIONS; i++) {
    A = minimumArmLength + step * i;
    if (abs(((L1*L1 + A*A) - L2*L2) / (2*L1*A)) > 1 || abs(((A*A + c*c) - L3*L3) / (2*c*A)) > 1 || abs(((L1*L1 + L2*L2) - A*A) / (2*L1*L2)) > 1 || abs(((L2*L2 + A*A) - L1*L1) / (2*L2*A)) > 1 || abs(((A*A + L3*L3) - c*c) / (2*A*L3)) > 1) {
      continue;
    }
    Q1S = acos(((L1*L1 + A*A) - L2*L2) / (2*L1*A));
    Q1L = acos(((A*A + c*c) - L3*L3) / (2*c*A));
    Q2 = acos(((L1*L1 + L2*L2) - A*A) / (2*L1*L2));
    Q3S = acos(((L2*L2 + A*A) - L1*L1) / (2*L2*A));
    Q3L = acos(((A*A + L3*L3) - c*c) / (2*A*L3));
    if (isPossibleForBraccio(phi, Q1S, Q1L, Q2, Q3S, Q3L)) {
      setArmJointAngles(phi, Q1S, Q1L, Q2, Q3S, Q3L);
      return true;
    }
  }
  return false;
}

InverseKinematicsSolver::setArmJointAngles(double phi, double Q1S, double Q1L, double Q2, double Q3S, double Q3L) {
  double Q1 = Q1S + Q1L;
  double Q3 = Q3S + Q3L;
  R1 = 180 - radiansToDegrees(phi + Q1);
  R2 = 270 - radiansToDegrees(Q2);
  R3 = 90 + radiansToDegrees(Q1S + Q2 - Q3L);
}

bool InverseKinematicsSolver::isPossibleForBraccio(double phi, double Q1S, double Q1L, double Q2, double Q3S, double Q3L) {
  double Q1 = Q1S + Q1L;
  double Q3 = Q3S + Q3L;
  double r1 = 180 - radiansToDegrees(phi + Q1);
  double r2 = 270 - radiansToDegrees(Q2);
  double r3 = 90 + radiansToDegrees(Q1S + Q2 - Q3L);
  return r1 >= 0.0 && r1 <= 180.0 && r2 >= 0.0 && r2 <= 180.0 && r3 >= 0.0 && r3 <= 180.0;
}

double InverseKinematicsSolver::radiansToDegrees(double radians) {
  return radians * 180.0/PI;
}

InverseKinematicsSolver::printSuccessStatus(double x, double y, double z) {
  Serial.print("Moving to (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(") with angles:\n//");
  Serial.print(R0);
  Serial.print(", ");
  Serial.print(R1);
  Serial.print(", ");
  Serial.print(R2);
  Serial.print(", ");
  Serial.print(R3);
  Serial.println(".\n");
}

InverseKinematicsSolver::printErrorStatus(double x, double y, double z) {
  Serial.print("Error: Could not set angles for (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println(").\n"); 
}
