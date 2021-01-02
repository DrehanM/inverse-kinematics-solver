/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Arduino.h"

class InverseKinematicsSolver
{
  public:
    InverseKinematicsSolver(double armLength, double forearmLength, double wristLength);
    bool solve3D(double x, double y, double z);
    bool solve2D(double r, double z);
    double R0;
    double R1;
    double R2;
    double R3;
    
  private:
    double radiansToDegrees(double radians);
    bool isPossibleForBraccio(double phi, double Q1S, double Q1L, double Q2, double Q3S, double Q3L);
    setArmJointAngles(double phi, double Q1S, double Q1L, double Q2, double Q3S, double Q3L);
    printSuccessStatus(double x, double y, double z);
    printErrorStatus(double x, double y, double z);
    double L1;
    double L2;
    double L3;
};