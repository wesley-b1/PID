#pragma once

void computeU();
void setTunings(double kP, double kI, double kD);
void setControlLimits(double min, double max);

extern double u, y, sp;
