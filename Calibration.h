#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <math.h>
#include "RobControl.h"
#include "Frame.h"

/* Declaration of calibration functions  */

unsigned short ToolCalibration(Frame_Type P[5], Coord_Type *Result);
unsigned short Triangulate(double A[4][4]);

unsigned short FrameCalibration(Coord_Type P[3], Frame_Type *Result);

#endif

