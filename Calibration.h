#ifndef CALIBRATION_H
#define CALIBRATION_H

//Calibration parameters
const double INF = 99999999999;

static int num_frames;

static int frames_start;
static int frames_end;

const int dSample = 5;
const double distReject  = 20.0;

const double clipRangeX[] = { 5, 553.800000000000 };
const double clipRangeY[] = { 5, 298.212500000000 };
const double clipRangeZ[] = { 5, 353.212500000000 };

const int nX = 1;
const int nY = 1;
//const double dX = 558.8;
//const double dY = 303.2125;
const double dX = 174.5;
const double dY = 107.5;

//const double fc[] = { 1029.088963049301100, 1028.772845684173700 };
//const double cc[] = { 287.368565122283710, 204.931308136716550 };
//const double alpha_c = 0.000000000000000;
//const double kc[] = { -0.148348422140938, 0.215129139753359, 0.004513111567607, 0.004877209469556, 0.000000000000000 };
const double fc[] = { 651.514345426603310 , 652.473938336535070 };
const double cc[] = { 266.119265633241980 , 254.036971067644600 };
const double alpha_c = 0.000000000000000;
const double kc[] = { 0.098786414623054 , -0.540246871283047 , -0.002621287341312 , 0.005006915613245 , 0.000000000000000  };


const double fc_error[] = { 17.070043715418674, 18.263987634583518 };
const double cc_error[] = { 27.412500281800273, 22.806875607135709 };
const double alpha_c_error = 0.000000000000000;
const double kc_error[] = { 0.085725043781932, 1.134230738991625, 0.005220394520242, 0.006113308879498, 0.000000000000000 };

//const int nx = 640;
//const int ny = 480;

const int nx = 512;
const int ny = 384;

const double minContrast = 50;

#endif

