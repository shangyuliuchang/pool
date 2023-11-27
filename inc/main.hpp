#ifndef __MAIN__
#define __MAIN__
#define WINDOW_WIDTH 1800
#define WINDOW_HEIGHT 900
#define g (9.8f)
#define holeR (0.08f)
#define edgeY (1.384f * 0.5f)
#define edgeX (2.654f * 0.5f)
#define holeD1 (holeR * 0.707107f)
#define holeD2 (holeR * 1.12132f)
#define holeD3 (1.4142f * holeR)
#define holeX (1.33872f)
#define holeY (0.70372f)
#define kX (1.0f)
#define kY (1.0f)
#define PI (3.1415926f)
#define edgeTheta (0.0f)
#define NPOINT (20)
#define epsilon (1e-3f)
#define kFail (0.25f)
#define kSuccess (1.5f)
#define cueK (0.02f)
#define shadowWidth (WINDOW_WIDTH)
#define shadowHeight (WINDOW_HEIGHT)
extern int mode;
extern int allowHit;
void quatRotate(float p[], float wx, float wy, float wz, float t);
#endif