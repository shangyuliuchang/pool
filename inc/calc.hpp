#ifndef __CALC__
#define __CALC__
void update(float t);
void updateWithDetect(float t);
void collision(float t);
void hitBall(float yaw, float pitch, float vx, float vy, float vz);
int edgeCollision(float x, float y, float r, float *dx, float *dy);
#endif