#ifndef __BALL__
#define __BALL__
class ball{
    public:
    float x, y, z;
    float wx, wy, wz;
    float vx, vy, vz;
    float r = 57.15e-3f / 2.0f;
    float m = 0.17f;
    float I = 0.4f * m * r * r;
    float quatX[4];
    float quatY[4];
    ball(){
        x = y = z = 0;
        vx = vy = vz = 0;
        wx = wy = wz = 0;
        quatX[0] = quatX[2] = quatX[3] = 0.0f;
        quatX[1] = 1.0f;
        quatY[0] = quatY[1] = quatY[3] = 0.0f;
        quatY[2] = 1.0f;
    }
    ball(float _x, float _y, float _z){
        x = _x;
        y = _y;
        z = _z;
        vx = vy = vz = 0;
        wx = wy = wz = 0;
        quatX[0] = quatX[2] = quatX[3] = 0.0f;
        quatX[1] = 1.0f;
        quatY[0] = quatY[1] = quatY[3] = 0.0f;
        quatY[2] = 1.0f;
    }
};
#define mu1 (0.2f) 
#define mu2 (0.01f)
#define mu3 (0.05f)
#define mu4 (0.8f)
#define az (6.0f)
#define collisionK (0.65f)
extern vector<ball> balls;
#endif