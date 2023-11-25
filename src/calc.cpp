#include<vector>
#include<math.h>
#include<stdio.h>
using namespace std;
#include"../inc/ball.hpp"
#include"../inc/main.hpp"
void hitBall(float yaw, float pitch, float vx, float vy, float vz){
    float rx, ry, rz, I, vx0, vy0, vz0, v, Ib, rv1, rv2, rv3, rv4, rvx, rvy, vn, vtx, vty, vtz, wvx, wvy, wvz;
    float rvx2, rvy2, vtx2, vty2, vtz2, wvx2, wvy2, wvz2;
    float tmpVx, tmpVy, tmpVz;
    float Itx, Ity, Itz, Ibx, Iby, It;
    float currentK, nextK;
    int state1, state2, state3, state4, reject;
    float frictionAngle, frictionAngle2;
    ball tmpBall;
    rx = balls[0].r * cosf(pitch) * cosf(yaw);
    ry = balls[0].r * cosf(pitch) * sinf(yaw);
    rz = balls[0].r * sinf(pitch);
    v = sqrtf(vx * vx + vy * vy + vz * vz);
    vx0 = vx / v;
    vy0 = vy / v;
    vz0 = vz / v;
    I = -2.0f * (vx * rx + vy * ry + vz * rz) / balls[0].r * 3.0f * balls[0].m;
    currentK = 0.0f;
    nextK = 1.0f;

    tmpBall = balls[0];
    tmpVx = vx;
    tmpVy = vy;
    tmpVz = vz;
    rvx = tmpBall.vx - tmpBall.wy * balls[0].r;
    rvy = tmpBall.vy + tmpBall.wx * balls[0].r;
    vn = ((tmpVx - tmpBall.vx) * rx + (tmpVy - tmpBall.vy) * ry + (tmpVz - tmpBall.vz) * rz) / balls[0].r;
    vtx = (tmpVx - tmpBall.vx) - vn * rx / balls[0].r;
    vty = (tmpVy - tmpBall.vy) - vn * ry / balls[0].r;
    vtz = (tmpVz - tmpBall.vz) - vn * rz / balls[0].r;
    wvx = tmpBall.wy * rz - tmpBall.wz * ry;
    wvy = tmpBall.wz * rx - tmpBall.wx * rz;
    wvz = tmpBall.wx * ry - tmpBall.wy * rx;
    if(fabsf(vtx - wvx) > epsilon || fabsf(vty - wvy) > epsilon || fabsf(vtz - wvz) > epsilon){
        state1 = 1;
        rv2 = sqrtf(powf(vtx - wvx, 2) + powf(vty - wvy, 2) + powf(vtz - wvz, 2));
        Itx = I * mu4 * (vtx - wvx) / rv2;
        Ity = I * mu4 * (vty - wvy) / rv2;
        Itz = I * mu4 * (vtz - wvz) / rv2;
    }else{
        state1 = 0;
        Itx = Ity = Itz = 0.0f;
    }
    It = (I * rx / balls[0].r - Itx) * vx0 + (I * ry / balls[0].r - Ity) * vy0 + (I * rz / balls[0].r - Itz) * vz0;
    Ib = I * rz / balls[0].r - Itz;
    if(Ib < 0.0f) Ib = 0.0f;
    if(fabsf(rvx) > epsilon || fabsf(rvy) > epsilon){
        state2 = 1;
        rv1 = sqrtf(rvx * rvx + rvy * rvy);
        frictionAngle = atan2f(rvy, rvx);
        Ibx = -Ib * mu1 * rvx / rv1;
        Iby = -Ib * mu1 * rvy / rv1;
    }else{
        state2 = 0;
        Ibx = 0.0f;
        Iby = 0.0f;
    }
    while(currentK < 1.0f){
        tmpBall.vx += ((-I * rx / balls[0].r + Itx + Ibx) / balls[0].m) * nextK;
        tmpBall.vy += ((-I * ry / balls[0].r + Ity + Iby) / balls[0].m) * nextK;
        tmpBall.wx += ((ry * Itz - rz * Ity + Iby * balls[0].r) / balls[0].I) * nextK;
        tmpBall.wy += ((rz * Itx - rx * Itz - Ibx * balls[0].r) / balls[0].I) * nextK;
        tmpBall.wz += ((rx * Ity - ry * Itx) / balls[0].I) * nextK;
        tmpVx += (It * vx0 / (3.0f * balls[0].m) + (I * rx / balls[0].r - Itx - It * vx0) / (cueK * 3.0f * balls[0].m)) * nextK;
        tmpVy += (It * vy0 / (3.0f * balls[0].m) + (I * ry / balls[0].r - Ity - It * vy0) / (cueK * 3.0f * balls[0].m)) * nextK;
        tmpVz += (It * vz0 / (3.0f * balls[0].m) + (I * rz / balls[0].r - Itz - It * vz0) / (cueK * 3.0f * balls[0].m)) * nextK;

        reject = 0;
        rvx2 = tmpBall.vx - tmpBall.wy * balls[0].r;
        rvy2 = tmpBall.vy + tmpBall.wx * balls[0].r;
        frictionAngle2 = atan2f(rvy2, rvx2);
        vn = ((tmpVx - tmpBall.vx) * rx + (tmpVy - tmpBall.vy) * ry + (tmpVz - tmpBall.vz) * rz) / balls[0].r;
        vtx2 = (tmpVx - tmpBall.vx) - vn * rx / balls[0].r;
        vty2 = (tmpVy - tmpBall.vy) - vn * ry / balls[0].r;
        vtz2 = (tmpVz - tmpBall.vz) - vn * rz / balls[0].r;
        wvx2 = tmpBall.wy * rz - tmpBall.wz * ry;
        wvy2 = tmpBall.wz * rx - tmpBall.wx * rz;
        wvz2 = tmpBall.wx * ry - tmpBall.wy * rx;
        rv4 = sqrtf(powf(vtx2 - wvx2, 2) + powf(vty2 - wvy2, 2) + powf(vtz2 - wvz2, 2));
        if(state1 == 1 && ((vtx - wvx) * (vtx2 - wvx2) + (vty - wvy) * (vty2 - wvy2) + (vtz - wvz) * (vtz2 - wvz2) <= 0.99f * rv2 * rv4)){
            reject = 1;
        }else if(state1 == 0 && (fabsf(vtx2 - wvx2) > epsilon || fabsf(vty2 - wvy2) > epsilon || fabsf(vtz2 - wvz2) > epsilon)){
            reject = 1;
        }
        if(state2 == 1 && (fabsf(frictionAngle2 - frictionAngle) > PI / 18.0f)){
            reject = 1;
        }else if(state2 == 0 && (fabsf(rvx2) > epsilon || fabsf(rvy2) > epsilon)){
            reject = 1;
        }
        if(((tmpVx - tmpBall.vx) * rx + (tmpVy - tmpBall.vy) * ry + (tmpVz - tmpBall.vz) * rz) / (v * vx0 * rx + v * vy0 * ry + v * vz0 * rz) < -0.6f){
            reject = 1;
        }
        if(reject && nextK > 1e-3f){
            nextK *= kFail;
            tmpBall = balls[0];
            tmpVx = vx;
            tmpVy = vy;
            tmpVz = vz;
        }else{
            currentK += nextK;
            balls[0] = tmpBall;
            vx = tmpVx;
            vy = tmpVy;
            vz = tmpVz;
            nextK *= kSuccess;
            if(nextK > 1.0f - currentK) nextK = 1.0f - currentK;
            if(((tmpVx - tmpBall.vx) * rx + (tmpVy - tmpBall.vy) * ry + (tmpVz - tmpBall.vz) * rz) / (v * vx0 * rx + v * vy0 * ry + v * vz0 * rz) < -0.6f){
                break;
            }
            if(currentK < 1.0f){
                rvx = tmpBall.vx - tmpBall.wy * balls[0].r;
                rvy = tmpBall.vy + tmpBall.wx * balls[0].r;
                vn = ((tmpVx - tmpBall.vx) * rx + (tmpVy - tmpBall.vy) * ry + (tmpVz - tmpBall.vz) * rz) / balls[0].r;
                vtx = (tmpVx - tmpBall.vx) - vn * rx / balls[0].r;
                vty = (tmpVy - tmpBall.vy) - vn * ry / balls[0].r;
                vtz = (tmpVz - tmpBall.vz) - vn * rz / balls[0].r;
                wvx = tmpBall.wy * rz - tmpBall.wz * ry;
                wvy = tmpBall.wz * rx - tmpBall.wx * rz;
                wvz = tmpBall.wx * ry - tmpBall.wy * rx;
                if(fabsf(vtx - wvx) > epsilon || fabsf(vty - wvy) > epsilon || fabsf(vtz - wvz) > epsilon){
                    state1 = 1;
                    rv2 = sqrtf(powf(vtx - wvx, 2) + powf(vty - wvy, 2) + powf(vtz - wvz, 2));
                    Itx = I * mu4 * (vtx - wvx) / rv2;
                    Ity = I * mu4 * (vty - wvy) / rv2;
                    Itz = I * mu4 * (vtz - wvz) / rv2;
                }else{
                    state1 = 0;
                    Itx = Ity = Itz = 0.0f;
                }
                It = (I * rx / balls[0].r - Itx) * vx0 + (I * ry / balls[0].r - Ity) * vy0 + (I * rz / balls[0].r - Itz) * vz0;
                Ib = I * rz / balls[0].r - Itz;
                if(Ib < 0.0f) Ib = 0.0f;
                if(fabsf(rvx) > epsilon || fabsf(rvy) > epsilon){
                    state2 = 1;
                    rv1 = sqrtf(rvx * rvx + rvy * rvy);
                    frictionAngle = atan2f(rvy, rvx);
                    Ibx = -Ib * mu1 * rvx / rv1;
                    Iby = -Ib * mu1 * rvy / rv1;
                }else{
                    state2 = 0;
                    Ibx = 0.0f;
                    Iby = 0.0f;
                }
            }
        }
    }
}
void updateWithDetect(float t){
    float currentT = 0.0f;
    float tmp, nextT, frictionAngle, ax, ay, rvx, rvy, rvx2, rvy2, v, rv, frictionAngle2, vx, vy, wx, wy;
    int state1, state2, reject;
    ball tmpBall;
    for(int i=0;i<balls.size();i++){
        tmpBall = balls[i];
        currentT = 0.0f;
        nextT = t;

        rvx = tmpBall.vx - tmpBall.wy * tmpBall.r;
        rvy = tmpBall.vy + tmpBall.wx * tmpBall.r;
        if(fabsf(tmpBall.wx) < epsilon && fabsf(tmpBall.wy) < epsilon && fabsf(tmpBall.vx) < epsilon && fabsf(tmpBall.vy) < epsilon){
            ax = 0.0f;
            ay = 0.0f;
            state1 = 1;
        }else if(fabsf(rvx) < epsilon && fabsf(rvy) < epsilon){
            v = sqrtf(tmpBall.vx * tmpBall.vx + tmpBall.vy * tmpBall.vy);
            ax = -mu2 * g * tmpBall.vx / v;
            ay = -mu2 * g * tmpBall.vy / v;
            vx = tmpBall.vx;
            vy = tmpBall.vy;
            wx = tmpBall.wx;
            wy = tmpBall.wy;
            state1 = 2;
        }else{
            frictionAngle = atan2f(rvy, rvx);
            rv = sqrtf(rvx * rvx + rvy * rvy);
            ax = -mu1 * g * rvx / rv;
            ay = -mu1 * g * rvy / rv;
            state1 = 3;
        }
        while(currentT < t){
            if(state1 == 1){
                tmpBall.vx = 0.0f;
                tmpBall.vy = 0.0f;
                tmpBall.wx = 0.0f;
                tmpBall.wy = 0.0f;
            }else if(state1 == 2){
                tmpBall.vx += ax * nextT;
                tmpBall.vy += ay * nextT;
                tmpBall.wx = -tmpBall.vy / tmpBall.r;
                tmpBall.wy = tmpBall.vx / tmpBall.r;
            }else{
                tmpBall.vx += ax * nextT;
                tmpBall.vy += ay * nextT;
                tmpBall.wx += ay * tmpBall.m * tmpBall.r / tmpBall.I * nextT;
                tmpBall.wy += -ax * tmpBall.m * tmpBall.r / tmpBall.I * nextT;
            }

            reject = 0;
            rvx2 = tmpBall.vx - tmpBall.wy * tmpBall.r;
            rvy2 = tmpBall.vy + tmpBall.wx * tmpBall.r;
            frictionAngle2 = atan2f(rvy2, rvx2);
            if(state1 == 3 && (fabsf(frictionAngle2 - frictionAngle) > PI / 18.0f)){
                reject = 1;
            }else if(state1 == 2 && (tmpBall.vx * vx <= 0.0f || tmpBall.vy * vy <= 0.0f)){
                reject = 1;
            }

            if(reject && nextT > 1e-4f){
                nextT *= kFail;
                tmpBall = balls[i];
            }else{
                tmpBall.x += tmpBall.vx * nextT + 0.5f * ax * nextT * nextT;
                tmpBall.y += tmpBall.vy * nextT + 0.5f * ay * nextT * nextT;
                if(tmpBall.wz > az * nextT)
                    tmpBall.wz -= az * nextT;
                else if(tmpBall.wz < -az * nextT)
                    tmpBall.wz += az * nextT;
                else
                    tmpBall.wz = 0.0f;
                quatRotate(tmpBall.quatX, tmpBall.wx, tmpBall.wy, tmpBall.wz, nextT);
                quatRotate(tmpBall.quatY, tmpBall.wx, tmpBall.wy, tmpBall.wz, nextT);
                currentT += nextT;
                balls[i] = tmpBall;
                nextT *= kSuccess;
                if(nextT > t - currentT) nextT = t - currentT;
                if(currentT < t){
                    rvx = tmpBall.vx - tmpBall.wy * tmpBall.r;
                    rvy = tmpBall.vy + tmpBall.wx * tmpBall.r;
                    if(fabsf(tmpBall.wx) < epsilon && fabsf(tmpBall.wy) < epsilon && fabsf(tmpBall.vx) < epsilon && fabsf(tmpBall.vy) < epsilon){
                        ax = 0.0f;
                        ay = 0.0f;
                        state1 = 1;
                    }else if(fabsf(rvx) < epsilon && fabsf(rvy) < epsilon){
                        v = sqrtf(tmpBall.vx * tmpBall.vx + tmpBall.vy * tmpBall.vy);
                        ax = -mu2 * g * tmpBall.vx / v;
                        ay = -mu2 * g * tmpBall.vy / v;
                        vx = tmpBall.vx;
                        vy = tmpBall.vy;
                        wx = tmpBall.wx;
                        wy = tmpBall.wy;
                        state1 = 2;
                    }else{
                        frictionAngle = atan2f(rvy, rvx);
                        rv = sqrtf(rvx * rvx + rvy * rvy);
                        ax = -mu1 * g * rvx / rv;
                        ay = -mu1 * g * rvy / rv;
                        state1 = 3;
                    }
                }
            }
        }
    }
}
int edgeCollision(float x, float y, float r, float *dx, float *dy){
    if(powf(x - (-edgeX - holeR), 2) + powf(y - (-edgeY + holeD2), 2) < powf(holeR + r, 2)){
        *dx = x - (-edgeX - holeR);
        *dy = y - (-edgeY + holeD2);
        return 1;
    }else if(powf(x - (-edgeX + holeD2), 2) + powf(y - (-edgeY - holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (-edgeX + holeD2);
        *dy = y - (-edgeY - holeR);
        return 1;
    }else if(powf(x - (edgeX + holeR), 2) + powf(y - (-edgeY + holeD2), 2) < powf(holeR + r, 2)){
        *dx = x - (edgeX + holeR);
        *dy = y - (-edgeY + holeD2);
        return 1;
    }else if(powf(x - (edgeX - holeD2), 2) + powf(y - (-edgeY - holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (edgeX - holeD2);
        *dy = y - (-edgeY - holeR);
        return 1;
    }else if(powf(x - (-edgeX + holeD2), 2) + powf(y - (edgeY + holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (-edgeX + holeD2);
        *dy = y - (edgeY + holeR);
        return 1;
    }else if(powf(x - (-edgeX - holeR), 2) + powf(y - (edgeY - holeD2), 2) < powf(holeR + r, 2)){
        *dx = x - (-edgeX - holeR);
        *dy = y - (edgeY - holeD2);
        return 1;
    }else if(powf(x - (edgeX + holeR), 2) + powf(y - (edgeY - holeD2), 2) < powf(holeR + r, 2)){
        *dx = x - (edgeX + holeR);
        *dy = y - (edgeY - holeD2);
        return 1;
    }else if(powf(x - (edgeX - holeD2), 2) + powf(y - (edgeY + holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (edgeX - holeD2);
        *dy = y - (edgeY + holeR);
        return 1;
    }else if(powf(x - (-holeD3), 2) + powf(y - (edgeY + holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (-2.0f * holeR);
        *dy = y - (edgeY + holeR);
        return 1;
    }else if(powf(x - (-holeD3), 2) + powf(y - (-edgeY - holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (-2.0f * holeR);
        *dy = y - (-edgeY - holeR);
        return 1;
    }else if(powf(x - (holeD3), 2) + powf(y - (edgeY + holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (2.0f * holeR);
        *dy = y - (edgeY + holeR);
        return 1;
    }else if(powf(x - (holeD3), 2) + powf(y - (-edgeY - holeR), 2) < powf(holeR + r, 2)){
        *dx = x - (2.0f * holeR);
        *dy = y - (-edgeY - holeR);
        return 1;
    }else if(x < (-edgeX + r) && x > (-edgeX - r) && y > (-edgeY + holeD2) && y < (edgeY - holeD2)){
        *dx = 1.0f;
        *dy = 0.0f;
        return 1;
    }else if(x > (edgeX - r) && x < (edgeX + r) && y > (-edgeY + holeD2) && y < (edgeY - holeD2)){
        *dx = -1.0f;
        *dy = 0.0f;
        return 1;
    }else if(y > (edgeY - r) && y < (edgeY + r) && x > (-edgeX + holeD2) && x < (-holeD3)){
        *dx = 0.0f;
        *dy = -1.0f;
        return 1;
    }else if(y > (edgeY - r) && y < (edgeY + r) && x > (holeD3) && x < (edgeX - holeD2)){
        *dx = 0.0f;
        *dy = -1.0f;
        return 1;
    }else if(y < (-edgeY + r) && y > (-edgeY - r) && x > (-edgeX + holeD2) && x < (-holeD3)){
        *dx = 0.0f;
        *dy = 1.0f;
        return 1;
    }else if(y < (-edgeY + r) && y > (-edgeY - r) && x > (holeD3) && x < (edgeX - holeD2)){
        *dx = 0.0f;
        *dy = 1.0f;
        return 1;
    }else if(x + y > edgeY + holeR - holeD3 && x + y < edgeY && x - y < -edgeY - holeR + 1.414f * balls[0].r && x - y > -edgeY - holeR - 1.414f * balls[0].r){
        *dx = cosf(-0.25f * PI);
        *dy = sinf(-0.25f * PI);
        return 1;
    }else if(-x + y > edgeY + holeR - holeD3 && -x + y < edgeY && -x - y < -edgeY - holeR + 1.414f * balls[0].r && -x - y > -edgeY - holeR - 1.414f * balls[0].r){
        *dx = cosf(-0.75f * PI);
        *dy = sinf(-0.75f * PI);
        return 1;
    }else if(x - y > edgeY + holeR - holeD3 && x - y < edgeY && x + y < -edgeY - holeR + 1.414f * balls[0].r && x + y > -edgeY - holeR - 1.414f * balls[0].r){
        *dx = cosf(0.25f * PI);
        *dy = sinf(0.25f * PI);
        return 1;
    }else if(-x - y > edgeY + holeR - holeD3 && -x - y < edgeY && -x + y < -edgeY - holeR + 1.414f * balls[0].r && -x + y > -edgeY - holeR - 1.414f * balls[0].r){
        *dx = cosf(0.75f * PI);
        *dy = sinf(0.75f * PI);
        return 1;
    }else
        return 0;
}
void collision(float t){
    vector<ball> tmpBalls;
    vector<vector<int>> tmpDRelation, dRelation, rRelation, tmpRRelation;
    vector<int> tmpERelation(balls.size(), 0), tmpCRelation(balls.size(), 0), eRelation(balls.size(), 0), cRelation(balls.size(), 0);
    float currentT = 0.0f, nextT = t, tmp, DeltaX, DeltaY, dx, dy, deltaVx, deltaVy, rvx, rvy, rv, rvx2, rvy2, rv2, DeltaX2, DeltaY2, Delta, DeltaX3, DeltaY3, DeltaX4, DeltaY4;
    float Ib, Is, Ix, Iy, Ix2, Iy2, currentK, nextK, Ix3, Iy3, ratio, frictionAngle, frictionAngle2, frictionAngle3, frictionAngle4, rejectRatio;
    int crossD, crossR, crossE, crossC, state1, state2, state3, state4, reject;
    for(int i=0;i<balls.size()-1;i++){
        dRelation.push_back(vector<int>(balls.size(), 0));
        rRelation.push_back(vector<int>(balls.size(), 0));
        tmpDRelation.push_back(vector<int>(balls.size(), 0));
        tmpRRelation.push_back(vector<int>(balls.size(), 0));
    }
    tmpBalls.clear();
    for(int i=0;i<balls.size();i++)
        tmpBalls.push_back(balls[i]);

    for(int i=0;i<balls.size()-1;i++){
        for(int j=i+1;j<balls.size();j++){
            if((balls[i].vx - balls[j].vx) * (balls[i].x - balls[j].x) + (balls[i].vy - balls[j].vy) * (balls[i].y - balls[j].y) > 0.0f)
                dRelation[i][j] = 1;
            else
                dRelation[i][j] = 0;
            if(powf(balls[i].x - balls[j].x, 2) + powf(balls[i].y - balls[j].y, 2) <= powf(2 * balls[0].r, 2))
                rRelation[i][j] = 1;
            else
                rRelation[i][j] = 0;
        }
    }
    for(int i=0;i<balls.size();i++){
        eRelation[i] = edgeCollision(balls[i].x, balls[i].y, balls[0].r, &dx, &dy);
        if(balls[i].x < -edgeX || balls[i].x > edgeX || balls[i].y < -edgeY || balls[i].y > edgeY)
            cRelation[i] = 1;
        else
            cRelation[i] = 0;
    }
    while(currentT < t){
        for(int i=0;i<balls.size();i++)
            tmpBalls[i] = balls[i];
        updateWithDetect(nextT);

        for(int i=0;i<balls.size()-1;i++){
            for(int j=i+1;j<balls.size();j++){
                if((balls[i].vx - balls[j].vx) * (balls[i].x - balls[j].x) + (balls[i].vy - balls[j].vy) * (balls[i].y - balls[j].y) > 0.0f)
                    tmpDRelation[i][j] = 1;
                else
                    tmpDRelation[i][j] = 0;
                if(powf(balls[i].x - balls[j].x, 2) + powf(balls[i].y - balls[j].y, 2) <= powf(2 * balls[0].r, 2))
                    tmpRRelation[i][j] = 1;
                else
                    tmpRRelation[i][j] = 0;
            }
        }
        for(int i=0;i<balls.size();i++){
            tmpERelation[i] = edgeCollision(balls[i].x, balls[i].y, balls[0].r, &dx, &dy);
            if(balls[i].x < -edgeX || balls[i].x > edgeX || balls[i].y < -edgeY || balls[i].y > edgeY)
                tmpCRelation[i] = 1;
            else
                tmpCRelation[i] = 0;
        }
        crossR = 0;
        crossE = 0;
        crossC = 0;
        crossD = 0;
        for(int i=0;i<balls.size()-1;i++){
            for(int j=i+1;j<balls.size();j++){
                if(tmpRRelation[i][j] != rRelation[i][j])
                    crossR = 1;
                if(tmpDRelation[i][j] != dRelation[i][j])
                    crossD = 1;
            }
        }
        for(int i=0;i<balls.size();i++){
            if(tmpERelation[i] != eRelation[i])
                crossE = 1;
            if(tmpCRelation[i] != cRelation[i])
                crossC = 1;
        }
        if(((crossR || crossE) && nextT > 1e-5f) || ((crossD || crossC) && nextT > 1e-4f)){
            nextT *= kFail;
            for(int i=0;i<tmpBalls.size();i++)
                balls[i] = tmpBalls[i];
        }else{
            currentT += nextT;
            nextT *= kSuccess;
            if(nextT > t - currentT)
                nextT = t - currentT;
            for(int i=0;i<balls.size()-1;i++){
                for(int j=i+1;j<balls.size();j++){
                    if(sqrtf(powf(balls[i].x - balls[j].x, 2) + powf(balls[i].y - balls[j].y, 2)) - 2 * balls[0].r < 1e-4f){
                        DeltaX = 0.5 * (balls[i].x - balls[j].x) / balls[0].r;
                        DeltaY = 0.5 * (balls[i].y - balls[j].y) / balls[0].r;
                        tmp = DeltaX * (balls[i].vx - balls[j].vx) + DeltaY * (balls[i].vy - balls[j].vy);
                        if(tmp < 0.0f){
                            currentK = 0.0f;
                            nextK = 1.0f;
                            tmpBalls[0] = balls[i];
                            tmpBalls[1] = balls[j];
                            rvx = (tmpBalls[0].vy - tmpBalls[1].vy) * DeltaX - (tmpBalls[0].vx - tmpBalls[1].vx) * DeltaY - (tmpBalls[0].wz + tmpBalls[1].wz) * balls[0].r;
                            rvy = (-(tmpBalls[0].wx + tmpBalls[1].wx) * DeltaY + (tmpBalls[0].wy + tmpBalls[1].wy) * DeltaX) * balls[0].r;
                            if(fabsf(rvx) > epsilon || fabsf(rvy) > epsilon){
                                rv = sqrtf(rvx * rvx + rvy * rvy);
                                Ix = -tmp * nextK * balls[0].m * mu3 * rvx / rv;
                                Iy = -tmp * nextK * balls[0].m * mu3 * rvy / rv;
                                frictionAngle = atan2f(rvy, rvx);
                                state1 = 1;
                            }else{
                                Ix = 0.0f;
                                Iy = 0.0f;
                                state1 = 0;
                            }
                            while(currentK < 1.0f){
                                tmpBalls[0].vx += tmp * nextK * -DeltaX + Ix / balls[0].m * DeltaY;
                                tmpBalls[0].vy += tmp * nextK * -DeltaY + Ix / balls[0].m * -DeltaX;
                                tmpBalls[1].vx += tmp * nextK * DeltaX  + Ix / balls[0].m * -DeltaY;
                                tmpBalls[1].vy += tmp * nextK * DeltaY  + Ix / balls[0].m * DeltaX;
                                tmpBalls[0].wz += Ix * balls[0].r / balls[0].I;
                                tmpBalls[1].wz += Ix * balls[0].r / balls[0].I;
                                tmpBalls[0].wx += Iy * balls[0].r / balls[0].I * DeltaY;
                                tmpBalls[1].wx += Iy * balls[0].r / balls[0].I * DeltaY;
                                tmpBalls[0].wy += Iy * balls[0].r / balls[0].I * -DeltaX;
                                tmpBalls[1].wy += Iy * balls[0].r / balls[0].I * -DeltaX;

                                reject = 0;
                                rejectRatio = 10.0f;
                                rvx2 = (tmpBalls[0].vy - tmpBalls[1].vy) * DeltaX - (tmpBalls[0].vx - tmpBalls[1].vx) * DeltaY - (tmpBalls[0].wz + tmpBalls[1].wz) * balls[0].r;
                                rvy2 = (-(tmpBalls[0].wx + tmpBalls[1].wx) * DeltaY + (tmpBalls[0].wy + tmpBalls[1].wy) * DeltaX) * balls[0].r;
                                frictionAngle2 = atan2f(rvy2, rvx2);
                                if(state1 == 1 && (fabsf(frictionAngle2 - frictionAngle) > PI / 18.0f)){
                                    reject = 1;
                                }

                                if(reject && nextK > 1e-2f){
                                    nextK *= kFail;
                                    tmpBalls[0] = balls[i];
                                    tmpBalls[1] = balls[j];
                                }else{
                                    balls[i] = tmpBalls[0];
                                    balls[j] = tmpBalls[1];
                                    currentK += nextK;
                                    nextK *= kSuccess;
                                    if(nextK > 1.0f - currentK)
                                        nextK = 1.0f - currentK;
                                    if(currentK < 1.0f){
                                        rvx = (tmpBalls[0].vy - tmpBalls[1].vy) * DeltaX - (tmpBalls[0].vx - tmpBalls[1].vx) * DeltaY - (tmpBalls[0].wz + tmpBalls[1].wz) * balls[0].r;
                                        rvy = (-(tmpBalls[0].wx + tmpBalls[1].wx) * DeltaY + (tmpBalls[0].wy + tmpBalls[1].wy) * DeltaX) * balls[0].r;
                                        if(fabsf(rvx) > epsilon || fabsf(rvy) > epsilon){
                                            rv = sqrtf(rvx * rvx + rvy * rvy);
                                            Ix = -tmp * nextK * balls[0].m * mu3 * rvx / rv;
                                            Iy = -tmp * nextK * balls[0].m * mu3 * rvy / rv;
                                            frictionAngle = atan2f(rvy, rvx);
                                            state1 = 1;
                                        }else{
                                            Ix = 0.0f;
                                            Iy = 0.0f;
                                            state1 = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            for(int i=0;i<balls.size();i++){
                if(powf(balls[i].x - holeX, 2) + powf(balls[i].y - holeY, 2) < powf(0.5f * holeR, 2) ||
                powf(balls[i].x - holeX, 2) + powf(balls[i].y + holeY, 2) < powf(0.5f * holeR, 2) ||
                powf(balls[i].x + holeX, 2) + powf(balls[i].y - holeY, 2) < powf(0.5f * holeR, 2) ||
                powf(balls[i].x + holeX, 2) + powf(balls[i].y + holeY, 2) < powf(0.5f * holeR, 2) ||
                powf(balls[i].x, 2) + powf(balls[i].y - edgeY - 0.5f * holeR, 2) < powf(0.5f * holeR, 2) ||
                powf(balls[i].x, 2) + powf(balls[i].y + edgeY + 0.5f * holeR, 2) < powf(0.5f * holeR, 2) ||
                balls[i].x > edgeX || balls[i].x < -edgeX || balls[i].y > edgeY || balls[i].y < -edgeY){
                    balls[i].x = edgeX + 0.3f;
                    balls[i].y = 3.0f * balls[0].r * (i - 7);
                    balls[i].vx = balls[i].vy = balls[i].vz = 0.0f;
                    balls[i].wx = balls[i].wy = balls[i].wz = 0.0f;
                }
                if(edgeCollision(balls[i].x, balls[i].y, balls[0].r + 1e-4f, &dx, &dy)){
                    tmp = (balls[i].vx * dx + balls[i].vy * dy) / sqrtf(dx * dx + dy * dy);
                    if(tmp < 0.0f){
                        deltaVx = -tmp * dx / sqrtf(dx * dx + dy * dy);
                        deltaVy = -tmp * dy / sqrtf(dx * dx + dy * dy);
                        dx = -deltaVx / tmp;
                        dy = -deltaVy / tmp;
                        currentK = 0.0f;
                        nextK = 1.0f;
                        tmpBalls[0] = balls[i];

                        Is = -tmpBalls[0].m * (1.0f + collisionK) * tmp / cosf(edgeTheta);

                        DeltaX = (dx * tmpBalls[0].vy - dy * tmpBalls[0].vx) - tmpBalls[0].wz * tmpBalls[0].r * cosf(edgeTheta) - (tmpBalls[0].wx * dx + tmpBalls[0].wy * dy) * tmpBalls[0].r * (sinf(edgeTheta));
                        DeltaY = (-dx * tmpBalls[0].vx - dy * tmpBalls[0].vy) * sinf(edgeTheta) + (tmpBalls[0].wx * dy - tmpBalls[0].wy * dx) * tmpBalls[0].r;
                        if(fabsf(DeltaX) > epsilon || fabsf(DeltaY) > epsilon){
                            Delta = sqrtf(DeltaX * DeltaX + DeltaY * DeltaY);
                            frictionAngle = atan2f(DeltaY, DeltaX);
                            Ix = mu1 * Is * DeltaX / Delta;
                            Iy = mu1 * Is * DeltaY / Delta;
                            state1 = 1;
                        }else{
                            state1 = 0;
                            Ix = 0.0f;
                            Iy = 0.0f;
                        }
                        Ib = Is * sinf(edgeTheta) - Iy;
                        if(Ib < 0.0f) Ib = 0.0f;
                        DeltaX2 = (dx * tmpBalls[0].vy - dy * tmpBalls[0].vx) + (dx * tmpBalls[0].wx + dy * tmpBalls[0].wy) * tmpBalls[0].r;
                        DeltaY2 = (-dx * tmpBalls[0].vx - dy * tmpBalls[0].vy) + (-tmpBalls[0].wx * dy + tmpBalls[0].wy * dx) * tmpBalls[0].r;
                        if(fabsf(DeltaX2) > epsilon || fabsf(DeltaY2) > epsilon){
                            Delta = sqrtf(DeltaX2 * DeltaX2 + DeltaY2 * DeltaY2);
                            frictionAngle2 = atan2f(DeltaY2, DeltaX2);
                            Ix2 = mu1 * Ib * DeltaX2 / Delta;
                            Iy2 = mu1 * Ib * DeltaY2 / Delta;
                            state2 = 1;
                        }else{
                            state2 = 0;
                            Ix2 = 0.0f;
                            Iy2 = 0.0f;
                        }
                        while(currentK < 1.0f){
                            tmpBalls[0].vx += ((1.0f + collisionK) * deltaVx + (Iy * sinf(edgeTheta) + Iy2) / tmpBalls[0].m * dx + (Ix + Ix2) / tmpBalls[0].m * dy) * nextK;
                            tmpBalls[0].vy += ((1.0f + collisionK) * deltaVy + (Iy * sinf(edgeTheta) + Iy2) / tmpBalls[0].m * dy - (Ix + Ix2) / tmpBalls[0].m * dx) * nextK;
                            tmpBalls[0].wx += ((Ix * sinf(edgeTheta) - Ix2) * tmpBalls[0].r / tmpBalls[0].I * dx + (-Iy + Iy2) * tmpBalls[0].r / tmpBalls[0].I * dy) * nextK;
                            tmpBalls[0].wy += ((Ix * sinf(edgeTheta) - Ix2) * tmpBalls[0].r / tmpBalls[0].I * dy + (Iy - Iy2) * tmpBalls[0].r / tmpBalls[0].I * dx) * nextK;
                            tmpBalls[0].wz += ((Ix * cosf(edgeTheta)) * tmpBalls[0].r / tmpBalls[0].I) * nextK;

                            reject = 0;
                            DeltaX3 = (dx * tmpBalls[0].vy - dy * tmpBalls[0].vx) - tmpBalls[0].wz * tmpBalls[0].r * cosf(edgeTheta) - (tmpBalls[0].wx * dx + tmpBalls[0].wy * dy) * tmpBalls[0].r * (sinf(edgeTheta));
                            DeltaY3 = (-dx * tmpBalls[0].vx - dy * tmpBalls[0].vy) * sinf(edgeTheta) + (tmpBalls[0].wx * dy - tmpBalls[0].wy * dx) * tmpBalls[0].r;
                            frictionAngle3 = atan2f(DeltaY3, DeltaX3);
                            if(state1 == 1 && (fabsf(frictionAngle3 - frictionAngle) > PI / 18.0f)){
                                reject = 1;
                            }else if(state1 == 0 && (fabsf(DeltaX3) > epsilon || fabsf(DeltaY3) > epsilon)){
                                reject = 1;
                            }

                            DeltaX4 = (dx * tmpBalls[0].vy - dy * tmpBalls[0].vx) + (dx * tmpBalls[0].wx + dy * tmpBalls[0].wy) * tmpBalls[0].r;
                            DeltaY4 = (-dx * tmpBalls[0].vx - dy * tmpBalls[0].vy) + (-tmpBalls[0].wx * dy + tmpBalls[0].wy * dx) * tmpBalls[0].r;
                            frictionAngle4 = atan2f(DeltaY4, DeltaX4);
                            if(state2 == 1 && (fabsf(frictionAngle4 - frictionAngle2) > PI / 18.0f)){
                                reject = 1;
                            }else if(state2 == 0 && (fabsf(DeltaX4) > epsilon || fabsf(DeltaY4) > epsilon)){
                                reject = 1;
                            }


                            if(reject && nextK > 1e-2f){
                                nextK *= kFail;
                                tmpBalls[0] = balls[i];
                            }else{
                                balls[i] = tmpBalls[0];
                                currentK += nextK;
                                nextK *= kSuccess;
                                if(nextK > 1.0f - currentK)
                                    nextK = 1.0f - currentK;
                                if(currentK < 1.0f){
                                    DeltaX = (dx * tmpBalls[0].vy - dy * tmpBalls[0].vx) - tmpBalls[0].wz * tmpBalls[0].r * cosf(edgeTheta) - (tmpBalls[0].wx * dx + tmpBalls[0].wy * dy) * tmpBalls[0].r * (sinf(edgeTheta));
                                    DeltaY = (-dx * tmpBalls[0].vx - dy * tmpBalls[0].vy) * sinf(edgeTheta) + (tmpBalls[0].wx * dy - tmpBalls[0].wy * dx) * tmpBalls[0].r;
                                    if(fabsf(DeltaX) > epsilon || fabsf(DeltaY) > epsilon){
                                        Delta = sqrtf(DeltaX * DeltaX + DeltaY * DeltaY);
                                        frictionAngle = atan2f(DeltaY, DeltaX);
                                        Ix = mu1 * Is * DeltaX / Delta;
                                        Iy = mu1 * Is * DeltaY / Delta;
                                        state1 = 1;
                                    }else{
                                        state1 = 0;
                                        Ix = 0.0f;
                                        Iy = 0.0f;
                                    }
                                    DeltaX2 = (dx * tmpBalls[0].vy - dy * tmpBalls[0].vx) + (dx * tmpBalls[0].wx + dy * tmpBalls[0].wy) * tmpBalls[0].r;
                                    DeltaY2 = (-dx * tmpBalls[0].vx - dy * tmpBalls[0].vy) + (-tmpBalls[0].wx * dy + tmpBalls[0].wy * dx) * tmpBalls[0].r;
                                    if(fabsf(DeltaX2) > epsilon || fabsf(DeltaY2) > epsilon){
                                        Delta = sqrtf(DeltaX2 * DeltaX2 + DeltaY2 * DeltaY2);
                                        frictionAngle2 = atan2f(DeltaY2, DeltaX2);
                                        Ix2 = mu1 * Ib * DeltaX2 / Delta;
                                        Iy2 = mu1 * Ib * DeltaY2 / Delta;
                                        state2 = 1;
                                    }else{
                                        state2 = 0;
                                        Ix2 = 0.0f;
                                        Iy2 = 0.0f;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(currentT < t){
                for(int i=0;i<balls.size()-1;i++){
                    for(int j=i+1;j<balls.size();j++){
                        if((balls[i].vx - balls[j].vx) * (balls[i].x - balls[j].x) + (balls[i].vy - balls[j].vy) * (balls[i].y - balls[j].y) > 0.0f)
                            dRelation[i][j] = 1;
                        else
                            dRelation[i][j] = 0;
                        if(powf(balls[i].x - balls[j].x, 2) + powf(balls[i].y - balls[j].y, 2) <= powf(2 * balls[0].r, 2))
                            rRelation[i][j] = 1;
                        else
                            rRelation[i][j] = 0;
                    }
                }
                for(int i=0;i<balls.size();i++){
                    eRelation[i] = edgeCollision(balls[i].x, balls[i].y, balls[0].r, &dx, &dy);
                    if(balls[i].x < -edgeX || balls[i].x > edgeX || balls[i].y < -edgeY || balls[i].y > edgeY)
                        cRelation[i] = 1;
                    else
                        cRelation[i] = 0;
                }
            }
        }
    }
}