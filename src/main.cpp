#define STB_IMAGE_IMPLEMENTATION
#include"../inc/stb_image.h"
#include"stdio.h"
#include"stdlib.h"
#include<vector>
#include<math.h>
#include"time.h"
#include<GL/glew.h>
#include<GL/glut.h>
using namespace std;
#include"../inc/ball.hpp"
#include"../inc/main.hpp"
#include"../inc/calc.hpp"
int WINDOW_WIDTH = 1800;
int WINDOW_HEIGHT = 900;
int shadowWidth = 1800;
int shadowHeight = 900;
int allowHit = 3, mouseState, keyState;
float viewYaw = 0.0f, viewPitch = 0.0f, viewDistance = 1.5f, hitDistance = 0.0f, hitSpeed = 0.0f;
float hitYaw = 0.0f, hitPitch = 0.0f, cuePitch, setCuePitch = PI / 36.0f, setHitPitch;
float viewX = 0.0f, viewY = 0.0f, viewZ = 0.0f;
float lightPM[16], lightVM[16];
float calcTime = 0.02f;
float tmpSpeed;
unsigned int texture[16];
unsigned int planeCoordBO, planeTexBO, planeNormBO, planeVAO;
unsigned int cueCoordBO, cueTexBO, cueNormBO, cueVAO;
unsigned int ballCoordBO, ballTexBO, ballNormBO, ballVAO;
unsigned int shadowTexture;
vector<float> planeOrderedCoord, planeOrderedNorm, planeOrderedTex;
vector<float> ballOrderedCoord, ballOrderedNorm, ballOrderedTex;
vector<float> cueOrderedCoord, cueOrderedNorm, cueOrderedTex;
vector<float> planeCoord, planeNorm, planeTex, ballCoord, ballNorm, ballTex, cueCoord, cueNorm, cueTex;
vector<unsigned int> planeCoordIndex, planeNormIndex, planeTexIndex, ballCoordIndex, ballNormIndex, ballTexIndex, cueNormIndex, cueCoordIndex, cueTexIndex;
GLfloat color[4];
struct timespec tStart, tEnd;
void quatRotate(float p[], float wx, float wy, float wz, float t){
    float q[4], w, tmp, mid[4];
    w = sqrtf(wx * wx + wy * wy + wz * wz);
    if(w < epsilon) return;
    q[0] = cosf(0.5f * w * t);
    q[1] = sinf(0.5f * w * t) * wx / w;
    q[2] = sinf(0.5f * w * t) * wy / w;
    q[3] = sinf(0.5f * w * t) * wz / w;
    mid[0] = q[0] * p[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3];
    mid[1] = q[0] * p[1] + q[1] * p[0] + q[2] * p[3] - q[3] * p[2];
    mid[2] = q[0] * p[2] + q[2] * p[0] + q[3] * p[1] - q[1] * p[3];
    mid[3] = q[0] * p[3] + q[3] * p[0] + q[1] * p[2] - q[2] * p[1];
    tmp = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    q[0] = q[0] / tmp;
    q[1] = -q[1] / tmp;
    q[2] = -q[2] / tmp;
    q[3] = -q[3] / tmp;
    p[0] = mid[0] * q[0] - mid[1] * q[1] - mid[2] * q[2] - mid[3] * q[3];
    p[1] = mid[0] * q[1] + mid[1] * q[0] + mid[2] * q[3] - mid[3] * q[2];
    p[2] = mid[0] * q[2] + mid[2] * q[0] + mid[3] * q[1] - mid[1] * q[3];
    p[3] = mid[0] * q[3] + mid[3] * q[0] + mid[1] * q[2] - mid[2] * q[1];
}
void getRotation(float quat0[], float quat1[], float *angleX, float vecX[], float *angleY, float vecY[]){
    float norm0, norm1;
    float quatX[] = {0.0f, 1.0f, 0.0f, 0.0f};
    float quatY[] = {0.0f, 0.0f, 1.0f, 0.0f};
    norm0 = 0.0f;
    norm1 = 0.0f;
    for(int i=1;i<4;i++){
        norm0 += quatX[i] * quatX[i];
        norm1 += quat0[i] * quat0[i];
    }
    vecX[0] = (quatX[2] * quat0[3] - quatX[3] * quat0[2]);
    vecX[1] = (quatX[3] * quat0[1] - quatX[1] * quat0[3]);
    vecX[2] = (quatX[1] * quat0[2] - quatX[2] * quat0[1]);
    if(sqrtf(norm0 * norm1) < epsilon)
        *angleX = 0.0f;
    else
        *angleX = acosf((quatX[1] * quat0[1] + quatX[2] * quat0[2] + quatX[3] * quat0[3]) / sqrtf(norm0 * norm1));
    norm0 = sqrtf(vecX[0] * vecX[0] + vecX[1] * vecX[1] + vecX[2] * vecX[2]);
    if(norm0 < epsilon){
        vecX[0] = 1.0f;
        vecX[1] = 0.0f;
        vecX[2] = 0.0f;
    }else{
        vecX[0] /= norm0;
        vecX[1] /= norm0;
        vecX[2] /= norm0;
    }
    quatRotate(quatY, vecX[0], vecX[1], vecX[2], *angleX);
    norm0 = 0.0f;
    norm1 = 0.0f;
    for(int i=1;i<4;i++){
        norm0 += quatY[i] * quatY[i];
        norm1 += quat1[i] * quat1[i];
    }
    vecY[0] = (quatY[2] * quat1[3] - quatY[3] * quat1[2]);
    vecY[1] = (quatY[3] * quat1[1] - quatY[1] * quat1[3]);
    vecY[2] = (quatY[1] * quat1[2] - quatY[2] * quat1[1]);
    if(sqrtf(norm0 * norm1) < epsilon)
        *angleY = 0.0f;
    else
        *angleY = acosf((quatY[1] * quat1[1] + quatY[2] * quat1[2] + quatY[3] * quat1[3]) / sqrtf(norm0 * norm1));
    norm0 = sqrtf(vecY[0] * vecY[0] + vecY[1] * vecY[1] + vecY[2] * vecY[2]);
    if(norm0 < epsilon){
        vecY[0] = 1.0f;
        vecY[1] = 0.0f;
        vecY[2] = 0.0f;
    }else{
        vecY[0] /= norm0;
        vecY[1] /= norm0;
        vecY[2] /= norm0;
    }
}
void drawPlane(int light){
    GLfloat specular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glPushMatrix();
    glTranslatef(0.0f, -balls[0].r, 0.0f);
    glColor4f(0.075f, 0.225f, 0.075f, 1.0f);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glBindVertexArray(planeVAO);
    glDrawArrays(GL_TRIANGLES, 0, planeCoordIndex.size());
    glBindVertexArray(0);
    glPopMatrix();
}
void drawBalls(int light){
    float vecX[3], vecY[3], angleX, angleY;
    GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128);
    if(light) glEnable(GL_TEXTURE_2D);
    for(int i=0;i<=15;i++){
        glPushMatrix();
        glTranslatef(balls[i].x, balls[i].z, -balls[i].y);
        getRotation(balls[i].quatX, balls[i].quatY, &angleX, vecX, &angleY, vecY);
        glRotatef(angleX * 180.0f / PI, vecX[0], vecX[2], -vecX[1]);
        if(balls[i].quatX[1] * vecY[0] + balls[i].quatX[2] * vecY[1] + balls[i].quatX[3] * vecY[2] > 0.0f)
            glRotatef(angleY * 180.0f / PI, 1.0f, 0.0f, 0.0f);
        else
            glRotatef(angleY * 180.0f / PI, -1.0f, 0.0f, 0.0f);

        if(light) glBindTexture(GL_TEXTURE_2D, texture[i]);
        glBindVertexArray(ballVAO);
        glDrawArrays(GL_TRIANGLES, 0, ballCoordIndex.size());
        glBindVertexArray(0);
        glPopMatrix();
    }
    if(light) glDisable(GL_TEXTURE_2D);
}
void drawCue(int light){
    GLfloat specular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    if(allowHit == 3){
        glPushMatrix();
        glTranslatef(balls[0].x + (balls[0].r) * cosf(hitPitch) * cosf(viewYaw + hitYaw), balls[0].r * sinf(hitPitch), -(balls[0].y + (balls[0].r) * cosf(hitPitch) * sinf(viewYaw + hitYaw)));
        glRotatef((viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f)) / PI * 180, 0.0f, 1.0f, 0.0f);
        glRotatef(90.0f + cuePitch * 180.0f / PI, 0.0f, 0.0f, 1.0f);
        glTranslatef(0.0f, hitDistance, 0.0f);
        glColor4f(0.2f, 0.2f, 0.2f, 1.0f);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0);
        glBindVertexArray(cueVAO);
        glDrawArrays(GL_TRIANGLES, 0, cueCoordIndex.size());
        glBindVertexArray(0);
        glPopMatrix();
    }
}
void multiplyMatrix4(float* matrix1,float* matrix2,float* result)
{
    result[0] = matrix1[0]*matrix2[0] + matrix1[1]*matrix2[4] + matrix1[2]*matrix2[8] + matrix1[3]*matrix2[12];
    result[1] = matrix1[0]*matrix2[1] + matrix1[1]*matrix2[5] + matrix1[2]*matrix2[9] + matrix1[3]*matrix2[13];
    result[2] = matrix1[0]*matrix2[2] + matrix1[1]*matrix2[6] + matrix1[2]*matrix2[10] + matrix1[3]*matrix2[14];
    result[3] = matrix1[0]*matrix2[3] + matrix1[1]*matrix2[7] + matrix1[2]*matrix2[11] + matrix1[3]*matrix2[15];

    result[4] = matrix1[4]*matrix2[0] + matrix1[5]*matrix2[4] + matrix1[6]*matrix2[8] + matrix1[7]*matrix2[12];
    result[5] = matrix1[4]*matrix2[1] + matrix1[5]*matrix2[5] + matrix1[6]*matrix2[9] + matrix1[7]*matrix2[13];
    result[6] = matrix1[4]*matrix2[2] + matrix1[5]*matrix2[6] + matrix1[6]*matrix2[10] + matrix1[7]*matrix2[14];
    result[7] = matrix1[4]*matrix2[3] + matrix1[5]*matrix2[7] + matrix1[6]*matrix2[11] + matrix1[7]*matrix2[15];

    result[8] = matrix1[8]*matrix2[0] + matrix1[9]*matrix2[4] + matrix1[10]*matrix2[8] + matrix1[11]*matrix2[12];
    result[9] = matrix1[8]*matrix2[1] + matrix1[9]*matrix2[5] + matrix1[10]*matrix2[9] + matrix1[11]*matrix2[13];
    result[10] = matrix1[8]*matrix2[2] + matrix1[9]*matrix2[6] + matrix1[10]*matrix2[10] + matrix1[11]*matrix2[14];
    result[11] = matrix1[8]*matrix2[3] + matrix1[9]*matrix2[7] + matrix1[10]*matrix2[11] + matrix1[11]*matrix2[15];

    result[12] = matrix1[12]*matrix2[0] + matrix1[13]*matrix2[4] + matrix1[14]*matrix2[8] + matrix1[15]*matrix2[12];
    result[13] = matrix1[12]*matrix2[1] + matrix1[13]*matrix2[5] + matrix1[14]*matrix2[9] + matrix1[15]*matrix2[13];
    result[14] = matrix1[12]*matrix2[2] + matrix1[13]*matrix2[6] + matrix1[14]*matrix2[10] + matrix1[15]*matrix2[14];
    result[15] = matrix1[12]*matrix2[3] + matrix1[13]*matrix2[7] + matrix1[14]*matrix2[11] + matrix1[15]*matrix2[15];
}
void display(){
    GLfloat lightPos[] = {0.0f, 3.0f, 0.0f, 1.0f};
    GLfloat lightDiffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightDiffuseShadow[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat lightAmbient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat lightSpecular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightSpecularShadow[] = {0.0f, 0.0f, 0.0f, 1.0f};
    float bias[]={0.5f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.5f, 0.0f,
        0.5f, 0.5f, 0.5f, 1.0f};
    float texM[16], tpM[16];
    float column1[4], column2[4], column3[4], column4[4];
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glCullFace(GL_FRONT);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(lightPM);
    glViewport(0, 0, shadowWidth, shadowHeight);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(lightVM);
    glColorMask(0, 0, 0, 0);
    glDisable(GL_LIGHTING);
    drawPlane(0);
    drawBalls(0);
    drawCue(0);
    glEnable(GL_LIGHTING);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, shadowTexture);
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, shadowWidth, shadowHeight);
    glActiveTexture(GL_TEXTURE0);
    glColorMask(1, 1, 1, 1);
    glCullFace(GL_BACK);

    glClear(GL_DEPTH_BUFFER_BIT);
    if(allowHit == 2){
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(20.0f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 50.0f);
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f);
    }else{
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(20.0f, (float)WINDOW_WIDTH / WINDOW_HEIGHT / 1.5f, 0.1f, 50.0f);
        glViewport(0, -WINDOW_HEIGHT / 2, WINDOW_WIDTH, 1.5f * WINDOW_HEIGHT);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        viewX = viewX * 0.95f + balls[0].x * 0.05f;
        viewY = viewY * 0.95f + balls[0].y * 0.05f;
        viewZ = viewZ * 0.95f + balls[0].z * 0.05f;
        gluLookAt(viewX + viewDistance * cosf(viewPitch) * cosf(viewYaw), viewZ + viewDistance * sinf(viewPitch), -viewY - viewDistance * cosf(viewPitch) * sinf(viewYaw), viewX, viewZ, -viewY, 0.0f, 1.0f, 0.0f);
    }

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuseShadow);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecularShadow);
    drawPlane(1);
    drawBalls(1);
    drawCue(1);

    multiplyMatrix4(lightPM, bias, tpM);
    multiplyMatrix4(lightVM, tpM, texM);
    column1[0] = texM[0];
    column1[1] = texM[4];
    column1[2] = texM[8];
    column1[3] = texM[12];
    column2[0] = texM[1];
    column2[1] = texM[5];
    column2[2] = texM[9];
    column2[3] = texM[13];
    column3[0] = texM[2];
    column3[1] = texM[6];
    column3[2] = texM[10];
    column3[3] = texM[14];
    column4[0] = texM[3];
    column4[1] = texM[7];
    column4[2] = texM[11];
    column4[3] = texM[15];
    glActiveTexture(GL_TEXTURE1);
    glEnable(GL_TEXTURE_GEN_S);
    glEnable(GL_TEXTURE_GEN_T);
    glEnable(GL_TEXTURE_GEN_R);
    glEnable(GL_TEXTURE_GEN_Q);
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGenfv(GL_S, GL_EYE_PLANE, column1);
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGenfv(GL_T, GL_EYE_PLANE, column2);
    glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGenfv(GL_R, GL_EYE_PLANE, column3);
    glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGenfv(GL_Q, GL_EYE_PLANE, column4);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, shadowTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LESS);
    glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_ALPHA);
    glActiveTexture(GL_TEXTURE0);

    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GEQUAL, 0.5f);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    drawPlane(1);
    drawBalls(1);
    drawCue(1);
    glDisable(GL_ALPHA_TEST);

    glutSwapBuffers();
}
int secCrossCircle(float r, float x1, float y1, float x2, float y2){
    float d1, d2, d3, prod, dis, p1, p2;
    prod = x1 * (y2 - y1) - y1 * (x2 - x1);
    d1 = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    d2 = sqrtf(x1 * x1 + y1 * y1);
    d3 = sqrtf(x2 * x2 + y2 * y2);
    dis = prod / d1;
    if(dis > r) return 0;
    if(d2 > r + d1) return 0;
    if(d3 > r + d1) return 0;
    p1 = (x2 - x1) * x1 + (y2 - y1) * y1;
    p2 = (x2 - x1) * x2 + (y2 - y1) * y2;
    if(p1 * p2 < 0.0f) return 1;
    if(d2 < r || d3 < r) return 1;
    return 0;
}
int secCrossEllipse(float a, float b, float dirX, float dirY, float x1, float y1, float x2, float y2){
    float dotProd, px, py, vx, vy, x1t, y1t, x2t, y2t;
    dotProd = dirX * x1 + dirY * y1;
    px = dotProd * dirX;
    py = dotProd * dirY;
    vx = x1 - px;
    vy = y1 - py;
    x1t = px / a * b + vx;
    y1t = py / a * b + vy;
    dotProd = dirX * x2 + dirY * y2;
    px = dotProd * dirX;
    py = dotProd * dirY;
    vx = x2 - px;
    vy = y2 - py;
    x2t = px / a * b + vx;
    y2t = py / a * b + vy;
    return secCrossCircle(b, x1t, y1t, x2t, y2t);
}
int validCueWithEdge(float x, float y, float a, float b, float dirX, float dirY){
    if(x > holeX + holeR * 0.5f) return 0;
    if(x < -holeX - holeR * 0.5f) return 0;
    if(y > edgeY + holeR) return 0;
    if(y < -edgeY - holeR) return 0;
    if(y < -edgeY && x > -edgeX + holeD2 && x < -holeD3) return 0;
    if(y < -edgeY && x > holeD3 && x < edgeX - holeD2) return 0;
    if(y > edgeY && x > -edgeX + holeD2 && x < -holeD3) return 0;
    if(y > edgeY && x > holeD3 && x < edgeX - holeD2) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -edgeX + holeD2 - x, -edgeY - y, -holeD3 - x, -edgeY - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, holeD3 - x, -edgeY - y, edgeX - holeD2 - x, -edgeY - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, edgeX - holeD2 - x, edgeY - y, holeD3 - x, edgeY - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -holeD3 - x, edgeY - y, -edgeX + holeD2 - x, edgeY - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, edgeX - x, -edgeY + holeD2 - y, edgeX - x, edgeY - holeD2 - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -edgeX - x, edgeY - holeD2 - y, -edgeX - x, -edgeY + holeD2 - y)) return 0;
    for(float i=0.75f*PI;i>0.5f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -edgeX + holeD2 + holeR * cosf(i) - x, -edgeY - holeR + holeR * sinf(i) - y, -edgeX + holeD2 + holeR * cosf(i - 0.1f) - x, -edgeY - holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=0.5f*PI;i>0.25f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -holeD3 + holeR * cosf(i) - x, -edgeY - holeR + holeR * sinf(i) - y, -holeD3 + holeR * cosf(i - 0.1f) - x, -edgeY - holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=0.75f*PI;i>0.5f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, holeD3 + holeR * cosf(i) - x, -edgeY - holeR + holeR * sinf(i) - y, holeD3 + holeR * cosf(i - 0.1f) - x, -edgeY - holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=0.5f*PI;i>0.25f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, edgeX - holeD2 + holeR * cosf(i) - x, -edgeY - holeR + holeR * sinf(i) - y, edgeX - holeD2 + holeR * cosf(i - 0.1f) - x, -edgeY - holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=0.0f*PI;i>-0.25f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -edgeX - holeR + holeR * cosf(i) - x, -edgeY + holeD2 + holeR * sinf(i) - y, -edgeX - holeR + holeR * cosf(i - 0.1f) - x, -edgeY + holeD2 + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=1.25f*PI;i>1.0f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, edgeX + holeR + holeR * cosf(i) - x, -edgeY + holeD2 + holeR * sinf(i) - y, edgeX + holeR + holeR * cosf(i - 0.1f) - x, -edgeY + holeD2 + holeR * sinf(i - 0.1f) - y)) return 0;

    for(float i=-0.25f*PI;i>-0.5f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, edgeX - holeD2 + holeR * cosf(i) - x, edgeY + holeR + holeR * sinf(i) - y, edgeX - holeD2 + holeR * cosf(i - 0.1f) - x, edgeY + holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=-0.5f*PI;i>-0.75f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, holeD3 + holeR * cosf(i) - x, edgeY + holeR + holeR * sinf(i) - y, holeD3 + holeR * cosf(i - 0.1f) - x, edgeY + holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=-0.25f*PI;i>-0.5f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -holeD3 + holeR * cosf(i) - x, edgeY + holeR + holeR * sinf(i) - y, -holeD3 + holeR * cosf(i - 0.1f) - x, edgeY + holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=-0.5f*PI;i>-0.75f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -edgeX + holeD2 + holeR * cosf(i) - x, edgeY + holeR + holeR * sinf(i) - y, -edgeX + holeD2 + holeR * cosf(i - 0.1f) - x, edgeY + holeR + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=1.0f*PI;i>0.75f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, edgeX + holeR + holeR * cosf(i) - x, edgeY - holeD2 + holeR * sinf(i) - y, edgeX + holeR + holeR * cosf(i - 0.1f) - x, edgeY - holeD2 + holeR * sinf(i - 0.1f) - y)) return 0;
    for(float i=0.25f*PI;i>0.0f*PI;i-=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -edgeX - holeR + holeR * cosf(i) - x, edgeY - holeD2 + holeR * sinf(i) - y, -edgeX - holeR + holeR * cosf(i - 0.1f) - x, edgeY - holeD2 + holeR * sinf(i - 0.1f) - y)) return 0;

    for(float i=-1.25f*PI;i<-0.25f*PI;i+=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -holeX + holeR * 0.5f * cosf(i) - x, -holeY + holeR * 0.5f * sinf(i) - y, -holeX + holeR * 0.5f * cosf(i + 0.1f) - x, -holeY + holeR * 0.5f * sinf(i + 0.1f) - y)) return 0;
    for(float i=-0.75f*PI;i<0.25f*PI;i+=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, holeX + holeR * 0.5f * cosf(i) - x, -holeY + holeR * 0.5f * sinf(i) - y, holeX + holeR * 0.5f * cosf(i + 0.1f) - x, -holeY + holeR * 0.5f * sinf(i + 0.1f) - y)) return 0;
    for(float i=-0.25f*PI;i<0.75f*PI;i+=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, holeX + holeR * 0.5f * cosf(i) - x, holeY + holeR * 0.5f * sinf(i) - y, holeX + holeR * 0.5f * cosf(i + 0.1f) - x, holeY + holeR * 0.5f * sinf(i + 0.1f) - y)) return 0;
    for(float i=0.25f*PI;i<1.25f*PI;i+=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, -holeX + holeR * 0.5f * cosf(i) - x, holeY + holeR * 0.5f * sinf(i) - y, -holeX + holeR * 0.5f * cosf(i + 0.1f) - x, holeY + holeR * 0.5f * sinf(i + 0.1f) - y)) return 0;

    for(float i=1.0f*PI;i<2.0f*PI;i+=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, holeR * 0.5f * cosf(i) - x, -holeY - 0.5f * holeR * 0.5f + holeR * 0.5f * sinf(i) - y, holeR * 0.5f * cosf(i + 0.1f) - x, -holeY - 0.5f * holeR * 0.5f + holeR * 0.5f * sinf(i + 0.1f) - y)) return 0;
    for(float i=0.0f*PI;i<1.0f*PI;i+=0.1f)
        if(secCrossEllipse(a, b, dirX, dirY, holeR * 0.5f * cosf(i) - x, holeY + 0.5f * holeR * 0.5f + holeR * 0.5f * sinf(i) - y, holeR * 0.5f * cosf(i + 0.1f) - x, holeY + 0.5f * holeR * 0.5f + holeR * 0.5f * sinf(i + 0.1f) - y)) return 0;
    
    if(secCrossEllipse(a, b, dirX, dirY, -holeD3 * 0.5f - x, -edgeY - holeR + holeD3 * 0.5f - y, -holeR * 0.5f - x, -edgeY - holeR * 0.5f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, holeR * 0.5f - x, -edgeY - holeR * 0.5f - y, holeD3 * 0.5f - x, -edgeY - holeR + holeD3 * 0.5f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, holeD3 * 0.5f - x, edgeY + holeR - holeD3 * 0.5f - y, holeR * 0.5f - x, edgeY + holeR * 0.5f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -holeR * 0.5f - x, edgeY + holeR * 0.5f - y, -holeD3 * 0.5f - x, edgeY + holeR - holeD3 * 0.5f - y)) return 0;

    if(secCrossEllipse(a, b, dirX, dirY, -edgeX - holeR + holeD3 * 0.5f - x, -edgeY + holeD2 - holeD3 * 0.5f - y, -holeX - holeD3 * 0.25f - x, -holeY + holeD3 * 0.25f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -holeX + holeD3 * 0.25f - x, -holeY - holeD3 * 0.25f - y, -edgeX + holeD2 - holeD3 * 0.5f - x, -edgeY - holeR + holeD3 * 0.5f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, edgeX - holeD2 + holeD3 * 0.5f - x, -edgeY - holeR + holeD3 * 0.5f - y, holeX - holeD3 * 0.25f - x, -holeY - holeD3 * 0.25f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, holeX + holeD3 * 0.25f - x, -holeY + holeD3 * 0.25f - y, edgeX + holeR - holeD3 * 0.5f - x, -edgeY + holeD2 - holeD3 * 0.5f - y)) return 0;

    if(secCrossEllipse(a, b, dirX, dirY, edgeX + holeR - holeD3 * 0.5f - x, edgeY - holeD2 + holeD3 * 0.5f - y, holeX + holeD3 * 0.25f - x, holeY - holeD3 * 0.25f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, holeX - holeD3 * 0.25f - x, holeY + holeD3 * 0.25f - y, edgeX - holeD2 + holeD3 * 0.5f - x, edgeY + holeR - holeD3 * 0.5f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -edgeX + holeD2 - holeD3 * 0.5f - x, edgeY + holeR - holeD3 * 0.5f - y, -holeX + holeD3 * 0.25f - x, holeY + holeD3 * 0.25f - y)) return 0;
    if(secCrossEllipse(a, b, dirX, dirY, -holeX - holeD3 * 0.25f - x, holeY - holeD3 * 0.25f - y, -edgeX - holeR + holeD3 * 0.5f - x, edgeY - holeD2 + holeD3 * 0.5f - y)) return 0;

    return 1;
}
int validCue(){
    float topX, topY, topZ, dirX, dirY, dirZ, diffX, diffY, diffZ;
    float dis, cosAngle, slope, secX, secY;
    float a, b;
    topX = balls[0].x + balls[0].r * cosf(hitPitch) * cosf(hitYaw + viewYaw);
    topY = balls[0].y + balls[0].r * cosf(hitPitch) * sinf(hitYaw + viewYaw);
    topZ = balls[0].r * sinf(hitPitch);
    dirX = cosf(cuePitch) * cosf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f));
    dirY = cosf(cuePitch) * sinf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f));
    dirZ = sinf(cuePitch);
    for(int i=1;i<=15;i++){
        diffX = balls[i].x - topX;
        diffY = balls[i].y - topY;
        diffZ = -topZ;
        dis = sqrtf(diffX * diffX + diffY * diffY + diffZ * diffZ);
        cosAngle = (dirX * diffX + dirY * diffY + dirZ * diffZ) / dis;
        if(cosAngle > 0.0f && sqrtf(1.0f - cosAngle * cosAngle) * dis < 0.0094f * dis + 5e-3f + balls[0].r) return 0;
    }
    secX = topX + (38e-3f - balls[0].r - topZ) / tanf(cuePitch) * cosf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f));
    secY = topY + (38e-3f - balls[0].r - topZ) / tanf(cuePitch) * sinf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f));
    dis = (38e-3f - balls[0].r - topZ) / sinf(cuePitch); 
    b = 0.0094f * dis + 5e-3f;
    a = b / sinf(cuePitch);
    dirX = cosf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f));
    dirY = sinf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f));
    return validCueWithEdge(secX, secY, a, b, dirX, dirY);
}
void limitCuePitch(){
    float lower, upper;
    cuePitch = setCuePitch;
    hitPitch = setHitPitch;
    if(cuePitch > hitPitch + 0.5f * PI)
        hitPitch = cuePitch - 0.5f * PI;
    if(validCue()) return;
    lower = cuePitch;
    upper = PI * 0.5f;
    for(int i=0;i<10;i++){
        cuePitch = (upper + lower) * 0.5f;
        hitPitch = setHitPitch;
        if(cuePitch > hitPitch + 0.5f * PI)
            hitPitch = cuePitch - 0.5f * PI;
        if(validCue()) upper = cuePitch;
        else lower = cuePitch;
    }
}
void movement(int input){
    clock_gettime(CLOCK_MONOTONIC, &tEnd);
    if((tEnd.tv_sec - tStart.tv_sec) + (tEnd.tv_nsec - tStart.tv_nsec) * 1e-9f > calcTime - 0.005f){
        calcTime += 0.0001f;
    }else if(calcTime > 0.02f){
        calcTime -= 0.0001f;
    }
    while((tEnd.tv_sec - tStart.tv_sec) + (tEnd.tv_nsec - tStart.tv_nsec) * 1e-9f < calcTime){
        clock_gettime(CLOCK_MONOTONIC, &tEnd);
    }
    clock_gettime(CLOCK_MONOTONIC, &tStart);

    if(allowHit == 1){
        allowHit = 2;
        for(int i=0;i<balls.size();i++){
            if(fabsf(balls[i].vx) > epsilon || fabsf(balls[i].vy) > epsilon || fabsf(balls[i].wx) > epsilon || fabsf(balls[i].wy) > epsilon || fabsf(balls[i].wz) > epsilon){
                allowHit = 1;
                break;
            }
        }
        if(allowHit == 2 && balls[0].x < edgeX){
            allowHit = 3;
        }
    }

    if(allowHit == 3){
        limitCuePitch();
    }

    display();
    if(allowHit != 2) collision(calcTime);
    glutTimerFunc(1, movement, 0);
}
void init(){
    int order[15], pos, step, cnt;
    balls.clear();
    for(int i=0;i<=15;i++)
        balls.push_back(ball(edgeX * 0.5f, 0.0f, 0.0f));
    srand(time(NULL));
    pos = 0;
    for(int i=0;i<15;i++)
        order[i] = 0;
    for(int i=0;i<15;i++){
        step = (rand() % (15 - i)) + 1;
        cnt = 0;
        while(cnt < step){
            pos = (pos + 1) % 15;
            if(order[pos] == 0)
                cnt++;
        }
        order[pos] = i + 1;
    }
    if(order[4] != 8)
        for(int i=0;i<15;i++){
            if(order[i] == 8){
                order[i] = order[4];
                order[4] = 8;
                break;
            }
        }
    for(int i=1;i<=5;i++){
        for(int j=0;j<i;j++){
            if(i % 2){
                balls[order[(i - 1) * i / 2 + j]].x = -edgeX * 0.5f - (i - 1) * 1.74f * balls[0].r + ((rand() % 1000) * 1e-7f);
                balls[order[(i - 1) * i / 2 + j]].y =(j - (i - 1) * 0.5f) * balls[0].r * 2.0f + ((rand() % 1000) * 1e-7f);
            }else{
                balls[order[(i - 1) * i / 2 + j]].x = -edgeX * 0.5f - (i - 1) * 1.74f * balls[0].r + ((rand() % 1000) * 1e-7f);
                balls[order[(i - 1) * i / 2 + j]].y = ((j - 0.5f) - (i - 2.0f) * 0.5f) * balls[0].r * 2.0f + ((rand() % 1000) * 1e-7f);
            }
        }
    }
}
void keyBoard(unsigned char key, int x, int y){
    keyState = key;
    if(key == 'f'){
        init();
    }else if(key == 'q'){
        exit(0);
    }else if(key == '0'){
        hitYaw = setHitPitch = 0.0f;
        setCuePitch = PI / 36.0f;
    }else if(key == 'r'){
        if(allowHit == 3) allowHit = 2;
    }else if(key == 'w'){
        mouseState = -1;
        glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
    }
}
void keyUp(unsigned char key, int x, int y){
    keyState = 0;
}
void passiveMotion(int x, int y){
    if(mouseState >= 0){
        if(allowHit != 2){
            if(keyState == ' '){
                setCuePitch += (float)(WINDOW_HEIGHT / 2 - y) / WINDOW_HEIGHT* 10.0f;
                if(setCuePitch > PI * 0.5f) setCuePitch = PI * 0.5f;
                else if(setCuePitch < 1e-2f) setCuePitch = 1e-2f;
            }else{
                viewYaw -= (float)(x - WINDOW_WIDTH / 2) / WINDOW_HEIGHT;
                viewPitch += (float)(y - WINDOW_HEIGHT / 2) / WINDOW_HEIGHT;
                if(viewYaw > 2.0f * PI)
                    viewYaw -= 2.0f * PI;
                else if(viewYaw < -2.0f * PI)
                    viewYaw += 2.0f * PI;
                if(viewPitch < 0.0f)
                    viewPitch = 0.0f;
                else if(viewPitch > 0.5f * PI)
                    viewPitch = 0.5f * PI;
            }
        }else{
            balls[0].x += (float)(x - WINDOW_WIDTH / 2) / WINDOW_HEIGHT * edgeY * 2.0f;
            balls[0].y += -(float)(y - WINDOW_HEIGHT / 2) / WINDOW_HEIGHT * edgeY * 2.0f;
            if(balls[0].x > edgeX - balls[0].r) balls[0].x = edgeX - balls[0].r;
            else if(balls[0].x < -edgeX + balls[0].r) balls[0].x = -edgeX + balls[0].r;
            if(balls[0].y > edgeY - balls[0].r) balls[0].y = edgeY - balls[0].r;
            else if(balls[0].y < -edgeY + balls[0].r) balls[0].y = -edgeY + balls[0].r;
        }
        glutWarpPointer(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
    }
}
void motion(int x, int y){
    float filterRatio;
    if(mouseState == 1){
        if(allowHit == 3){
            filterRatio = expf(-calcTime / 0.03f);
            tmpSpeed = tmpSpeed * filterRatio + (float)(WINDOW_HEIGHT / 2 - y) / WINDOW_HEIGHT * 2.0f / calcTime * (1 - filterRatio);
            hitSpeed = hitSpeed * filterRatio + tmpSpeed * (1 - filterRatio);
            hitDistance += hitSpeed * calcTime;
            if(hitDistance > 0.0f){
                if(hitSpeed > 10.0f) hitSpeed = 10.0f;
                hitBall(viewYaw + hitYaw, hitPitch, -hitSpeed * cosf(cuePitch) * cosf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f)), -hitSpeed * cosf(cuePitch) * sinf(viewYaw - asinf(sinf(hitYaw) * balls[0].r / 0.2f)), -hitSpeed * sinf(cuePitch));
                allowHit = 0;
                tmpSpeed = 0.0f;
                hitSpeed = 0.0f;
                hitDistance = 0.0f;
                for(int i=1;i<=15;i++)
                    if(balls[i].x > edgeX + 0.2f && balls[i].z > -0.1f){
                        balls[i].z = -0.2f;
                    }
            }
        }
    }else if(mouseState == 2){
        if(allowHit != 2){
            viewDistance += (float)(y - WINDOW_HEIGHT / 2) / WINDOW_HEIGHT * 10.0f;
            if(viewDistance > 5.0f) viewDistance = 5.0f;
            else if(viewDistance < 0.5f) viewDistance = 0.5f;
        }
    }else if(mouseState == 3){
        if(allowHit == 3){
            setHitPitch += (float)(WINDOW_HEIGHT / 2 - y) / WINDOW_HEIGHT * 10.0f;
            if(setHitPitch > PI / 4.0f) setHitPitch = PI / 4.0f;
            else if(setHitPitch < -PI / 4.0f) setHitPitch = -PI / 4.0f;
            if(keyState == ' '){
                hitYaw += (float)(x - WINDOW_WIDTH / 2) / WINDOW_HEIGHT * 10.0f;
                if(hitYaw > PI / 4.0f) hitYaw = PI / 4.0f;
                else if(hitYaw < -PI / 4.0f) hitYaw = -PI / 4.0f;
            }
        }
    }
    glutWarpPointer(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
}
void mouse(int button, int state, int x, int y){
    int allowPlace;
    if(state == 0){
        glutSetCursor(GLUT_CURSOR_NONE);
        mouseState = button + 1;
    }else{
        mouseState = 0;
        if(allowHit == 0) allowHit = 1;
        else if(allowHit == 2){
            allowPlace = 1;
            for(int i=1;i<=15;i++){
                if(powf(balls[0].x - balls[i].x, 2) + powf(balls[0].y - balls[i].y, 2) <= powf(2 * balls[0].r, 2)){
                    allowPlace = 0;
                    break;
                }
            }
            if(allowPlace){
                allowHit = 3;
            }
        }
    }
    tmpSpeed = 0.0f;
    hitSpeed = 0.0f;
    hitDistance = 0.0f;
}
void readOBJ(const char dir[], vector<float> &c, vector<float> &n, vector<float> &t, vector<unsigned int> &ic, vector<unsigned int> &in, vector<unsigned int> &it){
    FILE *fp = fopen(dir, "r");
    char tmp[100];
    float x, y, z;
    int t1, t2, t3, t4, t5, t6, t7, t8, t9;
    while(!feof(fp)){
        fgets(tmp, 100, fp);
        if(tmp[0] == 'v' && tmp[1] == ' '){
            sscanf(tmp, "v  %f %f %f", &x, &y, &z);
            c.push_back(x);
            c.push_back(y);
            c.push_back(z);
        }else if(tmp[0] == 'f'){
            sscanf(tmp, "f %d/%d/%d %d/%d/%d %d/%d/%d", &t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9);
            ic.push_back(t1);
            ic.push_back(t4);
            ic.push_back(t7);
            in.push_back(t3);
            in.push_back(t6);
            in.push_back(t9);
            it.push_back(t2);
            it.push_back(t5);
            it.push_back(t8);
        }else if(tmp[0] == 'v' && tmp[1] == 'n'){
            sscanf(tmp, "vn %f %f %f", &x, &y, &z);
            n.push_back(x);
            n.push_back(y);
            n.push_back(z);
        }else if(tmp[0] == 'v' && tmp[1] == 't'){
            sscanf(tmp, "vt %f %f %f", &x, &y, &z);
            t.push_back(x);
            t.push_back(y);
            t.push_back(z);
        }
    }
    fclose(fp);
}
void genVAO(vector<float> &c, vector<float> &n, vector<float> &t, vector<unsigned int> &ic, vector<unsigned int> &in, vector<unsigned int> &it, vector<float> &oc, vector<float> &on, vector<float> &ot, unsigned int *coordBO, unsigned int *normBO, unsigned int *texBO, unsigned int *vao){
    oc = vector<float>(ic.size() * 3, 0);
    on = vector<float>(ic.size() * 3, 0);
    ot = vector<float>(ic.size() * 2, 0);
    for(int i=0;i<ic.size();i++){
        oc[i * 3 + 0] = c[ic[i] * 3 - 3];
        oc[i * 3 + 1] = c[ic[i] * 3 - 2];
        oc[i * 3 + 2] = c[ic[i] * 3 - 1];
        on[i * 3 + 0] = n[in[i] * 3 - 3];
        on[i * 3 + 1] = n[in[i] * 3 - 2];
        on[i * 3 + 2] = n[in[i] * 3 - 1];
        ot[i * 2 + 0] = t[it[i] * 3 - 3];
        ot[i * 2 + 1] = t[it[i] * 3 - 2];
    }
    glGenVertexArrays(1, vao);
    glGenBuffers(1, coordBO);
    glGenBuffers(1, normBO);
    glGenBuffers(1, texBO);
    glBindBuffer(GL_ARRAY_BUFFER, *coordBO);
    glBufferData(GL_ARRAY_BUFFER, oc.size() * sizeof(float), oc.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, *normBO);
    glBufferData(GL_ARRAY_BUFFER, on.size() * sizeof(float), on.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, *texBO);
    glBufferData(GL_ARRAY_BUFFER, ot.size() * sizeof(float), ot.data(), GL_STATIC_DRAW);

    glBindVertexArray(*vao);
    // glClientActiveTexture(GL_TEXTURE0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, *coordBO);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, *normBO);
    glNormalPointer(GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, *texBO);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
void loadTex(unsigned int *tex, const char dir[]){
    int width, height, nChannel;
    unsigned char *data;
    data = stbi_load(dir, &width, &height, &nChannel, 0);
    glBindTexture(GL_TEXTURE_2D, *tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    stbi_image_free(data);
}
void genShadowTex(unsigned int *shadowTex){
    glBindTexture(GL_TEXTURE_2D, *shadowTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, shadowWidth, shadowHeight, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
}
void reshape(int width, int height){
    WINDOW_WIDTH = width;
    shadowWidth = width;
    WINDOW_HEIGHT = height;
    shadowHeight = height;
    genShadowTex(&shadowTexture);
}
int main(int argc, char *argv[]){
    char dir[100];
    init();
    readOBJ("../mat/plane.obj", planeCoord, planeNorm, planeTex, planeCoordIndex, planeNormIndex, planeTexIndex);
    readOBJ("../mat/ball.obj", ballCoord, ballNorm, ballTex, ballCoordIndex, ballNormIndex, ballTexIndex);
    readOBJ("../mat/cue.obj", cueCoord, cueNorm, cueTex, cueCoordIndex, cueNormIndex, cueTexIndex);
    glutInit(&argc, argv);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("pool");
    glewInit();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyBoard);
    glutKeyboardUpFunc(keyUp);
    glutMouseFunc(mouse);
    glutPassiveMotionFunc(passiveMotion);
    glutMotionFunc(motion);
    glutSetCursor(GLUT_CURSOR_NONE);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);

    genVAO(planeCoord, planeNorm, planeTex, planeCoordIndex, planeNormIndex, planeTexIndex, planeOrderedCoord, planeOrderedNorm, planeOrderedTex, &planeCoordBO, &planeNormBO, &planeTexBO, &planeVAO);
    genVAO(ballCoord, ballNorm, ballTex, ballCoordIndex, ballNormIndex, ballTexIndex, ballOrderedCoord, ballOrderedNorm, ballOrderedTex, &ballCoordBO, &ballNormBO, &ballTexBO, &ballVAO);
    genVAO(cueCoord, cueNorm, cueTex, cueCoordIndex, cueNormIndex, cueTexIndex, cueOrderedCoord, cueOrderedNorm, cueOrderedTex, &cueCoordBO, &cueNormBO, &cueTexBO, &cueVAO);

    glGenTextures(1, &shadowTexture);
    genShadowTex(&shadowTexture);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0f, 2.0f, 0.1f, 50.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f);
    glGetFloatv(GL_PROJECTION_MATRIX, lightPM);
    glGetFloatv(GL_MODELVIEW_MATRIX, lightVM);

    glGenTextures(16, texture);
    for(int i=0;i<=15;i++){
        sprintf(dir, "../mat/ball%d.jpg", i);
        loadTex(texture + i, dir);
    }

    clock_gettime(CLOCK_MONOTONIC, &tStart);
    glutTimerFunc(20, movement, 0);
    glutMainLoop();
    return 0;
}