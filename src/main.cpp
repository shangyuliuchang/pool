#define STB_IMAGE_IMPLEMENTATION
#include"../inc/stb_image.h"
#include"stdio.h"
#include"stdlib.h"
#include<GL/glut.h>
#include<vector>
#include<math.h>
#include"time.h"
using namespace std;
#include"../inc/ball.hpp"
#include"../inc/main.hpp"
#include"../inc/calc.hpp"
int win, allowHit, planeBAO, planeVBO, planeEBO;
float viewYaw = 0.0f, viewPitch = 0.0f, viewDistance = 0.6f, hitDistance = 0.0f, hitSpeed = 0.0f;
float viewX = 0.0f, viewY = 0.0f, viewZ = 0.0f;
float rotateAmount = 0.0;
float calcTime = 0.02f;
unsigned int texture[16];
vector<float> coords, norm, tex, ballX, ballY, ballZ, ballNX, ballNY, ballNZ, ballCoord, ballNorm, ballTex;
vector<unsigned int> index, normIndex, texIndex, ballCoordIndex, ballNormIndex, ballTexIndex;
GLfloat color[4];
struct timespec tStart;
struct timespec tEnd;
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
void drawTriangle(vector<unsigned int> &ic, vector<unsigned int> &in, vector<unsigned int> &it, vector<float> &c, vector<float> &n, vector<float> &t){
    for(int i=0;i<ic.size()/3;i++){
        glTexCoord2f(t[it[i * 3 + 0] * 3 - 3], t[it[i * 3 + 0] * 3 - 2]);
        glNormal3f(n[in[i * 3 + 0] * 3 - 3], n[in[i * 3 + 0] * 3 - 2], n[in[i * 3 + 0] * 3 - 1]);
        glVertex3f(c[ic[i * 3 + 0] * 3 - 3], c[ic[i * 3 + 0] * 3 - 2], c[ic[i * 3 + 0] * 3 - 1]);

        glTexCoord2f(t[it[i * 3 + 1] * 3 - 3], t[it[i * 3 + 1] * 3 - 2]);
        glNormal3f(n[in[i * 3 + 1] * 3 - 3], n[in[i * 3 + 1] * 3 - 2], n[in[i * 3 + 1] * 3 - 1]);
        glVertex3f(c[ic[i * 3 + 1] * 3 - 3], c[ic[i * 3 + 1] * 3 - 2], c[ic[i * 3 + 1] * 3 - 1]);

        glTexCoord2f(t[it[i * 3 + 2] * 3 - 3], t[it[i * 3 + 2] * 3 - 2]);
        glNormal3f(n[in[i * 3 + 2] * 3 - 3], n[in[i * 3 + 2] * 3 - 2], n[in[i * 3 + 2] * 3 - 1]);
        glVertex3f(c[ic[i * 3 + 2] * 3 - 3], c[ic[i * 3 + 2] * 3 - 2], c[ic[i * 3 + 2] * 3 - 1]);
    }
}
void drawPlane(){
    glLoadIdentity();
    glTranslatef(0.0f, -balls[0].r, 0.0f);
    glColor4f(0.2f, 0.4f, 0.2f, 1.0f);
    glBegin(GL_TRIANGLES);
    drawTriangle(index, normIndex, texIndex, coords, norm, tex);
    glEnd();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(0.0f, 0.0f, 0.0f, 0.0f);
    glLoadIdentity();
    glTranslatef(0.0f, -balls[0].r + 0.0003f, 0.0f);
    glBegin(GL_POLYGON);
    for(float i=0;i<2*PI;i+=0.1f)
        glVertex3f(holeX + holeR * 0.5f * cosf(i), 0.0f, -(holeY + holeR * 0.5f * sinf(i)));
    glEnd();
    glBegin(GL_POLYGON);
    for(float i=0;i<2*PI;i+=0.1f)
        glVertex3f(-holeX + holeR * 0.5f * cosf(i), 0.0f, -(holeY + holeR * 0.5f * sinf(i)));
    glEnd();
    glBegin(GL_POLYGON);
    for(float i=0;i<2*PI;i+=0.1f)
        glVertex3f(holeX + holeR * 0.5f * cosf(i), 0.0f, -(-holeY + holeR * 0.5f * sinf(i)));
    glEnd();
    glBegin(GL_POLYGON);
    for(float i=0;i<2*PI;i+=0.1f)
        glVertex3f(-holeX + holeR * 0.5f * cosf(i), 0.0f, -(-holeY + holeR * 0.5f * sinf(i)));
    glEnd();
    glBegin(GL_POLYGON);
    for(float i=0;i<2*PI;i+=0.1f)
        glVertex3f(holeR * 0.5f * cosf(i), 0.0f, -(edgeY + holeR * 0.5f + holeR * 0.5f * sinf(i)));
    glEnd();
    glBegin(GL_POLYGON);
    for(float i=0;i<2*PI;i+=0.1f)
        glVertex3f(holeR * 0.5f * cosf(i), 0.0f, -(-edgeY - holeR * 0.5f + holeR * 0.5f * sinf(i)));
    glEnd();


    glColor4f(0.0f, 0.0f, 0.0f, 0.8f);
    glLoadIdentity();
    glTranslatef(0.0f, -balls[0].r + 0.0001f, 0.0f);
    glBegin(GL_POLYGON);
    for(float i=PI;i>0.5f*PI;i-=0.1f)
        glVertex3f(edgeX + holeR + holeR * cosf(i), 0.0f, -(edgeY - holeD2 + holeR * sinf(i)));
    for(float i=1.5f*PI;i>PI;i-=0.1f)
        glVertex3f(edgeX + holeR + holeR * cosf(i), 0.0f, -(-edgeY + holeD2 + holeR * sinf(i)));
    glEnd();

    glBegin(GL_POLYGON);
    for(float i=PI;i>0.5f*PI;i-=0.1f)
        glVertex3f(-1.0f * (edgeX + holeR + holeR * cosf(i)), 0.0f, (edgeY - holeD2 + holeR * sinf(i)));
    for(float i=1.5f*PI;i>PI;i-=0.1f)
        glVertex3f(-1.0f * (edgeX + holeR + holeR * cosf(i)), 0.0f, (-edgeY + holeD2 + holeR * sinf(i)));
    glEnd();

    glBegin(GL_POLYGON);
    for(float i=PI;i<1.5f*PI;i+=0.1f)
        glVertex3f(holeD3 + holeR * cosf(i), 0.0f, -(edgeY + holeR + holeR * sinf(i)));
    for(float i=-0.5f*PI;i<0.0f;i+=0.1f)
        glVertex3f(edgeX - holeD2 + holeR * cosf(i), 0.0f, -(edgeY + holeR + holeR * sinf(i)));
    glEnd();

    glBegin(GL_POLYGON);
    for(float i=PI;i<1.5f*PI;i+=0.1f)
        glVertex3f(-1.0f * (holeD3 + holeR * cosf(i)), 0.0f, (edgeY + holeR + holeR * sinf(i)));
    for(float i=-0.5f*PI;i<0.0f;i+=0.1f)
        glVertex3f(-1.0f * (edgeX - holeD2 + holeR * cosf(i)), 0.0f, (edgeY + holeR + holeR * sinf(i)));
    glEnd();

    glBegin(GL_POLYGON);
    for(float i=PI;i<1.5f*PI;i+=0.1f)
        glVertex3f(-edgeX + holeD2 + holeR * cosf(i), 0.0f, -(edgeY + holeR + holeR * sinf(i)));
    for(float i=-0.5f*PI;i<0.0f;i+=0.1f)
        glVertex3f(-holeD3 + holeR * cosf(i), 0.0f, -(edgeY + holeR + holeR * sinf(i)));
    glEnd();

    glBegin(GL_POLYGON);
    for(float i=PI;i<1.5f*PI;i+=0.1f)
        glVertex3f(-(-edgeX + holeD2 + holeR * cosf(i)), 0.0f, (edgeY + holeR + holeR * sinf(i)));
    for(float i=-0.5f*PI;i<0.0f;i+=0.1f)
        glVertex3f(-(-holeD3 + holeR * cosf(i)), 0.0f, (edgeY + holeR + holeR * sinf(i)));
    glEnd();

    glDisable(GL_BLEND);
}
void display(){
    float vecX[3], vecY[3], angleX, angleY;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0f, 4.0f / 3.0f, 0.1f, 50.0f);
    viewX = viewX * 0.95f + balls[0].x * 0.05f;
    viewY = viewY * 0.95f + balls[0].y * 0.05f;
    viewZ = viewZ * 0.95f + balls[0].z * 0.05f;
    gluLookAt(viewX + viewDistance * cosf(viewPitch) * cos(viewYaw), viewZ + viewDistance * sinf(viewPitch), -viewY - viewDistance * cosf(viewPitch) * sinf(viewYaw), viewX, viewZ, -viewY, 0.0f, 1.0f, 0.0f);
    glViewport(0, -WINDOW_HEIGHT / 2, WINDOW_WIDTH, 1.5f * WINDOW_HEIGHT);
    glMatrixMode(GL_MODELVIEW);

    drawPlane();

    glEnable(GL_TEXTURE_2D);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    for(int i=0;i<=15;i++){
        glLoadIdentity();
        glTranslatef(balls[i].x, balls[i].z, -balls[i].y);
        getRotation(balls[i].quatX, balls[i].quatY, &angleX, vecX, &angleY, vecY);
        glRotatef(angleX * 180.0f / PI, vecX[0], vecX[2], -vecX[1]);
        if(balls[i].quatX[1] * vecY[0] + balls[i].quatX[2] * vecY[1] + balls[i].quatX[3] * vecY[2] > 0.0f)
            glRotatef(angleY * 180.0f / PI, 1.0f, 0.0f, 0.0f);
        else
            glRotatef(angleY * 180.0f / PI, -1.0f, 0.0f, 0.0f);

        glBindTexture(GL_TEXTURE_2D, texture[i]);
        glBegin(GL_TRIANGLES);
        drawTriangle(ballCoordIndex, ballNormIndex, ballTexIndex, ballCoord, ballNorm, ballTex);
        glEnd();
    }
    glDisable(GL_TEXTURE_2D);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0f, 0.0f, 0.0f, 0.8f);
    for(int i=0;i<=15;i++){
        glLoadIdentity();
        glTranslatef(balls[i].x, balls[i].z, -balls[i].y);
        glBegin(GL_POLYGON);
        for(float i=0;i<2*PI;i+=0.1f){
            glVertex3f(balls[0].r * cosf(i), 0.0002f - balls[0].r, balls[0].r * sinf(i));
        }
        glEnd();
    }
    glDisable(GL_BLEND);
    glutSwapBuffers();
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
    display();
    collision(calcTime);
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
    if(key == 'f'){
        init();
    }else if(key == 'q'){
        exit(0);
    }else if(key <= '9' && key >='1'){
        rotateAmount = ((int)key - '5') * 0.05f;
    }
}
void passiveMotion(int x, int y){
    viewYaw -= (x - WINDOW_WIDTH / 2) * 1e-3f;
    viewPitch += (y - WINDOW_HEIGHT / 2) * 1e-3f;
    if(viewYaw > 2.0f * PI)
        viewYaw -= 2.0f * PI;
    else if(viewYaw < -2.0f * PI)
        viewYaw += 2.0f * PI;
    if(viewPitch < 0.0f)
        viewPitch = 0.0f;
    else if(viewPitch > 0.5f * PI)
        viewPitch = 0.5f * PI;
    allowHit = 1;
    hitDistance = 0.0f;
    hitSpeed = 0.0f;
    glutWarpPointer(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
}
void motion(int x, int y){
    if(allowHit){
        hitDistance += (WINDOW_HEIGHT / 2 - y) * 0.15f;
        hitSpeed = hitSpeed * 0.8f + (WINDOW_HEIGHT / 2 - y) * 0.15f * 0.2f;
        if(hitDistance > 0.0f){
            if(hitSpeed > 10.0f) hitSpeed = 10.0f;
            balls[0].vx = -hitSpeed * cosf(viewYaw);
            balls[0].vy = -hitSpeed * sinf(viewYaw);
            balls[0].wx = rotateAmount * balls[0].r * hitSpeed * balls[0].m / balls[0].I * sinf(viewYaw);
            balls[0].wy = rotateAmount * balls[0].r * hitSpeed * balls[0].m / balls[0].I * -cosf(viewYaw);
            balls[0].wz = 0.0f;
            allowHit = 0;
        }
    }
    glutWarpPointer(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
}
void mouse(int button, int state, int x, int y){
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
int main(int argc, char *argv[]){
    GLfloat lightPos[] = {0.0f, 1.0f, 0.0f, 1.0f};
    GLfloat lightDiffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat lightAmbient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat lightSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    char dir[100];
    init();
    readOBJ("D:/projects/vccode/billard/mat/plane.obj", coords, norm, tex, index, normIndex, texIndex);
    readOBJ("D:/projects/vccode/billard/mat/ball.obj", ballCoord, ballNorm, ballTex, ballCoordIndex, ballNormIndex, ballTexIndex);
    glutInit(&argc, argv);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    win = glutCreateWindow("a");
    glutDisplayFunc(display);
    glutKeyboardFunc(keyBoard);
    glutMouseFunc(mouse);
    glutPassiveMotionFunc(passiveMotion);
    glutMotionFunc(motion);
    glutSetCursor(GLUT_CURSOR_NONE);

    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glGenTextures(16, texture);
    for(int i=0;i<=15;i++){
        sprintf(dir, "D:/projects/vccode/billard/mat/ball%d.jpg", i);
        loadTex(texture + i, dir);
    }

    clock_gettime(CLOCK_MONOTONIC, &tStart);
    glutTimerFunc(20, movement, 0);
    glutMainLoop();
    return 0;
}