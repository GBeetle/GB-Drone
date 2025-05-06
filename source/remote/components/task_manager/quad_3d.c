#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <TGL/gl.h>
#include "zbuffer.h"

#define STBIW_ASSERT(x)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "3dmath.h"
#include "tobjparse.h"
#include "log_sys.h"
#include "disp_driver.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define winSizeX LV_VER_RES_MAX
#define winSizeY LV_HOR_RES_MAX

vec3 campos = (vec3){.d[0] = 8, .d[1] = 8, .d[2] = 8};     // camera position
vec3 camforw = (vec3){.d[0] = -1, .d[1] = -1, .d[2] = -1}; // camera forward direction
vec3 camup = (vec3){.d[0] = 0, .d[1] = 1, .d[2] = 0};      // camera up direction
uint wasdstate[4] = {0, 0, 0, 0};
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

PIXEL *imbuf = NULL;
ZBuffer *frameBuffer = NULL;
GLuint modelDisplayList = 0;

SemaphoreHandle_t angleProtected;

GLubyte stipplepattern[128] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA,
    0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55,
    0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55,

    0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA,
    0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55,
    0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55,

    0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA,
    0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55,
    0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55,

    0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA,
    0xAA, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55,
    0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55};

GLuint createModelDisplayList(
    // HUGE important note! these depend on the math library using
    // f_ as float and not double!
    // Remember that!
    vec3 *points,
    uint npoints,
    vec3 *colors,
    vec3 *normals,
    vec3 *texcoords)
{
    GLuint ret = 0;
    if (!points)
        return 0;
    ret = glGenLists(1);
    glNewList(ret, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for (uint i = 0; i < npoints; i++)
    {
        if (colors)
        {
            glColor3f(colors[i].d[0], colors[i].d[1], colors[i].d[2]);
        }
        if (texcoords)
            glTexCoord2f(texcoords[i].d[0], texcoords[i].d[1]);
        if (normals)
            glNormal3f(normals[i].d[0], normals[i].d[1], normals[i].d[2]);
        glVertex3f(points[i].d[0], points[i].d[1], points[i].d[2]);
    }
    // printf("\ncreateModelDisplayList is not the problem.\n");
    glEnd();
    glEndList();
    return ret;
}

void quad3d_init()
{
    imbuf = calloc(1, sizeof(PIXEL) * winSizeX * winSizeY);

    // initialize TinyGL
    frameBuffer = ZB_open(winSizeX, winSizeY,
#if TGL_FEATURE_RENDER_BITS == 32
                          ZB_MODE_RGBA,
#else
                          ZB_MODE_5R6G5B,
#endif
                          0);
    if (!frameBuffer)
    {
        GB_DEBUGE(DISP_TAG, "\nZB_open failed!");
        return;
    }
    glInit(frameBuffer);

    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glSetEnableSpecular(0);
    static GLfloat white[4] = {1.0, 1.0, 1.0, 0.0};
    static GLfloat pos[4] = {5, 5, 10, 0.0}; // Light at infinity.

    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    // glLightfv( GL_LIGHT0, GL_AMBIENT, white);
    // glLightfv( GL_LIGHT0, GL_SPECULAR, white);
    glEnable(GL_CULL_FACE);

    // glDisable( GL_LIGHTING );
    glEnable(GL_LIGHT0);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glClearColor(0, 0, 0, 0);
    glClearDepth(1.0f);

    glClearColor(1, 1, 1, 1);
    glDisable(GL_TEXTURE_2D);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    // glDisable(GL_LIGHTING);
    glShadeModel(GL_SMOOTH);
    // glDisable(GL_DEPTH_TEST);
    glViewport(0, 0, winSizeX, winSizeY);
    // glEnable(GL_POLYGON_STIPPLE);
    glPolygonStipple(stipplepattern);

    {
        objraw omodel;
        model m = initmodel();
        omodel = tobj_load("/storage/quad.obj");

        if (!omodel.positions)
        {
            GB_DEBUGE(DISP_TAG, "\nERROR! No positions in model. Aborting...");
            return;
        }
        else
        {
            m = tobj_tomodel(&omodel);
            GB_DEBUGI(DISP_TAG, "\nHas %ld points.\n", m.npoints);
            modelDisplayList =
                createModelDisplayList(m.d, m.npoints, m.c, m.n, m.t);
            freemodel(&m);
        }
        freeobjraw(&omodel);
    }

    angleProtected = xSemaphoreCreateMutex();
}

void quad3d_get_image(uint16_t *image_buffer)
{
    if (NULL == imbuf || NULL == frameBuffer)
        quad3d_init();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    mat4 matrix = perspective(70, (float)winSizeX / (float)winSizeY, 0.1, 100.0);
    glLoadMatrixf(matrix.d);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix(); // Pushing on the LookAt Matrix.

    vec3 right = normalizev3(crossv3(camforw, camup));
    matrix = (lookAt(campos, addv3(campos, camforw), camup));
    glLoadMatrixf(matrix.d);
    if (wasdstate[0])
        campos = addv3(campos, scalev3(0.1, camforw));
    if (wasdstate[2])
        campos = addv3(campos, scalev3(-0.1, camforw));
    if (wasdstate[1])
        campos = addv3(campos, scalev3(-0.1, right));
    if (wasdstate[3])
        campos = addv3(campos, scalev3(0.1, right));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    {
        if (xSemaphoreTake(angleProtected, portMAX_DELAY) == pdTRUE)
        {
            // glDisable(GL_TEXTURE_2D);
            // glDisable(GL_COLOR_MATERIAL);

            glPushMatrix();
            mat4 total_translation = translate((vec3){{0.0, 0.0, 0.0}});
            glMultMatrixf(total_translation.d);
            glRotatef(pitch, 1.0f, 0.0f, 0.0f);
            glRotatef(yaw, 0.0f, 1.0f, 0.0f);
            glRotatef(roll, 0.0f, 0.0f, 1.0f);
            // glMultMatrixf(total_rotation.d);
            glCallList(modelDisplayList);
            glPopMatrix();

            xSemaphoreGive(angleProtected);
        }
    }
    // draw();
    glPopMatrix(); // The view transform.

    // rotateCamera();

    // Quickly convert all pixels to the correct format
#if TGL_FEATURE_RENDER_BITS == 32
    if (needsRGBAFix)
        for (int i = 0; i < frameBuffer->xsize * frameBuffer->ysize; i++)
        {
#define DATONE (frameBuffer->pbuf[i])
            DATONE =
                ((DATONE & 0x000000FF)) << screen->format->Rshift |
                ((DATONE & 0x0000FF00) >> 8) << screen->format->Gshift |
                ((DATONE & 0x00FF0000) >> 16) << screen->format->Bshift;
        }
#endif
    ZB_copyFrameBuffer(frameBuffer, imbuf, winSizeX * sizeof(PIXEL));

    for (int i = 0; i < winSizeX * winSizeY; i++)
    {
        uint8_t red = GET_RED(imbuf[i]);
        uint8_t green = GET_GREEN(imbuf[i]);
        uint8_t blue = GET_BLUE(imbuf[i]);

        image_buffer[i] = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
        image_buffer[i] = image_buffer[i] << 8 | image_buffer[i] >> 8;
    }
}

void quad3d_set_angle(float s_roll, float s_pitch, float s_yaw)
{
    if (NULL == imbuf || NULL == frameBuffer)
        quad3d_init();

    if (xSemaphoreTake(angleProtected, portMAX_DELAY) == pdTRUE)
    {
        roll = s_roll;
        pitch = s_pitch;
        yaw = s_yaw;

        xSemaphoreGive(angleProtected);
    }
}
