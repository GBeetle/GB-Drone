#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_heap_caps.h>
#include <TGL/gl.h>
#include "zbuffer.h"

#define STBIW_ASSERT(x)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "3dmath.h"
#include "tobjparse.h"
#include "log_sys.h"
#include "disp_driver.h"
#include "gb_timer.h"
#include "quad_3d.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif

#if (CONFIG_DISPLAY_ORIENTATION_PORTRAIT)
#define winSizeX RENDER_3D_HEIGHT
#define winSizeY RENDER_3D_WIDTH
#else
#define winSizeX RENDER_3D_HEIGHT
#define winSizeY RENDER_3D_WIDTH
#endif

vec3 campos = (vec3){.d[0] = 6, .d[1] = 6, .d[2] = 6};     // camera position
vec3 camforw = (vec3){.d[0] = -1, .d[1] = -1, .d[2] = -1}; // camera forward direction
vec3 camup = (vec3){.d[0] = 0, .d[1] = 1, .d[2] = 0};      // camera up direction
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

static ZBuffer *frameBuffer = NULL;
static GLuint modelDisplayList = 0;
static mat4 projection_matrix;
static mat4 view_matrix;
static bool init_fail = false;

SemaphoreHandle_t angleProtected;

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

bool quad3d_init()
{
    if (init_fail)
        return false;
    GB_DEBUGE(DISP_TAG, "quad3d_init start");

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
        return false;
    }
    glInit(frameBuffer);

    // Try to place Z-buffer in faster internal SRAM for better random-access performance
    GLint zbuf_size = winSizeX * winSizeY * sizeof(GLushort);
    void *fast_zbuf = heap_caps_malloc(zbuf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (fast_zbuf)
    {
        memcpy(fast_zbuf, frameBuffer->zbuf, zbuf_size);
        gl_free(frameBuffer->zbuf);
        frameBuffer->zbuf = fast_zbuf;
        GB_DEBUGI(DISP_TAG, "Z-buffer allocated in internal SRAM (%ld bytes)", (long)zbuf_size);
    }
    else
    {
        GB_DEBUGI(DISP_TAG, "Internal SRAM not enough for zbuf, keeping PSRAM");
    }

    if (frameBuffer->frame_buffer_allocated && frameBuffer->pbuf)
    {
        gl_free(frameBuffer->pbuf);
        frameBuffer->pbuf = NULL;
        frameBuffer->frame_buffer_allocated = 0;
    }

    glShadeModel(GL_FLAT);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
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
    glShadeModel(GL_FLAT);
    // glDisable(GL_DEPTH_TEST);
    glViewport(0, 0, winSizeX, winSizeY);
    // glEnable(GL_POLYGON_STIPPLE);

    {
        objraw omodel;
        model m = initmodel();
        omodel = tobj_load("/sdcard/quad.obj");

        if (!omodel.positions)
        {
            GB_DEBUGE(DISP_TAG, "\nERROR! No positions in model. Aborting...");
            goto error_exit;
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

    projection_matrix = perspective(70, (float)winSizeX / (float)winSizeY, 0.1, 100.0);
    view_matrix = lookAt(campos, addv3(campos, camforw), camup);

    angleProtected = xSemaphoreCreateMutex();

    GB_DEBUGE(DISP_TAG, "quad3d_init end");
    return true;

error_exit:
    if (frameBuffer)
    {
        free(frameBuffer);
        frameBuffer = NULL;
    }
    init_fail = true;
    return false;
}

bool quad3d_get_image(uint16_t *image_buffer)
{
    if (NULL == frameBuffer)
    {
        if (!quad3d_init())
            return false;
    }

    frameBuffer->pbuf = (PIXEL *)image_buffer;

    memset(image_buffer, 0xFF, winSizeX * winSizeY * sizeof(uint16_t));

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glLoadMatrixf(projection_matrix.d);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix(); // Pushing on the LookAt Matrix.

    glLoadMatrixf(view_matrix.d);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if (xSemaphoreTake(angleProtected, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        glPushMatrix();
        glRotatef(pitch, 1.0f, 0.0f, 0.0f);
        glRotatef(yaw, 0.0f, 1.0f, 0.0f);
        glRotatef(roll, 0.0f, 0.0f, 1.0f);
        glCallList(modelDisplayList);
        glPopMatrix();

        xSemaphoreGive(angleProtected);
    }

    glPopMatrix(); // The view transform.

    return true;
}

bool quad3d_set_angle(float s_roll, float s_pitch, float s_yaw)
{
    if (NULL == angleProtected)
    {
        return false;
    }

    if (xSemaphoreTake(angleProtected, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        roll = s_roll;
        pitch = s_pitch;
        yaw = s_yaw;

        xSemaphoreGive(angleProtected);
        return true;
    }
    return false;
}

static uint16_t *render_buffers[2] = {NULL, NULL};
static volatile int front_buf = 0;
static volatile bool frame_ready = false;
static TaskHandle_t render_task_handle = NULL;

static void render_3d_task(void *arg)
{
    (void)arg;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int back = 1 - front_buf;
        if (render_buffers[back] && quad3d_get_image(render_buffers[back]))
        {
            front_buf = back;
            frame_ready = true;
        }
    }
}

bool quad3d_start_render_task(uint16_t *buf_a, uint16_t *buf_b)
{
    if (render_task_handle != NULL)
        return false;

    render_buffers[0] = buf_a;
    render_buffers[1] = buf_b;
    front_buf = 0;
    frame_ready = false;

    BaseType_t ret = xTaskCreatePinnedToCore(render_3d_task, "3d_render",
        32768, NULL, 5 | portPRIVILEGE_BIT, &render_task_handle, 1);
    if (ret != pdPASS)
    {
        GB_DEBUGE(DISP_TAG, "Failed to create 3D render task");
        render_task_handle = NULL;
        return false;
    }

    // Trigger first render
    xTaskNotifyGive(render_task_handle);
    GB_DEBUGE(DISP_TAG, "3D render task started on core 1");
    return true;
}

void quad3d_stop_render_task(void)
{
    if (render_task_handle)
    {
        vTaskDelete(render_task_handle);
        render_task_handle = NULL;
    }

    render_buffers[0] = NULL;
    render_buffers[1] = NULL;
    frame_ready = false;
}

bool quad3d_is_frame_ready(void)
{
    return frame_ready;
}

uint16_t *quad3d_get_front_buffer(void)
{
    frame_ready = false;
    // Trigger next render
    if (render_task_handle)
    {
        xTaskNotifyGive(render_task_handle);
    }
    return render_buffers[front_buf];
}
