#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <GL/glfw.h>

#include <Eigen/Core>

#include "mesh/mesh.h"

#include "terrain.h"

using namespace std;
using namespace Eigen;
using namespace Mesh;

namespace App {

bool running = true;

GLuint scene_display_list = 0;
double vw=1., vx=0., vy=0., vz=0.;
double tx = 0., ty=0., tz=100.;
double fov=45., znear=1., zfar=1000.;
int mx = 0, my = 0, mz = 0;

void init() {

    scene_display_list = glGenLists(1);
    glNewList(scene_display_list, GL_COMPILE);
  
    

	TriMesh<TerrainVertex> base_mesh;
	TerrainGenerator terrain_func;
	TerrainAttribute terrain_attr(terrain_func);
	
	Mesh::isocontour(
		base_mesh,
		terrain_func,
		terrain_attr,
		Vector3f(0, 0, 0),
		Vector3f( 256, 128, 256),
		Vector3i(256, 128, 256) );
	
	TerrainVertex *vbuffer;
	int *ibuffer;
	int vcount, icount;

	base_mesh.get_buffers(
		&vbuffer,
		&vcount,
		&ibuffer,
		&icount);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glBegin(GL_TRIANGLES);
	for(int i=0; i<icount; ++i) {
		auto v = vbuffer[ibuffer[i]];
		glColor3f(v.color[0], v.color[1], v.color[2]);
		glNormal3f(v.normal[0], v.normal[1], v.normal[2]);
		glVertex3f(v.position[0], v.position[1], v.position[2]);
	}
	glEnd();
	
    glEndList();
}

void input() {

    if( glfwGetKey(GLFW_KEY_ESC) == GLFW_PRESS || 
        !glfwGetWindowParam(GLFW_OPENED) ) {
        running = false;
    }
    
    int w, h;
    glfwGetWindowSize(&w, &h);
    
    //Update view
    int px = mx, py = my, pz = mz;
    glfwGetMousePos(&mx, &my);
    mx -= w/2;
    my -= h/2;
    mz = glfwGetMouseWheel();
    
    if(glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT)) {
        double  nx = w * (my - py),
                ny = w * (px - mx),
                nz = mx * py - my * px,
                nw = mx * px + my * py + w * w;
                
        double  qw = vw * nw - vx * nx - vy * ny - vz * nz,
                qx = vw * nx + vx * nw + vy * nz - vz * ny,
                qy = vw * ny - vx * nz + vy * nw + vz * nx,
                qz = vw * nz + vx * ny - vy * nx + vz * nw;
                
        double l = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        
        if(l < 0.0001) {
            vx = vy = vz = 0.;
            vw = 1.;
        }
        else {
            vw = qw / l;
            vx = qx / l;
            vy = qy / l;
            vz = qz / l;
        }
    }
    
    tz += (pz - mz) * 10;
}

void draw() {

    int w, h;
    glfwGetWindowSize(&w, &h);
    glViewport(0, 0, w, h);

    glClearColor(0.3, 0.3, 0.8, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (float)w / (float)h, znear, zfar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslated(-tx, -ty, -tz);

    double m = sqrt(1. - vw*vw);
    if(m > 0.0001) {
        if(abs(vw) > 0.0001) {    
            glRotated(-acos(vw) * 360. / M_PI, vx / m, vy / m, vz / m);
        }
        else {
            glRotated(180., vx, vy, vz);
        }
    }
    
    glCallList(scene_display_list);
}

};

int main(int argc, char* argv[]) {
    glfwInit();
    if (!glfwOpenWindow(1280, 1024, 8, 8, 8, 8, 16, 0, GLFW_WINDOW)) {
        glfwTerminate();
        return -1;
    }
    glfwSetWindowTitle("Mesh Demo");
    
    App::init();
    while(App::running) {
        App::input();
        App::draw();
        glfwSwapBuffers();
    }

    glfwTerminate();
    return 0;
}

