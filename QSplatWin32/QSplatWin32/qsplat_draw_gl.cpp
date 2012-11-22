/*
Szymon Rusinkiewicz

qsplat_draw_gl.cpp
Draw splats using OpenGL points or quads

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_guimain.h"
#include <Windows.h>
#include <GL/gl.h>


// Local variables
static enum { GL_BEGIN_NONE, GL_BEGIN_POINTS,
	      GL_BEGIN_QUADS, GL_BEGIN_TRIANGLES } currentGLstate;
static int currentGLpointsize = -1;
static int point_size_thresh;
static bool circ;
static vec lr, ur;
static vec rdir, uldir, lldir;
static const float sqrt5 = 2.236068f;


// Set up texture map...
static void setuptexturing()
{
	static unsigned char *thetexture;
	static unsigned texcontext = 0;

	if (!glIsTexture(texcontext)) {
		glGenTextures(1, &texcontext);
		glBindTexture(GL_TEXTURE_2D, texcontext);
		const int texsize = 64;
		thetexture = new unsigned char[sqr(texsize) * 2];
		for (int x=0; x < texsize; x++) {
			for (int y=0; y < texsize; y++) {
				int index = x*texsize + y;
				float r2 = sqr(0.5f+x-texsize/2)+sqr(0.5f+y-texsize/2);
				r2 /= sqr(texsize/2);
				if (r2 < 0.2f) {
					thetexture[2*index  ] = 255;
					thetexture[2*index+1] = 255;
				} else {
					thetexture[2*index  ] = 0;
					thetexture[2*index+1] = 0;
				}
			}
		}
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA,
                             texsize, texsize, 0,
                             GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE,
                             thetexture);
		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	}
	glBindTexture(GL_TEXTURE_2D, texcontext);
}



// Little function called right before we start drawing
void start_drawing_gl(bool havecolor, int _point_size_thresh, bool _circ)
{
	circ = _circ;
	point_size_thresh = _point_size_thresh;

	currentGLstate = GL_BEGIN_NONE;
	currentGLpointsize = -1;

	float M[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, M);

	rdir[0] = M[0]; rdir[1]  = M[4]; rdir[2]  = M[8];
	uldir[0] = M[1] - M[0]; uldir[1] = M[5] - M[4]; uldir[2] = M[9] - M[8];
	lldir[0] = -M[1] - M[0]; lldir[1] = -M[5] - M[4]; lldir[2] = -M[9] - M[8];
	rdir[0] *= sqrt5;  rdir[1] *= sqrt5;  rdir[2] *= sqrt5;
	uldir[0] *= sqrt5; uldir[1] *= sqrt5; uldir[2] *= sqrt5;
	lldir[0] *= sqrt5; lldir[1] *= sqrt5; lldir[2] *= sqrt5;

	ur[0] =  M[0] + M[1];  ur[1] =  M[4] + M[5];  ur[2] =  M[8] + M[9];
	lr[0] =  M[0] - M[1];  lr[1] =  M[4] - M[5];  lr[2] =  M[8] - M[9];

	if (havecolor) {
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
	} else {
		glDisable(GL_COLOR_MATERIAL);
		float gray[] = { 0.9f, 0.9f, 0.9f, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gray);
	}

	if (circ) {
		setuptexturing();
		glEnable(GL_POINT_SMOOTH);
		glAlphaFunc(GL_GREATER, 0.5f);
		glEnable(GL_ALPHA_TEST);
	}
}


// Draw a point at (cx, cy, cz).  Radius is r in world coords, which must
// correspond to a screen-space size (diameter) of splatsize.
// Normal (3 floats) is pointed to by norm.
// Color (3 floats) is pointed to by col.
void drawpoint_gl(float cx, float cy, float cz,
		  float r, float splatsize,
		  const float *norm, const float *col)
{
	if (splatsize <= point_size_thresh) {

		// ceilf is slow on many systems
		int pointsize = int(splatsize+0.99f);

		if (currentGLstate != GL_BEGIN_POINTS) {

			if (currentGLstate != GL_BEGIN_NONE)
				glEnd();
			glDisable(GL_TEXTURE_2D);
			if (pointsize != currentGLpointsize) {
				glPointSize(pointsize);
				currentGLpointsize = pointsize;
			}
			glBegin(GL_POINTS);
			currentGLstate = GL_BEGIN_POINTS;

		} else if (pointsize != currentGLpointsize) {
			glEnd();
			glPointSize(pointsize);
			currentGLpointsize = pointsize;
			glBegin(GL_POINTS);
		}

		if (col) glColor3fv(col);
		glNormal3fv(norm);
		glVertex3f(cx, cy, cz);

	} else if (circ) {

		if (currentGLstate != GL_BEGIN_TRIANGLES) {
			if (currentGLstate != GL_BEGIN_NONE)
				glEnd();
			glEnable(GL_TEXTURE_2D);
			glBegin(GL_TRIANGLES);
			currentGLstate = GL_BEGIN_TRIANGLES;
		}

		if (col) glColor3fv(col);
		glNormal3fv(norm);

		glTexCoord2f(1.0f, 0.5f);
		glVertex3f(cx + r * rdir[0],
			   cy + r * rdir[1],
			   cz + r * rdir[2]);
		glTexCoord2f(0.0f, 1.0f);
		glVertex3f(cx + r * uldir[0],
			   cy + r * uldir[1],
			   cz + r * uldir[2]);
		glTexCoord2f(0.0f, 0.0f);
		glVertex3f(cx + r * lldir[0],
			   cy + r * lldir[1],
			   cz + r * lldir[2]);

	} else {

		if (currentGLstate != GL_BEGIN_QUADS) {
			if (currentGLstate != GL_BEGIN_NONE)
				glEnd();
			glDisable(GL_TEXTURE_2D);
			glBegin(GL_QUADS);
			currentGLstate = GL_BEGIN_QUADS;
		}

		if (col) glColor3fv(col);
		glNormal3fv(norm);

		glVertex3f(cx + r * ur[0], cy + r * ur[1], cz + r * ur[2]);
		glVertex3f(cx - r * lr[0], cy - r * lr[1], cz - r * lr[2]);
		glVertex3f(cx - r * ur[0], cy - r * ur[1], cz - r * ur[2]);
		glVertex3f(cx + r * lr[0], cy + r * lr[1], cz + r * lr[2]);

	}

	theQSplatGUI->pts_splatted++;
}


// Little function called right after we finish drawing
void end_drawing_gl()
{
	if (currentGLstate != GL_BEGIN_NONE)
		glEnd();
	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_TEXTURE_2D);
}

