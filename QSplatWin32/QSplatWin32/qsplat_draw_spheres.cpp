/*
Szymon Rusinkiewicz

qsplat_draw_spheres.cpp
Very slow splat type - draw as spheres

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_guimain.h"
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>


// Local variables
static GLUquadricObj *q;


// Little function called right before we start drawing
void start_drawing_spheres(bool havecolor)
{
	q = gluNewQuadric();
#ifdef MESA
	gluQuadricDrawStyle(q, (GLenum)GLU_FILL);
	gluQuadricNormals(q, (GLenum)GLU_SMOOTH);
#else
	gluQuadricDrawStyle(q, GLU_FILL);
	gluQuadricNormals(q, GLU_SMOOTH);
#endif
	glShadeModel(GL_SMOOTH);

	if (havecolor) {
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
	} else {
		glDisable(GL_COLOR_MATERIAL);
		float gray[] = { 0.9f, 0.9f, 0.9f, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gray);
	}
	glMatrixMode(GL_MODELVIEW);
}


// Draw a point at (cx, cy, cz).  Radius is r in world coords, which must
// correspond to a screen-space size (diameter) of splatsize.
// Normal (3 floats) is pointed to by norm.
void drawpoint_spheres(float cx, float cy, float cz,
		       float r, float splatsize,
		       const float *norm, const float *col)
{
	if (col) glColor3fv(col);
	glPushMatrix();
	glTranslatef(cx, cy, cz);
	gluSphere(q, r, 8, 8);
	glPopMatrix();
	theQSplatGUI->pts_splatted++;
}


// Little function called right after we finish drawing
void end_drawing_spheres()
{
	gluDeleteQuadric(q);
}

