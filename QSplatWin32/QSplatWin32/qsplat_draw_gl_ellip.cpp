/*
Szymon Rusinkiewicz

qsplat_draw_gl_ellip.cpp
Draw elliptical splats using texture-mapped OpenGL polygons

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_guimain.h"
#include <Windows.h>
#include <GL/gl.h>


// Local variables
static vec rdir, uldir, lldir;
static point campos;
static const float sqrt5 = 2.236068f;
static float splatscale;


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
void start_drawing_gl_ellip(bool havecolor, bool smallsplats)
{
	if (smallsplats)
		splatscale = 1.0f;
	else
		splatscale = sqrt5;

	float M[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, M);

	rdir[0] = M[0]; rdir[1]  = M[4]; rdir[2]  = M[8];
	uldir[0] = M[1] - M[0]; uldir[1] = M[5] - M[4]; uldir[2] = M[9] - M[8];
	lldir[0] = -M[1] - M[0]; lldir[1] = -M[5] - M[4]; lldir[2] = -M[9] - M[8];

	campos[0] = -(M[0]*M[12] + M[1]*M[13] + M[2]*M[14]);
	campos[1] = -(M[4]*M[12] + M[5]*M[13] + M[6]*M[14]);
	campos[2] = -(M[8]*M[12] + M[9]*M[13] + M[10]*M[14]);

	if (havecolor) {
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
	} else {
		glDisable(GL_COLOR_MATERIAL);
		float gray[] = { 0.9f, 0.9f, 0.9f, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gray);
	}

	setuptexturing();
	glAlphaFunc(GL_GREATER, 0.5f);
	glEnable(GL_ALPHA_TEST);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_TRIANGLES);
}


// Draw a point at (cx, cy, cz).  Radius is r in world coords, which must
// correspond to a screen-space size (diameter) of splatsize.
// Normal (3 floats) is pointed to by norm.
// Color (3 floats) is pointed to by col.
void drawpoint_gl_ellip(float cx, float cy, float cz,
			float r, float splatsize,
			const float *norm, const float *col)
{
	if (col) glColor3fv(col);
	glNormal3fv(norm);

	vec splat_to_cam = { campos[0] - cx, campos[1] - cy, campos[2] - cz };

	vec x, y;
	CrossProd(splat_to_cam, norm, x);
	float lx = Len(x);
	if (lx == 0.0f) {
		glTexCoord2f(1.0f, 0.5f);
		glVertex3f(cx + r * splatscale * rdir[0],
			   cy + r * splatscale * rdir[1],
			   cz + r * splatscale * rdir[2]);
		glTexCoord2f(0.0f, 1.0f);
		glVertex3f(cx + r * splatscale * uldir[0],
			   cy + r * splatscale * uldir[1],
			   cz + r * splatscale * uldir[2]);
		glTexCoord2f(0.0f, 0.0f);
		glVertex3f(cx + r * splatscale * lldir[0],
			   cy + r * splatscale * lldir[1],
			   cz + r * splatscale * lldir[2]);

		theQSplatGUI->pts_splatted++;
		return;
	}

	CrossProd(norm, x, y);

	float rdist2 = 1.0f / (sqr(splat_to_cam[0]) +
			sqr(splat_to_cam[1]) + sqr(splat_to_cam[2]));
	float xdot = Dot(x, splat_to_cam) * rdist2;
	x[0] -= splat_to_cam[0] * xdot;
	x[1] -= splat_to_cam[1] * xdot;
	x[2] -= splat_to_cam[2] * xdot;

	float ydot = Dot(y, splat_to_cam) * rdist2;
	y[0] -= splat_to_cam[0] * ydot;
	y[1] -= splat_to_cam[1] * ydot;
	y[2] -= splat_to_cam[2] * ydot;

	float scale = r * splatscale / lx;

	glTexCoord2f(1.0f, 0.5f);
	glVertex3f(cx + scale * x[0],
		   cy + scale * x[1],
		   cz + scale * x[2]);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(cx - scale * (x[0]-y[0]),
		   cy - scale * (x[1]-y[1]),
		   cz - scale * (x[2]-y[2]));
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(cx - scale * (x[0]+y[0]),
		   cy - scale * (x[1]+y[1]),
		   cz - scale * (x[2]+y[2]));

	theQSplatGUI->pts_splatted++;
}


// Little function called right after we finish drawing
void end_drawing_gl_ellip()
{
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_ALPHA_TEST);
}

