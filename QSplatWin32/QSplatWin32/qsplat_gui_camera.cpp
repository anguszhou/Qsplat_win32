/*
Szymon Rusinkiewicz

qsplat_gui_camera.cpp
A camera class for the QSplat GUI.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include <stdio.h>
#include "qsplat_gui_camera.h"
#include <Windows.h>
#include <GL/gl.h>


// Radius of trackball
#define TRACKBALL_R 0.85f


// Initialize parameters of the camera to the usual
// (i.e. at origin, looking down negative Z axis, etc.)
void Camera::Init()
{
	pos[0] = homepos[0] = savepos[0] = 0;
	pos[1] = homepos[1] = savepos[1] = 0;
	pos[2] = homepos[2] = savepos[2] = 0;

	rot = homerot = saverot = 0;

	rotaxis[0] = homerotaxis[0] = saverotaxis[0] = 0;
	rotaxis[1] = homerotaxis[1] = saverotaxis[1] = 0;
	rotaxis[2] = homerotaxis[2] = saverotaxis[2] = 1;

	fov = homefov = savefov = DEFAULT_FOV;
}


// Set the "home" position of the camera
void Camera::SetHome(const float *_pos,
		     float _rot,
		     const float *_rotaxis,
		     float _fov)
{
	homepos[0] = _pos[0]; homepos[1] = _pos[1]; homepos[2] = _pos[2];
	homerot = _rot;
	homerotaxis[0] = _rotaxis[0]; homerotaxis[1] = _rotaxis[1]; homerotaxis[2] = _rotaxis[2];
	homefov = _fov;
}


// Set the "home" position of the camera by reading it from a .xf file that
// specifies a transformation matrix
bool Camera::SetHome(const char *filename)
{
	FILE *f = fopen(filename, "r");
	if (!f)
		return false;

	float M[16];
	for (int i=0; i < 4; i++)
		for (int j=0; j < 4; j++)
			fscanf(f, "%f", M+i+4*j);
	fclose(f);

	// Matrix -> Quat, from GGems IV III.4 (Ken Shoemake)
	// and Xform.h (Kari Pulli)
	quat q;
	float tr = M[0] + M[5] + M[10];
	if (tr >= 0.0f) {
		float s = sqrtf(tr + M[15]);
		q[0] = s * 0.5f;
		s = 0.5f / s;
		q[1] = (M[4*1+2] - M[4*2+1]) * s;
		q[2] = (M[4*2+0] - M[4*0+2]) * s;
		q[3] = (M[4*0+1] - M[4*1+0]) * s;
	} else {
		int i=0;
		if (M[5] > M[0])
			i = 1;
		if (M[10] > M[5*i])
			i = 2;
		int j = (i+1) % 3;
		int k = (j+1) % 3;
		float s = sqrtf(M[15] + M[5*i] - M[5*j] - M[5*k]);
		q[i+1] = 0.5f * s;
		s = 0.5f / s;
		q[j+1] = (M[4*i+j] + M[4*j+i]) * s;
		q[0]   = (M[4*j+k] - M[4*k+j]) * s;
		q[k+1] = (M[4*k+i] + M[4*i+k]) * s;
	}

	QNorm(q);
	Q2RotAndAxis(q, homerot, homerotaxis);

	homepos[0] = -(M[12]*M[0] + M[13]*M[4] + M[14]*M[8]);
	homepos[1] = -(M[12]*M[1] + M[13]*M[5] + M[14]*M[9]);
	homepos[2] = -(M[12]*M[2] + M[13]*M[6] + M[14]*M[10]);

	homefov = DEFAULT_FOV;
	return true;
}


// Reset the camera to the "home" position
void Camera::GoHome()
{
	SaveForUndo();
	pos[0] = homepos[0]; pos[1] = homepos[1]; pos[2] = homepos[2];
	rot = homerot;
	rotaxis[0] = homerotaxis[0]; rotaxis[1] = homerotaxis[1]; rotaxis[2] = homerotaxis[2];
	fov = homefov;
}


// Set up the OpenGL projection and modelview matrices for the current
// camera position, given screen width and height (in pixels) and
// distance to near and far planes (in world coordinates)
void Camera::SetGL(float neardist, float fardist, float width, float height)
{
	float diag = sqrtf(sqr(width) + sqr(height));
	float top = height/diag * 0.5f*fov * neardist;
	float bottom = -top;
	float right = width/diag * 0.5f*fov * neardist;
	float left = -right;

	glMatrixMode(GL_PROJECTION);
	glFrustum(left, right, bottom, top, neardist, fardist);

	glMatrixMode(GL_MODELVIEW);
	glRotatef(180.0f/M_PI*rot, rotaxis[0], rotaxis[1], rotaxis[2]);
	glTranslatef(-pos[0], -pos[1], -pos[2]);
}


// Find the vector that gives the view direction of the camera, as well as
// vectors pointing "up" and "right"
void Camera::GetOrthoCoordSystem(vec &viewdir, vec &updir, vec &rightdir)
{
	quat q;
	RotAndAxis2Q(rot, rotaxis, q);
	q[0] = -q[0];

	viewdir[0] = viewdir[1] = 0;  viewdir[2] = -1.0f;
	QRotate(viewdir, q);

	updir[0] = updir[2] = 0;  updir[1] = 1.0f;
	QRotate(updir, q);

	CrossProd(viewdir, updir, rightdir);
}


// Translate the camera by (dx, dy, dz).  Note that these are specified
// *relative to the camera's frame*
void Camera::Move(float dx, float dy, float dz)
{
	SaveForUndo();

	vec xaxis = {1, 0, 0};
	vec yaxis = {0, 1, 0};
	vec zaxis = {0, 0, 1};

	quat currq;
	RotAndAxis2Q(rot, rotaxis, currq);

	currq[0] = -currq[0];
	QRotate(xaxis, currq);
	QRotate(yaxis, currq);
	QRotate(zaxis, currq);

	pos[0] += xaxis[0]*dx + yaxis[0]*dy + zaxis[0]*dz;
	pos[1] += xaxis[1]*dx + yaxis[1]*dy + zaxis[1]*dz;
	pos[2] += xaxis[2]*dx + yaxis[2]*dy + zaxis[2]*dz;
}


// Rotate the camera by quaternion q, about a point lying along the camera's
// view direction, a distance "rotdist" away from the camera
void Camera::Rotate(const quat &q, float rotdist)
{
	SaveForUndo();

	vec viewdir, updir, rightdir;
	GetOrthoCoordSystem(viewdir, updir, rightdir);
	point rotcenter = { pos[0] + rotdist*viewdir[0],
			    pos[1] + rotdist*viewdir[1],
			    pos[2] + rotdist*viewdir[2] };

	quat currq;
	RotAndAxis2Q(rot, rotaxis, currq);

	QCompose(q, currq, currq);
	Q2RotAndAxis(currq, rot, rotaxis);

	GetOrthoCoordSystem(viewdir, updir, rightdir);

	pos[0] = rotcenter[0] - rotdist*viewdir[0];
	pos[1] = rotcenter[1] - rotdist*viewdir[1];
	pos[2] = rotcenter[2] - rotdist*viewdir[2];
}


// Zoom the camera (i.e. change field of view)
void Camera::Zoom(float logzoom)
{
	SaveForUndo();
	fov /= exp(logzoom);
}


// Save the current position of the camera for a possible future undo
void Camera::SaveForUndo()
{
	savepos[0] = pos[0]; savepos[1] = pos[1]; savepos[2] = pos[2];
	saverot = rot;
	saverotaxis[0] = rotaxis[0]; saverotaxis[1] = rotaxis[1]; saverotaxis[2] = rotaxis[2];
	savefov = fov;
}


// Undo the last camera motion
void Camera::Undo()
{
	swap(pos[0], savepos[0]);
	swap(pos[1], savepos[1]);
	swap(pos[2], savepos[2]);
	swap(rot, saverot);
	swap(rotaxis[0], saverotaxis[0]);
	swap(rotaxis[1], saverotaxis[1]);
	swap(rotaxis[2], saverotaxis[2]);
	swap(fov, savefov);
}


// Normalize a quaternion
void QNorm(quat &q)
{
	float x = sqrtf(sqr(q[0]) + sqr(q[1]) + sqr(q[2]) + sqr(q[3]));
	if (x == 0.0f) {
		q[0] = 1.0f;
		q[1] = q[2] = q[3] = 0.0f;
		return;
	}

	x = 1.0f / x;
	q[0] *= x;
	q[1] *= x;
	q[2] *= x;
	q[3] *= x;
}


// Compose two quaternions
void QCompose(const quat &q1, const quat &q2, quat &q3)
{
	// Separate var to hold output, so you can say QCompose(q1, q2, q2);
	quat qout;

	qout[0] = q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	qout[1] = q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
	qout[2] = q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
	qout[3] = q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];

	QNorm(qout);
	q3[0] = qout[0]; q3[1] = qout[1]; q3[2] = qout[2]; q3[3] = qout[3];
}


// Compute a quaternion from a rotation and axis
void RotAndAxis2Q(float rot, const vec &rotaxis, quat &q)
{
	float c2 = cos(0.5f*rot);
	float s2 = sin(0.5f*rot);

	float x = s2 / Len(rotaxis);

	q[0] = c2;
	q[1] = rotaxis[0] * x;
	q[2] = rotaxis[1] * x;
	q[3] = rotaxis[2] * x;
}


// Compute a rotation and axis from a quaternion
void Q2RotAndAxis(const quat &q, float &rot, vec &rotaxis)
{
	rot = 2.0f * acos(q[0]);

	if (rot == 0.0f) {
		rotaxis[0] = rotaxis[1] = 0.0f;
		rotaxis[2] = 1.0f;
		return;
	}

	rotaxis[0] = q[1];
	rotaxis[1] = q[2];
	rotaxis[2] = q[3];
	Normalize(rotaxis);
}


// Convert an (x,y) normalized mouse position to a position on the trackball
static void Mouse2TrackballPos(float x, float y, point &pos)
{
	float r2 = sqr(x) + sqr(y);
	float t = 0.5f * sqr(TRACKBALL_R);

	pos[0] = x;
	pos[1] = y;
	if (r2 < t)
		pos[2] = sqrtf(2.0f*t - r2);
	else
		pos[2] = t / sqrtf(r2);
        
	Normalize(pos);
}


// Takes normalized mouse positions (x1, y1) and (x2, y2) and returns a
// quaternion representing a rotation on the trackball
void Mouse2Q(float x1, float y1, float x2, float y2, quat &q)
{
	if ((x1 == x2) && (y1 == y2)) {
		q[0] = 1.0f;
		q[1] = q[2] = q[3] = 0.0f;
		return;
	}

	point pos1, pos2;
	Mouse2TrackballPos(x1, y1, pos1);
	Mouse2TrackballPos(x2, y2, pos2);

	vec rotaxis;
	CrossProd(pos1, pos2, rotaxis);
	Normalize(rotaxis);

	float rotangle = TRACKBALL_R * sqrtf(sqr(x2-x1) + sqr(y2-y1));

	RotAndAxis2Q(rotangle, rotaxis, q);
}


// Rotate vector x by quaternion q
void QRotate(vec &x, const quat &q)
{
	float xlen = Len(x);
	if (xlen == 0.0f)
		return;

	quat p = { 0, x[0], x[1], x[2] };
	quat qbar = { q[0], -q[1], -q[2], -q[3] };
	quat qtmp;
	QCompose(q, p, qtmp);
	QCompose(qtmp, qbar, qtmp);

	x[0] = qtmp[1];  x[1] = qtmp[2];  x[2] = qtmp[3];
	Normalize(x);
	x[0] *= xlen;  x[1] *= xlen;  x[2] *= xlen;
}

