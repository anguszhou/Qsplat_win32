#ifndef QSPLAT_GUI_CAMERA_H
#define QSPLAT_GUI_CAMERA_H
/*
Szymon Rusinkiewicz

qsplat_gui_camera.h
A camera class for the QSplat GUI.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_util.h"


// Initial field of view of the camera
#define DEFAULT_FOV 0.8284271f


// Class for an OpenGL camera
class Camera {
public:
	point pos;
	float rot;
	vec rotaxis;
	float fov;

	void Init();
	Camera() { Init(); }
	void SetHome(const float *_pos, float _rot, const float *_rotaxis, float _fov);
	bool SetHome(const char *filename);
	void GoHome();

	void SetGL(float near, float far, float width, float height);

	void GetOrthoCoordSystem(vec &viewdir, vec &updir, vec &rightdir);

	void Move(float dx, float dy, float dz);
	void Rotate(const quat &q, float rotdist);
	void Zoom(float logzoom);

	void SaveForUndo();
	void Undo();

private:
	point homepos;
	float homerot;
	vec homerotaxis;
	float homefov;
	point savepos;
	float saverot;
	vec saverotaxis;
	float savefov;
};


// Some misc quaternion functions
extern void QNorm(quat &q);
extern void QCompose(const quat &q1, const quat &q2, quat &q3);
extern void RotAndAxis2Q(float rot, const vec &rotaxis, quat &q);
extern void Q2RotAndAxis(const quat &q, float &rot, vec &rotaxis);
extern void Mouse2Q(float x1, float y1, float x2, float y2, quat &q);
extern void QRotate(vec &x, const quat &q);

#endif
