/*
Szymon Rusinkiewicz

qsplat_guimain.cpp
The QSplat GUI.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include <stdio.h>
#include "qsplat_guimain.h"
#include "qsplat_util.h"
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>


// Various configuration parameters
#define SPECULAR	0.25f
#define AMBIENT		0.05f
#define LIGHT		0.85f
#define SHININESS	16.0f
#define DOF		30.0f
#define ROT_MULT	2.5f
#define TRANSZ_MULT	4.5f
#define WHEEL_MOVE	0.1f
#define RTSIZE		10
#define DEPTH_FUDGE	0.2f
#define REFINE_DELAY	0.5f
#define SHOWLIGHT_TIME	2.5f
#define SHOWPROG_TIME	1.5f


// The GUI global variable
QSplatGUI *theQSplatGUI;


// Tell the GUI to use this model
void QSplatGUI::SetModel(QSplat_Model *q)
{
	if (theQSplat_Model)
		delete theQSplat_Model;
	theQSplat_Model = q;
	if (!q)
		return;

	// Set to default rate
	theQSplat_Model->reset_rate();

	// Look for a .xf file, and set "home" camera position
	char xffilename[255];
	strcpy(xffilename, q->filename.c_str());
	xffilename[strlen(xffilename)-2] = 'x';
	xffilename[strlen(xffilename)-1] = 'f';
	if (!thecamera.SetHome(xffilename)) {
		point homepos = { q->center[0],
				  q->center[1],
				  q->center[2] + 3.0f * q->radius };
		point homerotaxis = { 0, 0, 1 };
		thecamera.SetHome(homepos, 0, homerotaxis, DEFAULT_FOV);
	}
}


// Do a redraw!  That's why we're here...
void QSplatGUI::redraw()
{
	// If we have no model, nothing to do
	if (!theQSplat_Model) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		swapbuffers();
		updatestatus("No model");
		updaterate("");
		return;
	}


	// We need to do resetviewer() before we start drawing, but
	// we can't do it until we're sure we have an OpenGL context.
	// So, we do it here...
	static bool first_time = true;
	if (first_time) {
		resetviewer();
		first_time = false;
	}

	// We're going to time how long the following takes
	timestamp start_time;
	get_timestamp(start_time);

	// Get the distance to the surface, so we know where to put clipping
	// planes.
	update_depth();

	// Set the viewport and clear the screen
	setupGLstate();
	if ((int)whichDriver < (int)SOFTWARE_GLDRAWPIXELS)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	// Set up projection and modelview matrices, and lighting
	setup_matrices(true);


	// Draw...
	pts_splatted = 0;
	bool bailed = theQSplat_Model->draw();


	// Examine what happened
	if (bailed) {
		if (dorefine) {
			// A run-of-the-mill move-the-mouse-to-stop-refine
			// sort of event happened
			stop_refine();
			return;
		} else {
			// That took waaaay longer than expected, and user
			// got bored.  Do a random fudge to try to avoid this
			// next time.
			theQSplat_Model->adjust_rate(4.0f);
			return;
		}
	}

	if (whichDriver != SOFTWARE &&
	    whichDriver != SOFTWARE_TILES &&
	    whichDriver != SOFTWARE_BEST) {
		draw_light();
		draw_progressbar();
		swapbuffers();
	}


	// Figure out how long that took
	timestamp end_time;
	get_timestamp(end_time);
	float elapsed = end_time - start_time;

	char ratemsg[255];
	sprintf(ratemsg, "%d points, %.3f sec.", pts_splatted, elapsed);
	updaterate(ratemsg);

	if (dorefine) {
		if (elapsed <= 1.0f/desiredrate)
			// We're refining, but this frame took shorter than the
			// desired rendering time.  Therefore, we can save
			// the current minsize and when we exit refining
			// it'll still be at an acceptable rate.
			theQSplat_Model->start_refine();
		return;
	}

	// Try to adjust the rate for the next frame based on how long
	// this one actually took
	float correction = elapsed * desiredrate;
	if (correction > 1.0f)
		correction *= sqrtf(sqrtf(correction));
	else
		correction /= sqrtf(sqrtf(correction));
	theQSplat_Model->adjust_rate(correction);
	return;
}


// Draw an indication of light source direction
void QSplatGUI::draw_light()
{
	if (showlight != SHOWLIGHT_ON)
		return;
	timestamp now;
	get_timestamp(now);
	if (now - last_relight_time > SHOWLIGHT_TIME) {
		showlight = SHOWLIGHT_OFF;
		return;
	}


	glPushAttrib(GL_VIEWPORT_BIT);
	int subwin_size = (int)((float)min(screenx(), screeny()) * 0.25f);
	int xoff = (screenx()-subwin_size) / 2;
	int yoff = (screeny()-subwin_size) / 2;
	glViewport(viewportx()+xoff, viewporty()+yoff, subwin_size, subwin_size);
	glScissor(viewportx()+xoff, viewporty()+yoff, subwin_size, subwin_size);
	glEnable(GL_SCISSOR_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-.03, .03, -.03, .03, .08, 20);
	glTranslatef(0, 0, -10);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	{
		GLfloat mat_diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
		GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
		GLfloat mat_shininess[] = { SHININESS };
		GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
		GLfloat light_diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
		GLfloat light_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
		GLfloat light_position[] = { lightdir[0], lightdir[1], lightdir[2], 0 };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_diffuse);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	}

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_COLOR_MATERIAL);

	GLUquadricObj *q = gluNewQuadric();
#ifdef MESA
	gluQuadricDrawStyle(q, (GLenum)GLU_FILL);
	gluQuadricNormals(q, (GLenum)GLU_SMOOTH);
#else
	gluQuadricDrawStyle(q, GLU_FILL);
	gluQuadricNormals(q, GLU_SMOOTH);
#endif
	
	gluSphere(q, 1.0f, 16, 8);
	if (lightdir[2] != 1.0f) {
		glRotatef(acos(lightdir[2])*180.0f/M_PI,
			  -lightdir[1], lightdir[0], 0);
	}
	glTranslatef(0, 0, 1.9f);

	{
		GLfloat mat_diffuse[] = { 0.5f, 1.0f, 0.5f, 1.0f };
		GLfloat light_position[] = { 0.0f, 0.0f, 1.0f, 0.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_diffuse);
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	}

	glRotatef(180, 1, 0, 0);
	gluCylinder(q, 0.3f, 0.0f, 0.8f, 8, 1);
	gluDisk(q, 0.0f, 0.3f, 8, 1);
	glRotatef(180, 1, 0, 0);
	glTranslatef(0, 0, 1.7f);

	{
		GLfloat mat_diffuse[] = { 0.5f, 0.5f, 1.0f, 1 };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_diffuse);
	}

	glRotatef(180, 1, 0, 0);
	gluCylinder(q, 0.1f, 0, 2.5f, 8, 1);

	gluDeleteQuadric(q);
	glPopAttrib();
}


// Draw an indication of how much we've refined
void QSplatGUI::draw_progressbar()
{
	if (showprogressbar != PROGRESS_ON)
		return;

	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glColor4f(0, 0, 0, 0.3f);
	glBegin(GL_QUADS);
		glVertex2f(-1.0f,   -1.0f);
		glVertex2f(-0.695f, -1.0f);
		glVertex2f(-0.695f, -0.965f);
		glVertex2f(-1.0f,   -0.965f);
	glEnd();
	glDisable(GL_BLEND);
	glColor3f(1, 1, 1);
	glBegin(GL_LINE_LOOP);
		glVertex2f(-0.995f, -0.995f);
		glVertex2f(-0.7f,   -0.995f);
		glVertex2f(-0.7f,   -0.97f);
		glVertex2f(-0.995f, -0.97f);
	glEnd();
	float refinement =  1.0f / theQSplat_Model->minsize;
	glBegin(GL_QUADS);
		glVertex2f(-0.995f,                   -0.995f);
		glVertex2f(-0.995f+0.295f*refinement, -0.995f);
		glVertex2f(-0.995f+0.295f*refinement, -0.97f);
		glVertex2f(-0.995f,                   -0.97f);
	glEnd();
	glPopAttrib();
	get_timestamp(last_progress_time);
}


// Process a mouse event
void QSplatGUI::mouse(int x, int y, mousebutton this_button)
{
	if (!theQSplat_Model)
		return;

	// We are not interested in mouse motion with no buttons pressed.
	if ((this_button == NO_BUTTON) && (last_button == NO_BUTTON))
		return;


	float this_mousex, this_mousey;
	mouse_i2f(x, y, &this_mousex, &this_mousey);


	// We are not interested in duplicate events
	if (this_mousex == last_mousex && this_mousey == last_mousey &&
	    this_button != UP_WHEEL && this_button != DOWN_WHEEL &&
	    this_button == last_button)
		return;

	// Handle rotation
	if ((this_button == ROT_BUTTON) && (last_button != ROT_BUTTON)) {
		spinspeed = 0;
	}
	if ((this_button == ROT_BUTTON) && (last_button == ROT_BUTTON)) {
		rotate(last_mousex, last_mousey, this_mousex, this_mousey);
		updatestatus("Rotating");
	}
	if ((this_button == NO_BUTTON)  && (last_button == ROT_BUTTON)) {
		startspin();
	}

	// Handle translation
	if ((this_button == TRANSXY_BUTTON) && (last_button == TRANSXY_BUTTON)) {
		move(this_mousex - last_mousex, this_mousey - last_mousey, 0);
		updatestatus("Panning");
	}
	if ((this_button == TRANSZ_BUTTON) && (last_button == TRANSZ_BUTTON)) {
		move(0, 0, last_mousey - this_mousey - (last_mousex - this_mousex));
		updatestatus("Zooming");
	}
	if (this_button == UP_WHEEL) {
		move(0, 0, -WHEEL_MOVE);
		updatestatus("Zooming");
	}
	if (this_button == DOWN_WHEEL) {
		move(0, 0, WHEEL_MOVE);
		updatestatus("Zooming");
	}

	// Handle lighting
	if ((this_button == LIGHT_BUTTON) && (last_button == LIGHT_BUTTON)) {
		relight(this_mousex, this_mousey);
		updatestatus("Relighting");
	}

	// Various other housekeeping chores
	if ((last_button == NO_BUTTON) || (this_button == last_button)) {
		// Button press or drag
		get_timestamp(last_event_time);
		dospin = false;
		stop_refine();
	} else if (this_button == NO_BUTTON) {
		// Button release
		update_depth(!dospin);
		if (!dospin &&
		    last_button != UP_WHEEL && last_button != DOWN_WHEEL) {
			theQSplat_Model->start_refine();
			dorefine = true;
		}
	}

	last_mousex = this_mousex;
	last_mousey = this_mousey;
	last_button = this_button;
}


// Our idle loop.  Handles auto-spin and auto-refine.
// Returns true iff we have refined all the way and are not spinning
bool QSplatGUI::idle()
{
	if (!theQSplat_Model) {
		updatestatus("No model");
		updaterate("");
		return true;
	}

	// Measure elapsed time since the last mouse event or idle loop
	timestamp new_time;
	get_timestamp(new_time);
	float dt = new_time - last_event_time;

	if ((showlight == SHOWLIGHT_ON) &&
	    (new_time - last_relight_time > SHOWLIGHT_TIME)) {
		showlight = SHOWLIGHT_OFF;
		need_redraw();
	}
	if ((showprogressbar == PROGRESS_ON) &&
	    (new_time - last_progress_time > SHOWPROG_TIME)) {
		showprogressbar = PROGRESS_OFF;
		need_redraw();
	}


	if (dospin) {
		float q[4];
		RotAndAxis2Q(spinspeed*dt, spinaxis, q);
		if (rot_depth == 0) { 
			q[1] = -q[1];
			q[2] = -q[2];
		}
		thecamera.Rotate(q, rot_depth);
		need_redraw();
		last_event_time = new_time;
		updatestatus("Spinning");
		return false;
	}

	if (dorefine && !theQSplat_Model->can_refine()) {
		updatestatus("Done refining");
		if (showlight == SHOWLIGHT_ON || showprogressbar == PROGRESS_ON) {
			usleep(10000);
			return false;
		} else {
			return true;
		}
	}

	if (!dorefine) {
		if (dt < REFINE_DELAY)
			return false;
		theQSplat_Model->start_refine();
		dorefine = true;
	}

	theQSplat_Model->refine();
	if (showprogressbar == PROGRESS_OFF)
		showprogressbar = PROGRESS_ON;
	need_redraw();
	updatestatus("Refining");
	return false;
}


// Reset viewer to default parameters
void QSplatGUI::resetviewer(bool splatsize_only /* = false */)
{
	if (!theQSplat_Model)
		return;

	theQSplat_Model->reset_rate();
	theQSplat_Model->start_refine();
	dorefine = true;
	dospin = false;

	if (splatsize_only)
		return;

	thecamera.GoHome();

	rot_depth = surface_depth =
		Dist(thecamera.pos, theQSplat_Model->center);

	lightdir[0] = 0;  lightdir[1] = 0;  lightdir[2] = 1;

	update_depth(true);
}


// Set whether surface material should have a specular component
void QSplatGUI::set_shiny(bool is_shiny)
{
	if (is_shiny)
		specular = SPECULAR;
	else
		specular = 0;

	stop_refine();
}


// Should we do backface culling?
void QSplatGUI::set_backfacecull(bool do_cull)
{
	backfacecull = do_cull;
	stop_refine();
}


// User has moved the "desired rate" slider
void QSplatGUI::set_desiredrate(float rate)
{
	float rate_ratio = rate / desiredrate;
	desiredrate = rate;
	if (!theQSplat_Model)
		return;
	stop_refine();
	theQSplat_Model->adjust_rate(rate_ratio);
}


// Switch into or out of tourist (restricted functionality) mode.
void QSplatGUI::set_touristmode(bool i_am_confused)
{
	if (i_am_confused) {
		touristmode = true;
		dospin = false;
	} else {
		touristmode = false;
	}
	stop_refine();
}


// Should we display an indication of the light source direction?
void QSplatGUI::set_showlight(bool do_light)
{
	if (do_light)
		showlight = SHOWLIGHT_ON;
	else
		showlight = SHOWLIGHT_NEVER;
	get_timestamp(last_relight_time);
}


// Should we display a graphical indication of refinement?
void QSplatGUI::set_showprogressbar(bool do_prog)
{
	if (do_prog)
		showprogressbar = PROGRESS_ON;
	else
		showprogressbar = PROGRESS_NEVER;
	get_timestamp(last_progress_time);
}

// Set up a consistent set of OpenGL modes
void QSplatGUI::setupGLstate()
{
	glViewport(viewportx(), viewporty(), screenx(), screeny());

	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_DITHER);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glShadeModel(GL_FLAT);

	glClearColor(0, 0, 0, 1);
	glClearDepth(1);
}


// Set up projection and modelview matrices, and (optionally) lighting
void QSplatGUI::setup_matrices(bool dolighting)
{
	// Reset everything to identity
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Set up light position before we load the modelview matrix
	if (dolighting) {
		GLfloat light0_position[] = { lightdir[0], lightdir[1], lightdir[2], 0 };
		glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	}

	// Set up projection and modelview matrices
	thecamera.SetGL(surface_depth/DOF, surface_depth*DOF,
			screenx(), screeny());

	// Set up lighting
	if (dolighting) {
		GLfloat mat_specular[4] = { specular, specular, specular, 0 };
		GLfloat mat_shininess[] = { 127 };
		GLfloat global_ambient[] = { AMBIENT, AMBIENT, AMBIENT, 1 };
		GLfloat light0_ambient[] = { 0, 0, 0, 1 };
		GLfloat light0_diffuse[] = { LIGHT, LIGHT, LIGHT, 1.0 };
		GLfloat light0_specular[] = { LIGHT, LIGHT, LIGHT, 1.0 };
		// light0_position is set above...

		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
		glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
		glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
		glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
	}

	// Backface culling
	if (backfacecull) {
		glCullFace(GL_BACK);
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
}


// If we were auto-refining, stop it
void QSplatGUI::stop_refine()
{
	if (theQSplat_Model && dorefine)
		theQSplat_Model->stop_refine();
	dorefine = false;
	if (showprogressbar == PROGRESS_OFF)
		showprogressbar = PROGRESS_ON;
}


// Updates the distance from the camera center to the rotation center.
// (i.e. recomputes surface_depth and rot_depth)
void QSplatGUI::update_depth(bool update_rot_depth /* = false */,
			     float hint /* = 0.0f */)
{
	if (!theQSplat_Model)
		return;

	setup_matrices(false); // traceray reads camera params from OpenGL
	float depth = theQSplat_Model->traceray(screenx() / 2,
						screeny() / 2,
						RTSIZE);
	if (!depth && hint) {
		if (hint + surface_depth > 1.0e-5 * theQSplat_Model->radius)
			depth = surface_depth + hint;
	}

	if (!depth)
		return;

	surface_depth = depth;
	if (!update_rot_depth)
		return;

	// Rotate around the surface if we are close to it, else around
	// the center of the object.
	float dist2center = sqrtf(sqr(thecamera.pos[0]-theQSplat_Model->center[0]) +
				  sqr(thecamera.pos[1]-theQSplat_Model->center[1]) +
				  sqr(thecamera.pos[2]-theQSplat_Model->center[2]));

	rot_depth = min(depth*(1.0f+DEPTH_FUDGE*thecamera.fov), dist2center);

}


// Mouse event helper function - rotate
void QSplatGUI::rotate(float ox, float oy, float nx, float ny)
{
	// Random multiplicative factor - adjust to taste
	nx += (ROT_MULT-1)*(nx-ox);
	ny += (ROT_MULT-1)*(ny-oy);

	float q[4];
	Mouse2Q(ox, oy, nx, ny, q);

	if (rot_depth == 0) {
		q[1] = -q[1];
		q[2] = -q[2];
	}

	thecamera.Rotate(q, rot_depth);


	// Update autospin params
	float this_rot;
	Q2RotAndAxis(q, this_rot, spinaxis);

	timestamp new_time;
	get_timestamp(new_time);
	float dt = new_time - last_event_time;

	const float time_const = 0.05f + 1.0f / desiredrate;
	if (dt > time_const)
		spinspeed = this_rot / time_const;
	else
		// Exponential decay.  Math trick.  Think about it.
		spinspeed = (this_rot/time_const) +
				(1.0f-dt/time_const)*spinspeed;

	last_event_time = new_time;
	need_redraw();
}


// Mouse event helper function - user has let go of "rotate" mouse button. 
// Should we start auto-spin?
void QSplatGUI::startspin()
{
	// No spinning in tourist mode
	if (touristmode)
		return;

	timestamp new_time;
	get_timestamp(new_time);
	float dt = new_time - last_event_time;

	const float time_const = 0.05f + 1.0f / desiredrate;
	if ((dt < time_const) && (fabs(spinspeed) > 0.03f))
		dospin = true;
}


// Mouse event helper function - translate
void QSplatGUI::move(float dx, float dy, float dz)
{
	float scalefactor = -0.5f*surface_depth;
	dx *= scalefactor*thecamera.fov;
	dy *= scalefactor*thecamera.fov;
	dz = surface_depth * (exp(-0.5f * TRANSZ_MULT * dz) - 1.0f);

	thecamera.Move(dx, dy, dz);

	// We adjust the recursion threshold *up* whether we zoom in or out
	// The rationale is that if we zoom in, we might need to page in
	// lower levels of the hierarchy.
	// On the other hand, if we zoom out, we're looking at geometry that
	// was previously off-screen, and might have gotten paged out.
	float madjust = exp(fabs(dz/scalefactor));
	theQSplat_Model->adjust_rate(madjust);

	update_depth(true, dz);
	need_redraw();
}


// Mouse event helper function - change direction of light
void QSplatGUI::relight(float x, float y)
{
	float theta = M_PI*sqrtf(x*x+y*y);
	if (theta > float(M_PI))
		theta = float(M_PI);
	float phi = atan2(y, x);


	if (touristmode)
		// Too easy to shoot yourself in the foot with backlighting
		theta *= 0.4f;

	lightdir[0] = sin(theta)*cos(phi);
	lightdir[1] = sin(theta)*sin(phi);
	lightdir[2] = cos(theta);

	get_timestamp(last_relight_time);
	if (showlight == SHOWLIGHT_OFF)
		showlight = SHOWLIGHT_ON;

	need_redraw();
}


// Helper function - convert integer coordinates of mouse position to
// normalized floats
void QSplatGUI::mouse_i2f(int x, int y, float *xf, float *yf)
{
	int screenwidth = screenx();
	int screenheight = screeny();

	float r = (float)screenwidth/screenheight;
	if (r > 1.0) {
		*xf = r*(2.0f*x/screenwidth-1.0f);
		*yf = 2.0f*y/screenheight-1.0f;
	} else {
		*xf = 2.0f*x/screenwidth-1.0f;
		*yf = (2.0f*y/screenheight-1.0f)/r;
	}
}


// Various splat rasterizers...
extern void start_drawing_gl(bool,int,bool);
extern void drawpoint_gl(float,float,float,float,float,const float *, const float *);
extern void end_drawing_gl();

extern void start_drawing_gl_ellip(bool,bool);
extern void drawpoint_gl_ellip(float,float,float,float,float,const float *, const float *);
extern void end_drawing_gl_ellip();

extern void start_drawing_spheres(bool);
extern void drawpoint_spheres(float,float,float,float,float,const float *, const float *);
extern void end_drawing_spheres();

extern void drawpoint_software(float,float,float,float,float,const float *, const float *);
extern void start_drawing_software(bool);
extern void end_drawing_software(bool);

extern void drawpoint_software_tiles(float,float,float,float,float,const float*, const float*);
extern void start_drawing_software_tiles(bool);
extern void end_drawing_software_tiles(bool);

#define SOFTWARE_TILES_CROSSOVER 7


// Set up the splat renderer
void QSplatGUI::start_drawing(bool havecolor)
{
	float ps[2];
	glGetFloatv(GL_POINT_SIZE_RANGE, ps);

	switch (whichDriver) {
	case OPENGL_POINTS:
	case OPENGL_POINTS_CIRC:
		drawpoint = drawpoint_gl;
		start_drawing_gl(havecolor, int(ps[1]), whichDriver == OPENGL_POINTS_CIRC);
		break;
	case OPENGL_QUADS:
	case OPENGL_POLYS_CIRC:
		drawpoint = drawpoint_gl;
		start_drawing_gl(havecolor, 0, whichDriver == OPENGL_POLYS_CIRC);
		break;
	case OPENGL_POLYS_ELLIP:
	case OPENGL_POLYS_ELLIP_SMALL:
		drawpoint = drawpoint_gl_ellip;
		start_drawing_gl_ellip(havecolor, whichDriver == OPENGL_POLYS_ELLIP_SMALL);
		break;
	case OPENGL_SPHERES:
		drawpoint = drawpoint_spheres;
		start_drawing_spheres(havecolor);
		break;
	case SOFTWARE_GLDRAWPIXELS:
	case SOFTWARE:
		drawpoint = drawpoint_software;
		start_drawing_software(whichDriver == SOFTWARE_GLDRAWPIXELS);
		break;
	case SOFTWARE_TILES_GLDRAWPIXELS:
	case SOFTWARE_TILES:
		drawpoint = drawpoint_software_tiles;
		start_drawing_software_tiles(whichDriver == SOFTWARE_TILES_GLDRAWPIXELS);
		break;
	case SOFTWARE_BEST_GLDRAWPIXELS:
	case SOFTWARE_BEST:
		if (theQSplat_Model->minsize < SOFTWARE_TILES_CROSSOVER) {
			drawpoint = drawpoint_software;
			start_drawing_software(whichDriver == SOFTWARE_BEST_GLDRAWPIXELS);
		} else {
			drawpoint = drawpoint_software_tiles;
			start_drawing_software_tiles(whichDriver == SOFTWARE_BEST_GLDRAWPIXELS);
		}
		break;
	default:
		drawpoint = drawpoint_gl;
		start_drawing_gl(havecolor, 0, false);
		break;
	}
}


// Wrap up the rendering
void QSplatGUI::end_drawing(bool bailed)
{
	switch (whichDriver) {
	case OPENGL_POINTS:
	case OPENGL_POINTS_CIRC:
	case OPENGL_QUADS:
	case OPENGL_POLYS_CIRC:
		end_drawing_gl();
		break;
	case OPENGL_POLYS_ELLIP:
	case OPENGL_POLYS_ELLIP_SMALL:
		end_drawing_gl_ellip();
		break;
	case OPENGL_SPHERES:
		end_drawing_spheres();
		break;
	case SOFTWARE_GLDRAWPIXELS:
	case SOFTWARE:
		end_drawing_software(bailed);
		break;
	case SOFTWARE_TILES_GLDRAWPIXELS:
	case SOFTWARE_TILES:
		end_drawing_software_tiles(bailed);
		break;
	case SOFTWARE_BEST_GLDRAWPIXELS:
	case SOFTWARE_BEST:
		if (theQSplat_Model->minsize < SOFTWARE_TILES_CROSSOVER)
			end_drawing_software(bailed);
		else
			end_drawing_software_tiles(bailed);
		break;
	default:
		end_drawing_gl();
	}
}

