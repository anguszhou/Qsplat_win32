/*
Szymon Rusinkiewicz

qsplat_gui_xforms.cpp
The infrastructure for the QSplat GUI.
This version is written for *NIX-like systems using the XForms toolkit.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include "qsplat_util.h"
#include "qsplat_gui_xforms.h"


// Forward declarations of event handlers
static int exposeEvent(FL_OBJECT *o, Window win, int w, int h, XEvent *ev, void *data);
static int motionEvent(FL_OBJECT *o, Window win, int w, int h, XEvent *ev, void *data);
static int buttonEvent(FL_OBJECT *o, Window win, int w, int h, XEvent *ev, void *data);
static int event_peek(FL_FORM *form, void *xev);
static bool expose_seen = false;
static int xforms_idle(XEvent *xev, void *data);


// Done drawing - swap buffers
void QSplatXFormsGUI::swapbuffers()
{
	glXSwapBuffers(fl_display, glCanvasWindow);
}


// Display usage
void QSplatXFormsGUI::usage(const char *myname)
{
	fprintf(stderr, "Usage: %s [model.qs]\n", myname);
	exit(1);
}



// Parse any command-line parameters that belong to the GUI
QSplatXFormsGUI::QSplatXFormsGUI(int *argcp, char *argv[]) : QSplatGUI()
{
	fprintf(stderr, "This is QSplat version %s\n", QSPLAT_VERSION);

	fl_initialize(argcp, argv, "QSplat", 0, 0);

	if (*argcp < 2)
		return;

	// Parse command-line params
	if (!strncasecmp(argv[1], "-h", 2) ||
	    !strncasecmp(argv[1], "--h", 3))
		usage(argv[0]);

	// Load the scan
	const char *scanfilename = argv[1];
	QSplat_Model *q = QSplat_Model::Open(scanfilename);
	if (!q)
		exit(1);
	SetModel(q);
}


// Fire up the GUI
void QSplatXFormsGUI::Go()
{
	static const char myname[] = "QSplat";

	// Create the GUI, and register callbacks
	fl_set_font_name(0, "-*-helvetica-medium-r-normal-*-*-?-75-75-*-*-*-*");
	fl_set_font_name(1, "-*-helvetica-bold-r-normal-*-*-?-75-75-*-*-*-*"); 
	fl_set_font_name(8, "-*-times-medium-r-normal-*-*-?-75-75-*-*-*-*"); 
	fl_set_font_name(9, "-*-times-bold-r-normal-*-*-?-75-75-*-*-*-*");

	fd_qsplat_gui = create_form_qsplat_gui();
	FL_OBJECT *glc = fd_qsplat_gui->theGLCanvas;
	fl_add_canvas_handler(glc, Expose, exposeEvent, 0);
	fl_add_canvas_handler(glc, MotionNotify, motionEvent, 0);
	fl_add_canvas_handler(glc, ButtonPress, buttonEvent, 0);
	fl_add_canvas_handler(glc, ButtonRelease, buttonEvent, 0);
	fl_register_raw_callback(fd_qsplat_gui->qsplat_gui, FL_ALL_EVENT, event_peek);
	fl_set_idle_callback(xforms_idle, 0);
	fl_set_idle_delta(1);


	// Show the GUI
	window_name = myname;
	mainwindow = fl_show_form(fd_qsplat_gui->qsplat_gui,
				  FL_PLACE_CENTERFREE, FL_FULLBORDER,
				  window_name);

	glCanvasWindow = FL_ObjWin(glc);
	fl_remove_selected_xevent(glCanvasWindow, PointerMotionMask|PointerMotionHintMask);


	// Set initial driver
	if (!strncmp((const char *)glGetString(GL_RENDERER), "Mesa", 4))
		whichDriver = SOFTWARE_BEST;
	else
		whichDriver = OPENGL_POINTS;

	fl_set_menu_item_mode(fd_qsplat_gui->DriverMenu,
			      int(whichDriver)+1,
			      FL_PUP_RADIO | FL_PUP_CHECK);

	// Set the initial desiredrate, and update the slider
	if ((int)whichDriver < (int)SOFTWARE_GLDRAWPIXELS) {
		set_desiredrate(8.0f);
		fl_set_slider_value(fd_qsplat_gui->DesiredRate, 8.0f);
	} else {
		set_desiredrate(4.0f);
		fl_set_slider_value(fd_qsplat_gui->DesiredRate, 4.0f);
	}

	// Guess whether we have a two-button or three-button mouse
	if (getenv("TWO_BUTTON_MOUSE")) {
		two_button_mouse = true;
	} else if (getenv("THREE_BUTTON_MOUSE")) {
		two_button_mouse = false;
	} else {
		two_button_mouse = !system("grep -i '^[^#]*emulate3buttons' /etc/X11/XF86Config > /dev/null 2>&1");
	}

	updatemenus();

	// Start the main GUI loop
	fl_do_forms();
}


// Enter or leave fullscreen mode
void QSplatXFormsGUI::fullscreen(bool go_fullscreen)
{
	if (go_fullscreen) {
		fl_get_winsize(mainwindow, &save_window_width, &save_window_height);
		fl_hide_form(fd_qsplat_gui->qsplat_gui);
		fl_show_form(fd_qsplat_gui->qsplat_gui,
			FL_PLACE_FULLSCREEN, FL_NOBORDER, window_name);

		// Some window managers are evil and won't actually put us at
		// (0,0), so we try to forcibly move ourselves there
		int rootwidth, rootheight;
		fl_get_winsize(fl_root, &rootwidth, &rootheight);
		fl_set_form_geometry(fd_qsplat_gui->qsplat_gui,
			0, 0, rootwidth, rootheight);

		// Resize the canvas portion of the display to obscure the
		// buttons 'n such
		/*
		fl_set_object_geometry(fd_qsplat_gui->theGLCanvas,
			10, 10, rootwidth-20, rootheight-20);
		*/
		/*
		fl_set_object_geometry(fd_qsplat_gui->theGLCanvas,
			0, 0, rootwidth, rootheight);
		*/
		fl_redraw_form(fd_qsplat_gui->qsplat_gui);

		glCanvasWindow = FL_ObjWin(fd_qsplat_gui->theGLCanvas);
	} else {
		fl_hide_form(fd_qsplat_gui->qsplat_gui);
		fl_set_form_size(fd_qsplat_gui->qsplat_gui,
			save_window_width, save_window_height);
		mainwindow = fl_show_form(fd_qsplat_gui->qsplat_gui,
			FL_PLACE_CENTERFREE, FL_FULLBORDER, window_name);
		fl_set_object_geometry(fd_qsplat_gui->theGLCanvas,
			10, 40, save_window_width-20, save_window_height-80);
		glCanvasWindow = FL_ObjWin(fd_qsplat_gui->theGLCanvas);
	}
}


// Update the rate bar
void QSplatXFormsGUI::updaterate(const char *text)
{
	fl_set_object_label(fd_qsplat_gui->RateBar, text);
}


// Update the status bar
void QSplatXFormsGUI::updatestatus(const char *text)
{
	fl_set_object_label(fd_qsplat_gui->StatusBar, text);
}


// Display an "About QSplat" box
void QSplatXFormsGUI::aboutQSplat()
{
	char buf[1024];
	sprintf(buf, "QSplat,  version %s\n"
		     "by\nSzymon Rusinkiewicz\nGary King\n\n"
		     "Copyright (c) 1999-2000 The Board of Trustees of the\nLeland Stanford Junior University. All Rights Reserved.",
		QSPLAT_VERSION);
	fl_show_messages(buf);
}


// Display an "About this model" box
void QSplatXFormsGUI::aboutmodel()
{
	if (theQSplat_Model)
		fl_show_messages(theQSplat_Model->comments.c_str());
	else
		fl_show_messages("No model loaded");
}


// Wrapper around the mouse callback - basically "decodes" mouse buttons
void QSplatXFormsGUI::mouseCB(int x, int y, unsigned b)
{
	if (!two_button_mouse) {
		// Standard 3-button Inventor-like mouse bindings
		if (!(b & AnyButtonMask))
			mouse(x, y, NO_BUTTON);
		else if (b & Button4Mask)
			mouse(x, y, UP_WHEEL);
		else if (b & Button5Mask)
			mouse(x, y, DOWN_WHEEL);
		else if ((b & AnyButtonMask) && (b & (ShiftMask | ControlMask)))
			mouse(x, y, LIGHT_BUTTON);
		else if ((b & Button1Mask) && (b & (Button2Mask | Button3Mask)))
			mouse(x, y, TRANSZ_BUTTON);
		else if (b & Button1Mask)
			mouse(x, y, ROT_BUTTON);
		else if (b & Button2Mask)
			mouse(x, y, TRANSXY_BUTTON);
		else if (b & Button3Mask)
			mouse(x, y, LIGHT_BUTTON);
		/* else do nothing... */
	} else {
		// Modified mouse bindings for 2-button mice
		if (!(b & AnyButtonMask))
			mouse(x, y, NO_BUTTON);
		else if (b & Button4Mask)
			mouse(x, y, UP_WHEEL);
		else if (b & Button5Mask)
			mouse(x, y, DOWN_WHEEL);
		else if ((b & AnyButtonMask) && (b & (ShiftMask | ControlMask)))
			mouse(x, y, LIGHT_BUTTON);
		else if ((b & Button1Mask) && (b & Button3Mask))
			mouse(x, y, TRANSZ_BUTTON);
		else if ((b & Button2Mask) && (b & Button3Mask))
			mouse(x, y, LIGHT_BUTTON);
		else if (b & Button1Mask)
			mouse(x, y, ROT_BUTTON);
		else if (b & Button2Mask)
			mouse(x, y, TRANSZ_BUTTON);
		else if (b & Button3Mask)
			mouse(x, y, TRANSXY_BUTTON);
		/* else do nothing... */
	}
}


// Gray or ungray a menu item
static void grayoutMenuItem(FL_OBJECT *ob, int numb, bool gray)
{
	unsigned m = fl_get_menu_item_mode(ob, numb);
	if (gray)
		m |= FL_PUP_GRAY;
	else
		m &= ~FL_PUP_GRAY;
	fl_set_menu_item_mode(ob, numb, m);
}


// Update the menus to show what's available
void QSplatXFormsGUI::updatemenus()
{
	bool softwaredriver = ((int)whichDriver >= (int)SOFTWARE_GLDRAWPIXELS);
	grayoutMenuItem(fd_qsplat_gui->OptionsMenu, 1, softwaredriver);

	bool nonopengldriver = (whichDriver == SOFTWARE ||
				whichDriver == SOFTWARE_TILES ||
				whichDriver == SOFTWARE_BEST);
	grayoutMenuItem(fd_qsplat_gui->OptionsMenu, 3, nonopengldriver);
	grayoutMenuItem(fd_qsplat_gui->OptionsMenu, 4, nonopengldriver);

	bool nomodel = !theQSplat_Model;
	grayoutMenuItem(fd_qsplat_gui->CommandsMenu, 2, nomodel);
	grayoutMenuItem(fd_qsplat_gui->HelpMenu, 2, nomodel);
}


// Handler for expose events
static int exposeEvent(FL_OBJECT *o, Window win, int w, int h, XEvent *ev, void *data)
{
	GUI->canvas_width = w;
	GUI->canvas_height = h;
	GUI->resetviewer(true);
	GUI->need_redraw();
	return 0;
}


// Handler for mouse motion events
static int motionEvent(FL_OBJECT *o, Window win, int w, int h, XEvent *ev, void *data)
{
	XMotionEvent *mev = (XMotionEvent *)ev;
	GUI->mouseCB(mev->x, GUI->canvas_height - 1 - mev->y, mev->state);
	return 0;
}


// Handler for mouse presses and releases
static int buttonEvent(FL_OBJECT *o, Window win,
		       int w, int h, XEvent *ev,
		       void *data)
{
	XButtonEvent *bev = (XButtonEvent *)ev;
	unsigned state = bev->state;

	// state is the *old* state, so we update it to reflect the button
	// that was just pressed/released.
	switch (bev->button) {
		case 1: state ^= Button1Mask; break;
		case 2: state ^= Button2Mask; break;
		case 3: state ^= Button3Mask; break;
		case 4: state ^= Button4Mask; break;
		case 5: state ^= Button5Mask; break;
	}

	GUI->mouseCB(bev->x, GUI->canvas_height - 1 - bev->y, state);
	return 0;
}


// This is a function that gets called with all the events for the form.
// We use it to keep track of expose events, to try to work around an
// XForms library bug
static int event_peek(FL_FORM *form, void *xev)
{
	switch (((XEvent *)xev)->type) {
		case Expose:
		case ConfigureNotify:
			expose_seen = true;
	}
	return 0;
}


// This is the XForms idle loop.  We hijack this here to do redraws if needed,
// and only pass control onto the "logical" user-defined idle loop if there's
// no need to do a redraw right now.
static int xforms_idle(XEvent *xev, void *data)
{
	// If there are pending X events, don't even bother
	if (XPending(fl_display) > 2)   
		return 0;

	// If we haven't been mapped yet, don't bother
	if (!GUI->screenx() || !GUI->screeny())
		return 0;

	// Try to work around several nasty XForms bugs
	if (expose_seen) {
		fl_redraw_form(GUI->fd_qsplat_gui->qsplat_gui);
		expose_seen = false;
	}
	XAutoRepeatOn(fl_display);

	// If the mouse is pressed, but not in the GL window, abort
	int x, y;
	unsigned m;
	fl_get_mouse(&x, &y, &m);
	if (m & AnyButtonMask && !GUI->dragging())
		return 0;

	// Run the real idle loop if no events so far
	if (!GUI->do_redraw) {
		bool done = GUI->idle();
		// If the idle loop said that refinement was done,
		// wait for the next event
		if (!GUI->do_redraw && done) {
			XEvent ev;
			XPeekEvent(fl_display, &ev);
			return 0;
		}
	}

	// Do a redraw if necessary
	if (GUI->do_redraw) {
		GUI->activate_context();
		GUI->redraw();
		GUI->do_redraw = false;
	}

	return 0;
}


// Exit...
static void Quit()
{
	fl_finish();
	exit(0);
}


// The following are the raw callback functions used by the fdesign-built GUI.
void FileMenuCB(FL_OBJECT *ob, long data)
{
	switch (fl_get_menu(ob)) {
		case 1: {
			const char *filename = fl_show_fselector(NULL, NULL, "*.qs", NULL);
			if (!filename)
				return;
			GUI->SetModel(NULL);
			QSplat_Model *q = QSplat_Model::Open(filename);
			GUI->updatemenus();
			if (!q) {
				fl_show_message("Couldn't open QSplat file", filename, NULL);
				return;
			}
			GUI->SetModel(q);
			GUI->resetviewer();
			GUI->need_redraw();
			return;
		}
		case 2:
			Quit();
	}
}

void OptionsMenuCB(FL_OBJECT *ob, long data)
{
	switch (fl_get_menu(ob)) {
		case 1: {
			bool s = fl_get_menu_item_mode(ob, 1) & FL_PUP_CHECK;
			GUI->set_shiny(s);
			GUI->need_redraw();
			break;
		}
		case 2: {
			bool c = fl_get_menu_item_mode(ob, 2) & FL_PUP_CHECK;
			GUI->set_backfacecull(c);
			GUI->need_redraw();
			break;
		}
		case 3: {
			bool l = fl_get_menu_item_mode(ob, 3) & FL_PUP_CHECK;
			GUI->set_showlight(l);
			GUI->need_redraw();
			break;
		}
		case 4: {
			bool p = fl_get_menu_item_mode(ob, 4) & FL_PUP_CHECK;
			GUI->set_showprogressbar(p);
			GUI->need_redraw();
			break;
		}
		case 5: {
			bool t = !(fl_get_menu_item_mode(ob, 5) & FL_PUP_CHECK);
			GUI->set_touristmode(t);
			GUI->need_redraw();
			break;
		}
	}
}

void DriverMenuCB(FL_OBJECT *ob, long data)
{
	GUI->whichDriver = Driver(fl_get_menu(ob)-1);
	GUI->updatemenus();
	GUI->resetviewer(true);
	GUI->need_redraw();
}

void CommandsMenuCB(FL_OBJECT *ob, long data)
{
	switch (fl_get_menu(ob)) {
		case 1: {
			bool fullscreen = fl_get_menu_item_mode(ob, 1) & FL_PUP_CHECK;
			GUI->fullscreen(fullscreen);
			GUI->need_redraw();
			break;
		}
		case 2: {
			GUI->resetviewer();
			GUI->need_redraw();
			break;
		}
	}
}

void HelpMenuCB(FL_OBJECT *ob, long data)
{
	switch (fl_get_menu(ob)) {
		case 1:
			GUI->aboutQSplat();
			break;
		case 2:
			GUI->aboutmodel();
			break;
	}
}

static bool toggleMenuItem(FL_OBJECT *ob, int numb)
{
	unsigned m = fl_get_menu_item_mode(ob, numb);
	m ^= FL_PUP_CHECK;
	m |= FL_PUP_BOX; // Bug workaround...
	fl_set_menu_item_mode(ob, numb, m);
	return !!(m & FL_PUP_CHECK);
}

void ShortcutCB(FL_OBJECT *ob, long data)
{
	bool yn;
	switch (fl_get_button_numb(ob)) {
		case FL_SHORTCUT+'b':
			yn = toggleMenuItem(GUI->fd_qsplat_gui->OptionsMenu, 2);
			GUI->set_backfacecull(yn);
			GUI->need_redraw();
			break;
		case FL_SHORTCUT+'d':
		case FL_SHORTCUT+'f': {
			yn = toggleMenuItem(GUI->fd_qsplat_gui->CommandsMenu, 1);
			GUI->fullscreen(yn);
			GUI->set_touristmode(yn);
			GUI->need_redraw();
			}
			break;
		case FL_SHORTCUT+'s':
			yn = toggleMenuItem(GUI->fd_qsplat_gui->OptionsMenu, 1);
			GUI->set_shiny(yn);
			GUI->need_redraw();
			break;
		case FL_SHORTCUT+' ':
			GUI->resetviewer();
			GUI->need_redraw();
			break;
		case FL_SHORTCUT+'q':
		case FL_SHORTCUT+'Q':
		case FL_SHORTCUT+17:
		case FL_SHORTCUT+24:
		case FL_SHORTCUT+27:
			Quit();
		default:
			printf("%d %d\n", FL_SHORTCUT, fl_get_button_numb(ob));
	}

}

void DesiredRateCB(FL_OBJECT *ob, long data)
{
	float fps = fl_get_slider_value(ob);
	GUI->set_desiredrate(fps);
	GUI->need_redraw();
}


// Should we abort the rendering?
std::vector<XEvent> QSplatXFormsGUI::events_received;
bool QSplatXFormsGUI::abort_drawing(float time_elapsed)
{
	if (time_elapsed < 1.2f / rate() + 0.1f)
		return false;

	while (XPending(fl_display)) {
		XEvent ev;
		XNextEvent(fl_display, &ev);
		events_received.push_back(ev);
		switch (ev.type) {
			case KeyPress:
			case ButtonPress:
			case Expose:
				return true;
			case MotionNotify:
				if ((((XMotionEvent *) &ev)->state & AnyButtonMask) &&
				    (!theQSplat_Model->coarsest()))
					return true;
				else
					break;
			case ButtonRelease:
				if (!dragging())
					return true;
		}
	}
	return false;
}


// Finish up rendering.  Takes the events abort_drawing pulled off
// the event queue and stuffs them back to be re-digested
void QSplatXFormsGUI::end_drawing(bool bailed)
{
	QSplatGUI::end_drawing(bailed);

	while (!events_received.empty()) {
		XPutBackEvent(fl_display, &events_received.back());
		events_received.pop_back();
	}
}


// Since the XForms shared libraries do not include gl.c on non-SGI systems, we
// include it here...
#ifndef sgi
#include "xforms_gl.c"
#endif

