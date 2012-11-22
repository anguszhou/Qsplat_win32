#ifndef QSPLAT_GUI_XFORMS_H
#define QSPLAT_GUI_XFORMS_H
/*
Szymon Rusinkiewicz

qsplat_gui_xforms.h
The infrastructure for the QSplat GUI.
This version is written for *NIX-like systems using the XForms toolkit.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_guimain.h"
#include <forms.h>
extern "C" {
#include "xforms_gui.h"
}


#define AnyButtonMask (Button1Mask | Button2Mask | Button3Mask | Button4Mask | Button5Mask)
#define GUI ((QSplatXFormsGUI *) theQSplatGUI)


class QSplatXFormsGUI : public QSplatGUI {
public:
	FD_qsplat_gui *fd_qsplat_gui;
	Window mainwindow;
	Window glCanvasWindow;
	const char *window_name;
	int save_window_width, save_window_height;
	int two_button_mouse;
	int canvas_width, canvas_height;
	bool do_redraw;

	virtual void Go();
	virtual int screenx() { return canvas_width; }
	virtual int screeny() { return canvas_height; }
	virtual int viewportx() { return 0; }
	virtual int viewporty() { return 0; }
	virtual void need_redraw() { do_redraw = true; }
	virtual void updaterate(const char *text);
	virtual void updatestatus(const char *text);
	void aboutQSplat();
	virtual void aboutmodel();
	void updatemenus();
	virtual void swapbuffers();

	void usage(const char *myname);
	QSplatXFormsGUI(int *argcp, char *argv[]);
	Window theGLwindow()
	{
		return glCanvasWindow;
	}
	void activate_context()
	{
	 	fl_activate_glcanvas(fd_qsplat_gui->theGLCanvas);
	}
	void fullscreen(bool go_fullscreen);
	void mouseCB(int x, int y, unsigned b);

	static std::vector<XEvent> events_received;
	virtual bool abort_drawing(float time_elapsed);
	virtual void end_drawing(bool bailed);
};

#endif
