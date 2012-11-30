#ifndef QSPLAT_GUI_WIN32_H
#define QSPLAT_GUI_WIN32_H
/*
Gary King
Szymon Rusinkiewicz

qsplat_gui_win32.h
The infrastructure for the QSplat GUI.  This version uses the Win32 API.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "qsplat_guimain.h"
#include "resource.h"
#include <queue>


#define APPTITLE "QSplat for Win32"
#define GUI ((QSplatWin32GUI *) theQSplatGUI)


class QSplatWin32GUI : public QSplatGUI {
public:
	int windowPosX, windowPosY, windowBorderY;
	int windowWidth, windowHeight;
	
	//窗口或控件的句柄，用来标识自己
	HWND progress;
	HWND ghWnd;
	//定义一个设备上下文的句柄
	HDC  hDC;
	unsigned int width;
	unsigned int height;

	HGLRC hRC;
	HINSTANCE ghInst;
	bool resizing;
	bool ghWndIsMinimized;
	bool do_redraw;

	static char szTitle[];
	static char szAppName[];
	static char szWindowBuff[256];
	static int statusWidths[2];
	static char lastFileName[256];
	static char buildFileName[256];


	virtual void Go();
	virtual int screenx() { return windowWidth; }
	virtual int screeny() { return windowHeight; }
	virtual int viewportx() { return 0; }
	virtual int viewporty() { return windowBorderY; }
	virtual void need_redraw();
	virtual void swapbuffers();
	virtual void updaterate(const char *text);
	virtual void updatestatus(const char *text);
	virtual void aboutmodel();

	bool menu_shiny, menu_backfacecull, menu_showlight;
	bool menu_showprogress, menu_autospin, menu_fullscreen;
	void updatemenus();

	QSplatWin32GUI(HINSTANCE hInstance,HINSTANCE hPrevInstance,
		       LPSTR lpszCmdLine, int nCmdShow);

	static LONG WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
	static HGLRC SetUpOpenGL(HWND hWnd, HDC hDC);
	static BOOL FormatFilterString(UINT uID, LPSTR lpszString);
	static LRESULT CALLBACK AboutBox(HWND hDlg,UINT message,WPARAM wParam,LPARAM lParam);
	static LRESULT CALLBACK FrameRateSet(HWND hDlg,UINT message,WPARAM wParam, LPARAM lParam);
	static void GetLongName(const char *shortname, char *longname);

	static std::queue<MSG> messageQueue;
	virtual bool abort_drawing(float time_elapsed);
	virtual void end_drawing(bool bailed);
};

#endif
