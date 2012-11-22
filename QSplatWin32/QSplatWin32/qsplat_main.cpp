/*
Szymon Rusinkiewicz

qsplat_main.cpp
The main() routine for the QSplat viewer.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/


#ifdef WIN32
# include "qsplat_gui_win32.h"
#else
# include "qsplat_gui_xforms.h"
#endif


// Version stamp
const char *QSPLAT_VERSION = "1.0";


#ifdef WIN32

int WINAPI WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,
		   LPSTR lpszCmdLine, int nCmdShow) 
{
	theQSplatGUI = new QSplatWin32GUI(hInstance, hPrevInstance,
					  lpszCmdLine, nCmdShow);
	theQSplatGUI->Go();
	return 0;
}

#else

int main(int argc, char *argv[])
{
	theQSplatGUI = new QSplatXFormsGUI(&argc, argv);
	theQSplatGUI->Go();
}

#endif

