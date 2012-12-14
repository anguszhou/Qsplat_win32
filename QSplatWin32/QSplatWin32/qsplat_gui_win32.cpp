/*
Gary King
Szymon Rusinkiewicz

qsplat_gui_win32.cpp
The infrastructure for the QSplat GUI.  This version uses the Win32 API.

  Usage:

  QSplatWin32 <options> <filename.qs>

  Command Line Options:
	-nospin: disable auto-spin
	-x <int>: x-position for window
	-y <int>: y-position for window
	-w <int>: width of window
	-h <int>: height of window
	-framerate <float>: initial desired frame rate


Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_gui_win32.h"
#include <stdio.h>
#include <conio.h>
#include <commdlg.h>
#include <commctrl.h>
#include <winnls.h>
#include <zmouse.h>
#include <Windows.h>
#include <GL/gl.h>
#include "qsplat_make_main.h"


char QSplatWin32GUI::szTitle[] = APPTITLE;
char QSplatWin32GUI::szAppName[] = "QSplat";
char QSplatWin32GUI::szWindowBuff[256] = APPTITLE;
int QSplatWin32GUI::statusWidths[2] = { 400, 623 };
char QSplatWin32GUI::lastFileName[256];
char QSplatWin32GUI::buildFileName[256];

// Parse command-line options (if any) and create the QSplat window
QSplatWin32GUI::QSplatWin32GUI(HINSTANCE hInstance, HINSTANCE hPrevInstance,
			       LPSTR lpszCmdLine, int nCmdShow) : QSplatGUI()
{
  WNDCLASSEX	wc;
  LPSTR		lpFileName = NULL;
  LPSTR		lpX = NULL, lpY = NULL, lpW = NULL, lpH = NULL;
  int           x = CW_USEDEFAULT, y = CW_USEDEFAULT, w = 640, h = 480;
  int           xborder, yborder;

  theQSplatGUI = this;
  resizing = false;
  ghWndIsMinimized = false;
  float framerate = 0.0f;
  menu_shiny = menu_backfacecull = menu_showprogress = menu_autospin = true;
  menu_showlight = menu_fullscreen = false;

  xborder = 2*GetSystemMetrics(SM_CXEDGE)+GetSystemMetrics(SM_CXSIZEFRAME); 
  yborder = 2*GetSystemMetrics(SM_CYBORDER)+2*GetSystemMetrics(SM_CYEDGE)+
            GetSystemMetrics(SM_CYSIZEFRAME)+GetSystemMetrics(SM_CYSIZE)+
            GetSystemMetrics(SM_CYMENUSIZE);

  wc.cbSize = sizeof(wc);
  wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
  wc.lpfnWndProc = (WNDPROC)WndProc;
  wc.cbClsExtra = 0;
  wc.cbWndExtra = 0;
  wc.hInstance = hInstance;
  wc.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(QSPLAT_ICON));
  wc.hCursor = LoadCursor(NULL, IDC_ARROW);
  wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
  wc.lpszMenuName = MAKEINTRESOURCE(IDR_MENU1);
  wc.lpszClassName = szAppName;
  wc.hIconSm = LoadIcon(hInstance, MAKEINTRESOURCE(QSPLAT_ICON));

  if (!(RegisterClassEx(&wc)))
  {
    MessageBox(NULL,"Can't register window classes","QSplat", 
	       MB_OK | MB_ICONHAND);
    exit(1);
  }

  INITCOMMONCONTROLSEX icx;
  icx.dwSize = sizeof(icx);
  icx.dwICC = ICC_BAR_CLASSES;
  
  InitCommonControlsEx(&icx);

  ghInst = hInstance;
  char *token, *nextToken;
  nextToken = strtok(lpszCmdLine," ");
  while (nextToken)
  {
    token = nextToken;
    nextToken = strtok(NULL," ");
    if (stricmp(token,"-nospin")==0)
    {
	set_touristmode(true);
	menu_autospin = false;
    }
    else if (stricmp(token,"-x")==0) 
    { 
      x = atoi(nextToken);
      nextToken = strtok(NULL," "); 
    }
    else if (stricmp(token,"-y")==0) 
    { 
      y = atoi(nextToken);
      nextToken = strtok(NULL," "); 
    }
    else if (stricmp(token,"-w")==0) 
    { 
      w = atoi(nextToken);
      nextToken = strtok(NULL," "); 
    }
    else if (stricmp(token,"-h")==0) 
    { 
      h = atoi(nextToken);
      nextToken = strtok(NULL," "); 
    }
    else if (stricmp(token,"-framerate")==0) 
    { 
      framerate = atof(nextToken);
      nextToken = strtok(NULL," "); 
    }
    
    else if ((stricmp(token,"-help")==0) || (stricmp(token,"-?")==0))
    {
      MessageBox(NULL,"QSplat Help (Command Line Options)\n\
                  \t-nospin:Disables Auto-spin\n\
                  \t-help,-?: Display this screen\n\
                  \t-framerate <float>:set initial framerate\n\
                  \t-x, -y, -w, -h <int>: Specify initial window dimensions",
		 "QSplat Command Line Help",MB_OK);
      exit(0);
    }
    
    else lpFileName = token;
  }

  ghWnd = CreateWindow(szAppName,szTitle,WS_OVERLAPPEDWINDOW,
		       x,y,w+xborder,h+yborder,NULL,NULL,hInstance,NULL);

  if (!ghWnd || !progress)
  {
    MessageBox(NULL,"Can't create window.","QSplat",MB_OK | MB_ICONHAND);
    exit(1);
  }

  ShowWindow(ghWnd, nCmdShow);
  UpdateWindow(ghWnd);
  
  whichDriver = strncmp((char *)glGetString(GL_VENDOR), "Microsoft", 9) ? OPENGL_POINTS : SOFTWARE_BEST;

  if (lpFileName)
  {
    GetLongName(lpFileName, lastFileName);
    QSplat_Model *q = QSplat_Model::Open(lastFileName);
    if (q) {
      SetModel(q);
      char* filename = lastFileName;
      char* resVal;
      if (resVal = (strrchr(filename,'\\'))) {filename=resVal+1; }
      sprintf(szWindowBuff,"%s - %s",APPTITLE,filename);
      SetWindowText(ghWnd, szWindowBuff);
    }
  }
  else
  {
    GetCurrentDirectory(sizeof(lastFileName),lastFileName);
	GetCurrentDirectory(sizeof(buildFileName),buildFileName);
    strcat(lastFileName,"\\*.qs");
	strcat(buildFileName,"\\*.ply");
  }
  if (framerate == 0.0f)
    framerate = ((int)whichDriver >= (int)SOFTWARE_GLDRAWPIXELS) ? 4.0f : 8.0f;
  set_desiredrate(framerate);
  updatemenus();
}


// Main loop
void QSplatWin32GUI::Go()
{
  MSG msg;

  while (TRUE) 
  {
    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) 
    {
      if (msg.message == WM_QUIT) break;
      TranslateMessage(&msg);
      DispatchMessage(&msg);
      continue;
    }

    if (!do_redraw)
    {
      bool done = idle();
      if (ghWndIsMinimized || (!do_redraw && done))
      {
	GetMessage(&msg, NULL, 0, 0);
	if (msg.message == WM_QUIT) break;
	TranslateMessage(&msg);
	DispatchMessage(&msg);
      }
    }
    
    if (do_redraw)
    {
      wglMakeCurrent(GUI->hDC, GUI->hRC);
      redraw();
      do_redraw = false;
    }
  }
  SetModel(NULL);
}


/****************************************************
 *  WndProc().  Processes all messages.             *
 ****************************************************/

LONG WINAPI QSplatWin32GUI::WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) 
{
  switch(msg) 
  {
    
  case WM_CREATE:
  {
    /*  Generate the status bar, 2 panes of a given width  */
    GUI->progress = CreateWindow(STATUSCLASSNAME,APPTITLE, WS_CHILD | WS_VISIBLE,0,0,0,0,hWnd, NULL, GUI->ghInst, NULL);
    int borders[3];
    RECT pbR;
    SendMessage(GUI->progress,SB_GETBORDERS,0,(WPARAM)borders);
    SendMessage(GUI->progress,SB_GETRECT,0,(LPARAM)&pbR);
    statusWidths[1] = pbR.right;
    SendMessage(GUI->progress,SB_SETPARTS,2,(LPARAM)statusWidths);
    GUI->windowBorderY = pbR.bottom-pbR.top+borders[1];
    /*  Initialize OpenGL context  */
    GUI->hDC = GetDC(hWnd);
    GUI->hRC = SetUpOpenGL(hWnd, GUI->hDC);
    SetBkMode(GUI->hDC,TRANSPARENT);
    SetTextColor(GUI->hDC, RGB(255,255,255));
    SetBkColor(GUI->hDC, RGB(0,0,0));
    wglMakeCurrent(GUI->hDC, GUI->hRC);
    char progbarName[128];
    sprintf(progbarName,"%s",APPTITLE);
    SendMessage(GUI->progress,SB_SETTEXT,0,(LPARAM)progbarName);
    SetWindowText(hWnd,szWindowBuff);

    return 0;
  }
  case WM_MOVE:
  {
    GUI->windowPosX = (int) LOWORD(lParam);
    GUI->windowPosY = (int) HIWORD(lParam);
    return 0;
  }
  case WM_SIZE:
  {
    if ((UINT)wParam==SIZE_MINIMIZED) GUI->ghWndIsMinimized=true;
    else
    {
      GUI->ghWndIsMinimized=false;
      int borders[3];
      RECT pbR;
      SendMessage(GUI->progress,SB_GETBORDERS,0,(LPARAM)borders);
      SendMessage(GUI->progress,SB_GETRECT,0,(LPARAM)&pbR);
      MoveWindow(GUI->progress, 0, HIWORD(lParam) - 10, LOWORD(lParam), 
		 HIWORD(lParam), TRUE);
      if (LOWORD(lParam) <= 400) 
      {
	statusWidths[0]=LOWORD(lParam)/2; statusWidths[1]=LOWORD(lParam); 
	SendMessage(GUI->progress,SB_SETPARTS,2,(LPARAM)statusWidths); 
      }
      else 
      {
	statusWidths[0]=400; statusWidths[1]=LOWORD(lParam); 
	SendMessage(GUI->progress,SB_SETPARTS,2,(LPARAM)statusWidths); 
      }
      GUI->windowBorderY = pbR.bottom - pbR.top + borders[1];
      GUI->windowWidth = (LOWORD(lParam));
      GUI->windowHeight = (HIWORD(lParam)) - GUI->windowBorderY;
      GUI->resetviewer(true);
      GUI->need_redraw();
    }

    return 0;
  }

  case WM_MOUSEACTIVATE:
  {
    GUI->resetviewer(true);
    GUI->need_redraw();
    return 0;
  }
  
  case WM_LBUTTONDOWN: 
  case WM_MBUTTONDOWN:
  case WM_RBUTTONDOWN:
  case WM_LBUTTONUP:
  case WM_MBUTTONUP:
  case WM_RBUTTONUP:
  case WM_MOUSEMOVE:
  {
    POINT mousepos;
    static mousebutton this_button = NO_BUTTON;

    if (!wParam) 
    { 
      if (this_button!=NO_BUTTON) ReleaseCapture(); 
      this_button = NO_BUTTON; 
    }
    else if (wParam&(MK_LBUTTON|MK_MBUTTON|MK_RBUTTON)&&
             wParam&(MK_CONTROL|MK_SHIFT)) this_button = LIGHT_BUTTON;
    else if (wParam&MK_LBUTTON&&wParam&MK_RBUTTON) this_button = TRANSZ_BUTTON;
    else if (wParam&MK_RBUTTON&&wParam&MK_MBUTTON) this_button = LIGHT_BUTTON;
    else if (wParam & MK_LBUTTON) this_button = ROT_BUTTON;
    else if (wParam & MK_MBUTTON) this_button = TRANSZ_BUTTON;
    else if (wParam & MK_RBUTTON) this_button = TRANSXY_BUTTON;
    else this_button = NO_BUTTON;

    if (this_button!=NO_BUTTON) SetCapture(GUI->ghWnd);
    GetCursorPos(&mousepos);
    
    GUI->mouse(mousepos.x - GUI->windowPosX,
	       (GUI->windowHeight - GUI->windowBorderY) -
				(mousepos.y - GUI->windowPosY),
	       this_button);
	return 0;
  }
  case WM_MOUSEWHEEL:
  {
    POINT mousepos;
    GetCursorPos(&mousepos);
    GUI->mouse(mousepos.x - GUI->windowPosX,
	       (GUI->windowHeight - GUI->windowBorderY) -
				(mousepos.y - GUI->windowPosY),
	       (signed short)HIWORD(wParam) > 0 ? UP_WHEEL : DOWN_WHEEL);
    return 0;
  }
  case WM_KEYDOWN:
  {
    switch ((int)wParam) {
      case 'q': case 'Q': case 'x': case 'X':
      case 27: // Esc
	PostQuitMessage(0);
	break;
      case ' ':
	GUI->resetviewer();
	GUI->need_redraw();
	break;
    }
    return 0;
  }
  case WM_PAINT:
  {
    PAINTSTRUCT ps;
    RECT updateRect;
    if (!GetUpdateRect(hWnd,&updateRect,false)) return 0;
    if (!BeginPaint(hWnd,&ps)) return 0;
    GUI->resetviewer(true);
    GUI->need_redraw();
    EndPaint(hWnd, &ps);
    return 0;
  }

  case WM_DESTROY:
  {
    ReleaseCapture();
    wglDeleteContext(GUI->hRC);
    ReleaseDC(hWnd, GUI->hDC);
    if (!GUI->resizing) PostQuitMessage(0);
    return 0;
  }
	
  case WM_COMMAND:
    switch(wParam)
    {
    case QSPLAT_SET_FRAMERATE:
    {
      DialogBox(GUI->ghInst, MAKEINTRESOURCE(QSPLAT_FRAMERATE_CONTROL), 
		hWnd, (DLGPROC) FrameRateSet);
      return 0;
    }

    case QSPLAT_ABOUT:
    {
      //DialogBox(GUI->ghInst, MAKEINTRESOURCE(QSPLAT_ABOUT_DIALOG),
      //		hWnd, (DLGPROC) AboutBox);
      return 0;
    }

    case QSPLAT_ABOUT_MODEL:
    {
		//GUI->aboutmodel();
		return 0;
    }
    
    case QSPLAT_EXIT:
    {
      PostQuitMessage(WM_DESTROY);
      return 0;
    }
	
	case QSPLAT_BUILD:
	{
		OPENFILENAME ofn;
      
      static char szFilter[260];
      FormatFilterString(IDS_POINTFILTER, szFilter);
      
      ofn.lStructSize = sizeof(OPENFILENAME);
      ofn.hwndOwner = hWnd;
      ofn.hInstance = NULL;
      ofn.lpstrFilter = szFilter;
      ofn.lpstrCustomFilter = NULL;
      ofn.nMaxCustFilter = 0;
      ofn.nFilterIndex = 0;
      ofn.lpstrFile = buildFileName;
      ofn.nMaxFile = 260;
      ofn.lpstrFileTitle = NULL;
      ofn.nMaxFileTitle = 0;
      ofn.lpstrInitialDir = NULL;
      ofn.lpstrTitle = "Build QS from PLY..";
      ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
      ofn.nFileOffset = 0;
      ofn.nFileExtension = 0;
      ofn.lpstrDefExt = NULL;
      ofn.lCustData = 0;
      ofn.lpfnHook = NULL;
      ofn.lpTemplateName = NULL;
      
      if (!GetOpenFileName(&ofn)) return 0;
	  
	  char* param = "point";
	  int result = build_ply_to_qs(param,buildFileName);
	  if(result)
	  {
		  char buf[256];
		  sprintf(buf, "Build Qsplat file successful.");
		  MessageBox(NULL, buf, NULL, MB_OK);
	  }
	  else
	  {
		  char buf[256];
		  sprintf(buf, "Build Qsplat file failed.");
		  MessageBox(NULL, buf, NULL, MB_OK);
	  }	


	  return 0;
	}

    case QSPLAT_OPEN:
    {
      OPENFILENAME ofn;
      
      static char szFilter[260];
      FormatFilterString(IDS_FILTERSTRING, szFilter);
      
      ofn.lStructSize = sizeof(OPENFILENAME);
      ofn.hwndOwner = hWnd;
      ofn.hInstance = NULL;
      ofn.lpstrFilter = szFilter;
      ofn.lpstrCustomFilter = NULL;
      ofn.nMaxCustFilter = 0;
      ofn.nFilterIndex = 0;
      ofn.lpstrFile = lastFileName;
      ofn.nMaxFile = 260;
      ofn.lpstrFileTitle = NULL;
      ofn.nMaxFileTitle = 0;
      ofn.lpstrInitialDir = NULL;
      ofn.lpstrTitle = "Open QS";
      ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
      ofn.nFileOffset = 0;
      ofn.nFileExtension = 0;
      ofn.lpstrDefExt = NULL;
      ofn.lCustData = 0;
      ofn.lpfnHook = NULL;
      ofn.lpTemplateName = NULL;
      
      if (!GetOpenFileName(&ofn)) return 0;

      GUI->SetModel(NULL);
      QSplat_Model *q = QSplat_Model::Open(lastFileName);
      if (!q)
      {
	  char buff[256];
	  sprintf(buff, "Can't open %s!.",lastFileName);
	  MessageBox(NULL, buff, szAppName, MB_OK | MB_ICONEXCLAMATION);
      }
      else
      {
	  char* filename = lastFileName;
	  char* resVal;
	  //char* strrchr(char*str,char c):search position of last time of c
	  //appear in str and return substring of str from the position to end 
	  //if success else return null.
	  if (resVal = (strrchr(filename,'\\'))) {filename=resVal+1; }
	  sprintf(szWindowBuff, "%s - %s",APPTITLE, filename);
	  SetWindowText(hWnd,szWindowBuff);

	  GUI->SetModel(q);
	  GUI->resetviewer();
	  GUI->need_redraw();
      }

      GUI->updatemenus();
      return 0;
    }

    case QSPLAT_DRIVERS_OPENGL_POINTS:
    case QSPLAT_DRIVERS_OPENGL_CIRCLES:
    case QSPLAT_DRIVERS_OPENGL_QUADS:
    case QSPLAT_DRIVERS_OPENGL_POLY_CIRC:
    case QSPLAT_DRIVERS_OPENGL_POLY_ELLIPSES:
    case QSPLAT_DRIVERS_OPENGL_POLY_ELLIPSE_SMALL:
    case QSPLAT_DRIVERS_OPENGL_SPHERES:
    case QSPLAT_DRIVERS_SOFTWARE_GLDRAWPIXELS:
    case QSPLAT_DRIVERS_SOFTWARE:
    case QSPLAT_DRIVERS_SOFTWARE_TILES_GLDRAWPIXELS:
    case QSPLAT_DRIVERS_SOFTWARE_TILES:
    case QSPLAT_DRIVERS_SOFTWARE_BEST_GLDRAWPIXELS:
    case QSPLAT_DRIVERS_SOFTWARE_BEST:
    {
      GUI->whichDriver = (Driver) (wParam - 50000);  // #defines in resource.h are numbered 50000..50010 in the same order as the Driver enum
      GUI->resetviewer(true);
      GUI->need_redraw();
      GUI->updatemenus();
      return 0;
    }
    
    case QSPLAT_OPTIONS_SHOWLIGHT:
    {
      GUI->menu_showlight = !GUI->menu_showlight;
      GUI->set_showlight(GUI->menu_showlight);
      GUI->need_redraw();
      GUI->updatemenus();
      return 0;
    }

    case QSPLAT_OPTIONS_SHINY:
    {
      GUI->menu_shiny = !GUI->menu_shiny;
      GUI->set_shiny(GUI->menu_shiny);
      GUI->need_redraw();
      GUI->updatemenus();
      return 0;
    }
	
    case QSPLAT_OPTIONS_BACKFACECULL:
    {
      GUI->menu_backfacecull = !GUI->menu_backfacecull;
      GUI->set_backfacecull(GUI->menu_backfacecull);
      GUI->need_redraw();
      GUI->updatemenus();
      return 0;
    }

    case QSPLAT_OPTIONS_SHOWPROGRESS:
    {
      GUI->menu_showprogress = !GUI->menu_showprogress;
      GUI->set_showprogressbar(GUI->menu_showprogress);
      GUI->need_redraw();
      GUI->updatemenus();
      return 0;
    }

    case QSPLAT_OPTIONS_AUTOSPIN:
    {
      GUI->menu_autospin = !GUI->menu_autospin;
      GUI->set_touristmode(!GUI->menu_autospin);
      GUI->need_redraw();
      GUI->updatemenus();
      return 0;
    }

    case QSPLAT_COMMANDS_NORMSCREEN:
    {
      GUI->resizing = true;
      GUI->menu_fullscreen = false;
      wglDeleteContext(GUI->hRC);
      ReleaseDC(hWnd, GUI->hDC);
      DestroyWindow(GUI->progress);
      DestroyWindow(GUI->ghWnd);
      GUI->ghWnd = CreateWindow(szAppName, szTitle, 
				 WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 
				 CW_USEDEFAULT, 640, 480, NULL, NULL, 
				 GUI->ghInst, NULL);
      if (!(GUI->ghWnd && GUI->progress))
      {
	char buff[64];
	sprintf(buff,"Can't resize window!: %d",GetLastError());
	MessageBox(NULL, buff, NULL, MB_OK);
	PostQuitMessage(0);
      }
      ShowWindow(GUI->ghWnd, SW_SHOW);
      GUI->resetviewer(true);
      GUI->need_redraw();
      GUI->resizing = false;
      GUI->updatemenus();
      return 0;
    }
    
    case QSPLAT_COMMANDS_FULLSCREEN:
    {
      GUI->resizing = true;
      GUI->menu_fullscreen = true;
      wglDeleteContext(GUI->hRC);
      ReleaseDC(hWnd, GUI->hDC);
      DestroyWindow(GUI->progress);
      DestroyWindow(GUI->ghWnd);
      GUI->ghWnd = CreateWindow(szAppName, szTitle,
				 WS_POPUP | WS_MAXIMIZE | WS_CLIPCHILDREN
				 | WS_CLIPSIBLINGS,CW_USEDEFAULT, 
				 CW_USEDEFAULT, 640, 480,NULL, NULL, 
				 GUI->ghInst, NULL);
      if (!(GUI->ghWnd && GUI->progress))
      {
	char buff[64];
	sprintf(buff,"Can't go Fullscreen: %d",GetLastError());
	MessageBox(NULL, buff, NULL, MB_OK);
	PostQuitMessage(0);
      }
      ShowWindow(GUI->ghWnd, SW_SHOWMAXIMIZED);
      GUI->resetviewer(true);
      GUI->need_redraw();
      GUI->resizing = false;
      GUI->updatemenus();
      return 0;
    }

    case QSPLAT_COMMANDS_RESET:
    {
      GUI->resetviewer();
      GUI->need_redraw();
      return 0;
    }
    default:
      return DefWindowProc(hWnd,msg,wParam,lParam);
    }
  }
  return DefWindowProc(hWnd, msg, wParam, lParam);	
}


/*******************************************
 * SetUpOpenGL().  Windows specific OpenGL *
 * initialization.                         *
 *******************************************/

HGLRC QSplatWin32GUI::SetUpOpenGL(HWND hWnd, HDC hDC) 
{
  static PIXELFORMATDESCRIPTOR pfd = 
  { 
    sizeof(PIXELFORMATDESCRIPTOR), 1, PFD_DRAW_TO_WINDOW | 
    PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
    PFD_TYPE_RGBA, 24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0,0 
  };

  int nMyPixelFormatID;
  HGLRC hRC;
  
  nMyPixelFormatID = ChoosePixelFormat(hDC, &pfd);
  SetPixelFormat(hDC, nMyPixelFormatID, &pfd);
  hRC = wglCreateContext(hDC);
  if (!hRC)
  {
    char buff[256];
    sprintf(buff,"Can't create render context: %d",GetLastError());
    MessageBox(NULL,buff,"QSplat",MB_OK | MB_ICONHAND);
  }
  return hRC;
}


// Print out frame rate statistics
void QSplatWin32GUI::updaterate(const char *message)
{
  SendMessage(progress,SB_SETTEXT,0,(LPARAM)message);
}


// Print out what QSplat thinks it's doing
void QSplatWin32GUI::updatestatus(const char *message)
{
  SendMessage(progress,SB_SETTEXT,1,(LPARAM)message);
}


// Pop up a box with information about the current mesh
void QSplatWin32GUI::aboutmodel()
{
  if (theQSplat_Model)
    MessageBox(NULL,theQSplat_Model->comments.c_str(),"About this Model",MB_OK);
  else
    MessageBox(NULL,"No model loaded","About this Model",MB_OK);
}

//
BOOL QSplatWin32GUI::FormatFilterString(UINT uID, LPSTR lpszString)
{
  int nLength, nCtr = 0;
  char chWildCard;
  
  *lpszString = 0;
  
  nLength = LoadString( GUI->ghInst, uID, lpszString, 260 );
  
  chWildCard = lpszString[nLength-1];
  
  while( lpszString[nCtr] ) 
  {
    if( lpszString[nCtr] == chWildCard ) lpszString[nCtr] = 0;
    nCtr++;
  }
  
  return TRUE;
}


/***************************
 Win32 GUI stuff
 ***************************/

LRESULT CALLBACK QSplatWin32GUI::AboutBox(HWND hDlg,UINT message,WPARAM wParam,LPARAM lParam)
{
  switch (message)
  {
  case WM_INITDIALOG: return TRUE;
  case WM_COMMAND:
  {
    switch (wParam)
    {
    case IDOK:
    case IDCANCEL:
    {
      EndDialog(hDlg, TRUE);
      return TRUE;
    }
    default: return FALSE;
    }
  }
  default: return FALSE;
  }
}

LRESULT CALLBACK QSplatWin32GUI::FrameRateSet(HWND hDlg,UINT message,
					      WPARAM wParam, LPARAM lParam)
{
  static HWND hTrack;
  static int val;
  static char frameratetxt[5];
  switch (message)
  {
  case WM_INITDIALOG:
  {
    hTrack = GetDlgItem(hDlg,FRAMERATE_SLIDER);
    SendMessage(hTrack,TBM_SETRANGEMIN,0,2);
    SendMessage(hTrack,TBM_SETRANGEMAX,0,40);
    SendMessage(hTrack,TBM_SETPOS,1,val = int(2.0f*GUI->rate()));
    sprintf(frameratetxt,"%.1f", 0.5f*val);
    SetDlgItemText(hDlg,FRAMERATE_TEXT,frameratetxt);
    return TRUE;
  }
  return TRUE;
  case WM_HSCROLL:
  {
    val = SendMessage(hTrack,TBM_GETPOS, 0, 0);
    sprintf(frameratetxt,"%.1f\0", 0.5f*val);
    SetDlgItemText(hDlg,FRAMERATE_TEXT,frameratetxt);
    return TRUE;
  }
  case WM_COMMAND:
  {
    switch (wParam)
    {
    case IDOK:
    {
      GUI->set_desiredrate(0.5f * val);
      EndDialog(hDlg, TRUE);
      return TRUE;
    }
    default: return FALSE;
    }
  }
  default: return FALSE;
  }
}

// Update the state of the menus so that they reflect reality (or our
// perception thereof, at least)
void QSplatWin32GUI::updatemenus()
{
  HMENU hOptionsMenu = GetSubMenu(GetMenu(ghWnd),1);
  HMENU hDriverMenu = GetSubMenu(GetMenu(ghWnd),2);
  HMENU hOpenGLMenu = GetSubMenu(hDriverMenu,0);
  HMENU hSoftwareMenu = GetSubMenu(hDriverMenu,1);
  HMENU hVisMenu = GetSubMenu(hDriverMenu,2);
  HMENU hCommandMenu = GetSubMenu(GetMenu(ghWnd),3);
  HMENU hHelpMenu = GetSubMenu(GetMenu(ghWnd),4);

  CheckMenuItem(hOptionsMenu,QSPLAT_OPTIONS_SHINY,MF_BYCOMMAND|(menu_shiny?MF_CHECKED:MF_UNCHECKED));
  CheckMenuItem(hOptionsMenu,QSPLAT_OPTIONS_BACKFACECULL,MF_BYCOMMAND|(menu_backfacecull?MF_CHECKED:MF_UNCHECKED));
  CheckMenuItem(hOptionsMenu,QSPLAT_OPTIONS_SHOWLIGHT,MF_BYCOMMAND|(menu_showlight?MF_CHECKED:MF_UNCHECKED));
  CheckMenuItem(hOptionsMenu,QSPLAT_OPTIONS_SHOWPROGRESS,MF_BYCOMMAND|(menu_showprogress?MF_CHECKED:MF_UNCHECKED));
  CheckMenuItem(hOptionsMenu,QSPLAT_OPTIONS_AUTOSPIN,MF_BYCOMMAND|(menu_autospin?MF_CHECKED:MF_UNCHECKED));
  CheckMenuItem(hOpenGLMenu,QSPLAT_DRIVERS_OPENGL_POINTS,MF_BYCOMMAND|(whichDriver==OPENGL_POINTS)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hOpenGLMenu,QSPLAT_DRIVERS_OPENGL_CIRCLES,MF_BYCOMMAND|(whichDriver==OPENGL_POINTS_CIRC)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hOpenGLMenu,QSPLAT_DRIVERS_OPENGL_QUADS,MF_BYCOMMAND|(whichDriver==OPENGL_QUADS)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hOpenGLMenu,QSPLAT_DRIVERS_OPENGL_POLY_CIRC,MF_BYCOMMAND|(whichDriver==OPENGL_POLYS_CIRC)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hOpenGLMenu,QSPLAT_DRIVERS_OPENGL_POLY_ELLIPSES,MF_BYCOMMAND|(whichDriver==OPENGL_POLYS_ELLIP)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hSoftwareMenu,QSPLAT_DRIVERS_SOFTWARE_GLDRAWPIXELS,MF_BYCOMMAND|(whichDriver==SOFTWARE_GLDRAWPIXELS)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hSoftwareMenu,QSPLAT_DRIVERS_SOFTWARE,MF_BYCOMMAND|(whichDriver==SOFTWARE)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hSoftwareMenu,QSPLAT_DRIVERS_SOFTWARE_TILES_GLDRAWPIXELS,MF_BYCOMMAND|(whichDriver==SOFTWARE_TILES_GLDRAWPIXELS)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hSoftwareMenu,QSPLAT_DRIVERS_SOFTWARE_TILES,MF_BYCOMMAND|(whichDriver==SOFTWARE_TILES)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hSoftwareMenu,QSPLAT_DRIVERS_SOFTWARE_BEST_GLDRAWPIXELS,MF_BYCOMMAND|(whichDriver==SOFTWARE_BEST_GLDRAWPIXELS)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hSoftwareMenu,QSPLAT_DRIVERS_SOFTWARE_BEST,MF_BYCOMMAND|(whichDriver==SOFTWARE_BEST)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hVisMenu,QSPLAT_DRIVERS_OPENGL_POLY_ELLIPSE_SMALL,MF_BYCOMMAND|(whichDriver==OPENGL_POLYS_ELLIP_SMALL)?MF_CHECKED:MF_UNCHECKED);
  CheckMenuItem(hVisMenu,QSPLAT_DRIVERS_OPENGL_SPHERES,MF_BYCOMMAND|(whichDriver==OPENGL_SPHERES)?MF_CHECKED:MF_UNCHECKED);

  if (menu_fullscreen) {
    static const MENUITEMINFO windowMode = 
    { 
      sizeof(MENUITEMINFO), MIIM_DATA | MIIM_ID | MIIM_TYPE, 
      MFT_STRING, MFS_ENABLED, QSPLAT_COMMANDS_NORMSCREEN,
      NULL, NULL, NULL, QSPLAT_COMMANDS_NORMSCREEN, "&Window Mode", 11 
    };

    SetMenuItemInfo(hCommandMenu,QSPLAT_COMMANDS_FULLSCREEN,false,&windowMode);
  }

  bool softwaredriver = ((int)whichDriver >= (int)SOFTWARE_GLDRAWPIXELS);
  unsigned s = softwaredriver?MF_GRAYED:MF_ENABLED;
  EnableMenuItem(hOptionsMenu,QSPLAT_OPTIONS_SHINY,MF_BYCOMMAND|s);

  bool nonopengldriver = (whichDriver == SOFTWARE ||
			  whichDriver == SOFTWARE_TILES ||
			  whichDriver == SOFTWARE_BEST);
  unsigned g = nonopengldriver?MF_GRAYED:MF_ENABLED;
  EnableMenuItem(hOptionsMenu,QSPLAT_OPTIONS_SHOWLIGHT,MF_BYCOMMAND|g);
  EnableMenuItem(hOptionsMenu,QSPLAT_OPTIONS_SHOWPROGRESS,MF_BYCOMMAND|g);

  bool nomodel = !theQSplat_Model;
  unsigned n = nomodel?MF_GRAYED:MF_ENABLED;
  EnableMenuItem(hCommandMenu,QSPLAT_COMMANDS_RESET,MF_BYCOMMAND|n);
  EnableMenuItem(hHelpMenu,QSPLAT_ABOUT_MODEL,MF_BYCOMMAND|n);
}

void QSplatWin32GUI::need_redraw()
{
  do_redraw = true;
}


// Should we abort the rendering?
std::queue<MSG> QSplatWin32GUI::messageQueue;
bool QSplatWin32GUI::abort_drawing(float time_elapsed)
{
  if (time_elapsed < 1.2f / rate() + 0.1f) return false;

  MSG msg;
  while (PeekMessage(&msg,GUI->ghWnd,0,0,PM_REMOVE))
  {
    if (msg.hwnd!=GUI->ghWnd && msg.message==WM_PAINT)
    {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
    else
    {
      messageQueue.push(msg);
      switch (msg.message)
      {
        case WM_ERASEBKGND:
        case WM_NCLBUTTONDOWN:
        case WM_NCRBUTTONDOWN:
        case WM_NCMBUTTONDOWN:
        case WM_MBUTTONDOWN:
        case WM_RBUTTONDOWN:
        case WM_LBUTTONDOWN: 
        case WM_PAINT:
	  return true;
        case WM_MOUSEMOVE: 
        {
	  if ((msg.wParam&MK_LBUTTON || msg.wParam&MK_RBUTTON || 
	       msg.wParam&MK_MBUTTON) && (!theQSplat_Model->coarsest()))
	    return true; 
        }
      }
    }
  }

  return false;
}


// Finish up rendering.  Takes the messages abort_drawing grabbed
// and re-posts them
void QSplatWin32GUI::end_drawing(bool bailed)
{
  QSplatGUI::end_drawing(bailed);

  while (!messageQueue.empty())
  {
    MSG &msg = messageQueue.front();
    PostMessage(msg.hwnd, msg.message, msg.wParam, msg.lParam);
    messageQueue.pop();
  }
}

void QSplatWin32GUI::swapbuffers()
{
	SwapBuffers(hDC);
}


void QSplatWin32GUI::GetLongName(const char *shortname, char *longname)
{
  strcpy(longname, shortname);

  WIN32_FIND_DATA filedata;
  if (FindFirstFile(longname, &filedata) == INVALID_HANDLE_VALUE)
    return;

  char *c = strrchr(longname,'\\');
  if (!c)
    strcpy(longname, filedata.cFileName);
  else
    strcpy(c+1, filedata.cFileName);
}

