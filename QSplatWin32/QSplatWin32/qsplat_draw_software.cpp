/*
Szymon Rusinkiewicz

qsplat_draw_software.cpp
Draw splats manually into an off-screen FB, performing Z-buffering in software.
Then blit the whole thing onto the screen using whatever means available.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_guimain.h"
#include <Windows.h>
#include <GL/gl.h>
#include <float.h>

#ifdef WIN32
# include "qsplat_gui_win32.h"
#else
# include "qsplat_gui_xforms.h"
# include "xshm.h"
#endif


// Local variables
static float cameramatrix[16];
static float lightdir_mesh[3];
static int width, height, depth;
static int rshift, gshift, bshift;
static float rmult, gmult, bmult;
static bool flipY;
static bool use_gldrawpixels;
static unsigned char *framebuffer;
static float *depthbuffer;

#ifndef WIN32
  static XImage *ximg;
#endif
static bool xshm_error = false;


#define RGB2BITS(r,g,b) ((unsigned((r)*rmult) << rshift) | \
			 (unsigned((g)*gmult) << gshift) | \
			 (unsigned((b)*bmult) << bshift))

// Little function called right before we start drawing
void start_drawing_software(bool _use_gldrawpixels)
{
	use_gldrawpixels = _use_gldrawpixels;

	// Read back OpenGL matrices
	float P[16], M[16], V[4];
	glGetFloatv(GL_PROJECTION_MATRIX, P);
	glGetFloatv(GL_MODELVIEW_MATRIX, M);
	glGetFloatv(GL_VIEWPORT, V);
	width = int(V[2]);  height = int(V[3]);
	V[0] = 0;  V[1] = 0; // Get rid of effects of glViewport(x,y,...)
	FastProjectPrecompute(P, M, V, cameramatrix);

	// Figure out light relative to mesh. Assumes directional light...
	float lightdir[4];
	glGetLightfv(GL_LIGHT0, GL_POSITION, lightdir);
	lightdir_mesh[0] = lightdir[0]*M[0]+lightdir[1]*M[1]+lightdir[2]*M[2];
	lightdir_mesh[1] = lightdir[0]*M[4]+lightdir[1]*M[5]+lightdir[2]*M[6];
	lightdir_mesh[2] = lightdir[0]*M[8]+lightdir[1]*M[9]+lightdir[2]*M[10];


	framebuffer = NULL;
	flipY = false;
#ifdef WE_ARE_LITTLE_ENDIAN
	rshift = 0; gshift = 8; bshift = 16;
#else
	rshift = 24; gshift = 16; bshift = 8;
#endif
	rmult = gmult = bmult = 255.99f;

	if (!use_gldrawpixels) {
#ifdef WIN32
		rshift = 16; gshift = 8; bshift = 0;
#else
		// Test if XSHM available
		if (fl_state[fl_vmode].depth >= 15 &&
		    fl_state[fl_vmode].depth <= 32 &&
		    check_for_xshm(fl_display)) {
			// Create XSHM XImage
			if ((fl_state[fl_vmode].depth <= 16) && (width & 1))
				width++;
			ximg = alloc_xshm_image(fl_display,
						fl_state[fl_vmode].xvinfo->visual,
						width, height,
						fl_state[fl_vmode].depth);
			if (ximg) {
				depth = ximg->bits_per_pixel / 8;
				if (depth != 2 && depth != 4) {
					fprintf(stderr, "Packed 24bpp mode not supported\n");
					destroy_xshm_image(fl_display, ximg);
					ximg = NULL;
				} else {
					framebuffer = (unsigned char *)ximg->data;
					flipY = true;
					rshift = fl_state[fl_vmode].rshift;
					gshift = fl_state[fl_vmode].gshift;
					bshift = fl_state[fl_vmode].bshift;
					rmult = 0.99f + (fl_state[fl_vmode].rmask >> rshift);
					gmult = 0.99f + (fl_state[fl_vmode].gmask >> gshift);
					bmult = 0.99f + (fl_state[fl_vmode].bmask >> bshift);
				}
			}
		}
		xshm_error = !ximg;
#endif
	}

	if (!framebuffer) {
		// Allocate a FB on the heap
		depth = 4;
		framebuffer = new unsigned char[depth*width*height];
	}

	// Allocate a depth buffer
	depthbuffer = new float[width*height];

	// Zero the FB and depth buffer
	memset(framebuffer, 0, depth*width*height);
	for (int i=0; i < width*height; i++)
		depthbuffer[i] = FLT_MAX;
}


// Draw a point at (cx, cy, cz).  Radius is r in world coords, which must
// correspond to a screen-space size (diameter) of splatsize.
// Normal (3 floats) is pointed to by norm.
// Color (3 floats) is pointed to by col.
void drawpoint_software(float cx, float cy, float cz,
			float r, float splatsize,
			const float *norm, const float *col)
{
	float X, Y, Z;
	FastProject(cameramatrix, cx, cy, cz, X, Y, Z);
	if (Z < 0.0f)
		return;
	if (flipY)
		Y = height - Y;

	float ss2 = 0.5f * splatsize;
	int startx = max(0, int(X - ss2 + 0.5f));
	int endx = min(width-1, int(X + ss2 + 0.5f));
	if (startx > width-1 || endx < 0)
		return;

	int starty = max(0, int(Y - ss2 + 0.5f));
	int endy = min(height-1, int(Y + ss2 + 0.5f));
	if (starty > height-1 || endy < 0)
		return;


	float lighting = 0.05f + 0.85f * max(Dot(norm, lightdir_mesh), 0.0f);

	int startindex = startx + width * starty;
	unsigned char *f = framebuffer + depth*startindex;
	float *d = depthbuffer + startindex;
	int lineskip = width + startx - endx;

#define BLIT4 if (Z < *d) { *d = Z; * (unsigned *)f = COL; }
#define BLIT2 if (Z < *d) { *d = Z; * (unsigned short *)f = COL; }

	if (depth == 4) {
		unsigned COL = col ?
			RGB2BITS(lighting*col[0], lighting*col[1], lighting*col[2]) :
			(unsigned(lighting*230.4f) * 0x01010101u);
		for (int row = starty; row <= endy; row++) {
			switch (endx-startx) {
				case 19: BLIT4 ; f+=4; d++;
				case 18: BLIT4 ; f+=4; d++;
				case 17: BLIT4 ; f+=4; d++;
				case 16: BLIT4 ; f+=4; d++;
				case 15: BLIT4 ; f+=4; d++;
				case 14: BLIT4 ; f+=4; d++;
				case 13: BLIT4 ; f+=4; d++;
				case 12: BLIT4 ; f+=4; d++;
				case 11: BLIT4 ; f+=4; d++;
				case 10: BLIT4 ; f+=4; d++;
				case 9:  BLIT4 ; f+=4; d++;
				case 8:  BLIT4 ; f+=4; d++;
				case 7:  BLIT4 ; f+=4; d++;
				case 6:  BLIT4 ; f+=4; d++;
				case 5:  BLIT4 ; f+=4; d++;
				case 4:  BLIT4 ; f+=4; d++;
				case 3:  BLIT4 ; f+=4; d++;
				case 2:  BLIT4 ; f+=4; d++;
				case 1:  BLIT4 ; f+=4; d++;
				case 0:  BLIT4 ; f += 4*lineskip; d += lineskip;
					 continue;
			}
			for (int column = startx; column <= endx; column++) {
				BLIT4 ; f+=4; d++;
			}
			f += 4*lineskip-4; d += lineskip-1;
		}
	} else {
		// depth == 2
		unsigned short COL = col ?
			RGB2BITS(lighting*col[0], lighting*col[1], lighting*col[2]) :
			RGB2BITS(lighting*0.9f, lighting*0.9f, lighting*0.9f);
		for (int row = starty; row <= endy; row++) {
			switch (endx-startx) {
				case 19: BLIT2 ; f+=2; d++;
				case 18: BLIT2 ; f+=2; d++;
				case 17: BLIT2 ; f+=2; d++;
				case 16: BLIT2 ; f+=2; d++;
				case 15: BLIT2 ; f+=2; d++;
				case 14: BLIT2 ; f+=2; d++;
				case 13: BLIT2 ; f+=2; d++;
				case 12: BLIT2 ; f+=2; d++;
				case 11: BLIT2 ; f+=2; d++;
				case 10: BLIT2 ; f+=2; d++;
				case 9:  BLIT2 ; f+=2; d++;
				case 8:  BLIT2 ; f+=2; d++;
				case 7:  BLIT2 ; f+=2; d++;
				case 6:  BLIT2 ; f+=2; d++;
				case 5:  BLIT2 ; f+=2; d++;
				case 4:  BLIT2 ; f+=2; d++;
				case 3:  BLIT2 ; f+=2; d++;
				case 2:  BLIT2 ; f+=2; d++;
				case 1:  BLIT2 ; f+=2; d++;
				case 0:  BLIT2 ; f += 2*lineskip; d += lineskip;
					 continue;
			}
			for (int column = startx; column <= endx; column++) {
				BLIT2 ; f+=2; d++;
			}
			f += 2*lineskip-2; d += lineskip-1;
		}
	}

	theQSplatGUI->pts_splatted++;
}


// Free all the resources we allocated
static void cleanup()
{
#ifndef WIN32
	if (!use_gldrawpixels && !xshm_error)
	{
		destroy_xshm_image(fl_display,ximg);
		framebuffer = NULL;
	}
#endif
	if (framebuffer)
		delete[] framebuffer;
	delete [] depthbuffer;
}


// Little function called right after we finish drawing
void end_drawing_software(bool bailed)
{
	// If we bailed, don't bother drawing
	if (bailed) {
		cleanup();
		return;
	}


	// Draw the image
	if (!use_gldrawpixels && !xshm_error) {
		// Direct blit to screen
#ifdef WIN32
		BITMAPINFO info = { { sizeof(BITMAPINFOHEADER),
			width, height, 1, 32, BI_RGB, 0, 0, 0, 0, 0 }, NULL };
		::SetStretchBltMode(GUI->hDC,COLORONCOLOR);
		::StretchDIBits(GUI->hDC,0,0,width, height, 0, 0, width, 
			height, framebuffer, &info, DIB_RGB_COLORS, SRCCOPY);
#else
		XShmPutImage(fl_display, GUI->theGLwindow(), fl_get_gc(),
			     ximg, 0, 0,
			     0, 0, width, height, False);
#endif
		cleanup();
		return;
	}
	

	// Else use glDrawpixels()
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRasterPos2f(-1.0f, -1.0f);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDrawPixels(width, height,
		     GL_RGBA, GL_UNSIGNED_BYTE,
		     framebuffer);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	if (xshm_error) {
		// GUI layer won't swapbuffers for us, since it expected that
		// we'd be drawing directly to the screen
		GUI->swapbuffers();
	}

	cleanup();
}

