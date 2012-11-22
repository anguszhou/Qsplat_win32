/*
Gary King
Szymon Rusinkiewicz

qsplat_draw_software_tiles.cpp
Draws splats into an off-screen FB using Z-sorted tile-based rendering, then
blits them onto the screen.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_guimain.h"
#include <Windows.h>
#include <GL/gl.h>
#include <float.h>
#include <vector>
#include <algorithm>

#ifdef WIN32
# include "qsplat_gui_win32.h"
#else
# include "qsplat_gui_xforms.h"
# include "xshm.h"
#endif


// Class containing a quad, together with comparison and memory-management
// routines
struct quadrangle
{
	int startx, endx, starty, endy;
	unsigned col;
	float Z;

	struct compare
	{
		bool operator () (const quadrangle *lhs, const quadrangle *rhs) const
		{
			return lhs->Z < rhs->Z;
		}
	};

	class quadrangle_mempool
	{
	private:
		enum { chunksize = 10000 };
		std::vector<quadrangle *> v;
		int n;
	public:
		quadrangle *next()
		{
			if (v.empty() || n == chunksize) {
				v.push_back(new quadrangle[chunksize]);
				n = 0;
			}
			return v.back() + (n++);
		}
		void release()
		{
			for (int i=0; i < v.size(); i++)
				delete [] v[i];
			v.clear();
		}
	};

	static quadrangle_mempool qpool;
};

quadrangle::quadrangle_mempool quadrangle::qpool;


// Words with 0 through 32 bits set, starting with the lsb
static const unsigned binary_ones[33] =
{
	0,
	0x00000001u, 0x00000003u, 0x00000007u, 0x0000000fu,
	0x0000001fu, 0x0000003fu, 0x0000007fu, 0x000000ffu,
	0x000001ffu, 0x000003ffu, 0x000007ffu, 0x00000fffu,
	0x00001fffu, 0x00003fffu, 0x00007fffu, 0x0000ffffu,
	0x0001ffffu, 0x0003ffffu, 0x0007ffffu, 0x000fffffu,
	0x001fffffu, 0x003fffffu, 0x007fffffu, 0x00ffffffu,
	0x01ffffffu, 0x03ffffffu, 0x07ffffffu, 0x0fffffffu,
	0x1fffffffu, 0x3fffffffu, 0x7fffffffu, 0xffffffffu
};


// Local variables
static float cameramatrix[16];
static float lightdir_mesh[3];
static int width, height, depth, depthshift;
static int rshift, gshift, bshift;
static float rmult, gmult, bmult;
static bool flipY;
static bool use_gldrawpixels;
static unsigned char *framebuffer;
static int horz_tiles, vert_tiles;
static int final_tile_width, final_tile_height;
static timestamp renderstarttime;

#ifndef WIN32
  static XImage *ximg;
#endif
static bool xshm_error = false;

static std::vector<quadrangle*> *quadQueue;


#define RGB2BITS(r,g,b) ((unsigned((r)*rmult) << rshift) | \
			 (unsigned((g)*gmult) << gshift) | \
			 (unsigned((b)*bmult) << bshift))


/*******************************************************************************
*  Sets up the final transformation matrix, any memory needed (e.g., the frame *
*  buffer), etc.                                                               *
*******************************************************************************/

void start_drawing_software_tiles(bool _use_gldrawpixels)
{
	use_gldrawpixels = _use_gldrawpixels;

	float P[16], M[16], V[4];
	glGetFloatv(GL_PROJECTION_MATRIX, P);
	glGetFloatv(GL_MODELVIEW_MATRIX,  M);
	glGetFloatv(GL_VIEWPORT, V);
	width = int(V[2]);  height = int(V[3]);
	V[0] = 0;  V[1] = 0; // Get rid of effects of glViewport(x,y,...)
	FastProjectPrecompute(P, M, V, cameramatrix);

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

	if (!use_gldrawpixels)
	{
#ifdef WIN32
		rshift = 16; gshift = 8; bshift = 0;
#else
		if (fl_state[fl_vmode].depth >= 15 &&
		    fl_state[fl_vmode].depth <= 32 &&
		    check_for_xshm(fl_display)) {
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

	if (!framebuffer)
	{
		depth = 4;
		framebuffer = new unsigned char[depth*width*height];
	}

	memset(framebuffer, 0, depth*width*height);

	depthshift = (depth == 4) ? 2 : 1;

	horz_tiles = ((width-1) >> 5) + 1;
	vert_tiles = ((height-1) >> 5) + 1;
	final_tile_width = width & 31;
	final_tile_height = height & 31;

	quadQueue = new std::vector<quadrangle*>[horz_tiles*vert_tiles];

	get_timestamp(renderstarttime);

}  //  start_drawing_software_tiles


/****************************************************************************
*  drawpoint_software_tiles().  Clips a given splat to any tiles which it   *
*  intersects, then inserts the mini-splat into a vector of all other mini- *
*  splats for that tile.                                                    *
****************************************************************************/
void drawpoint_software_tiles(float cx, float cy, float cz, float rad, float splatsize, const float* norm, const float* col)
{
	float X, Y, Z;
	FastProject(cameramatrix, cx, cy, cz, X, Y, Z);
	if (Z < 0.0f) return;

	if (flipY) Y = height - Y;

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

	unsigned COL = col ?
		RGB2BITS(lighting*col[0], lighting*col[1], lighting*col[2]) :
		(depth == 4) ? unsigned(230.4f*lighting)*0x01010101u :
			       RGB2BITS(lighting*0.9f, lighting*0.9f, lighting*0.9f);

	int x_tile_start = startx >> 5,  x_tile_end = endx >> 5;
	int y_tile_start = starty >> 5,  y_tile_end = endy >> 5;
	for (int ytile = y_tile_start; ytile<=y_tile_end; ytile++)
	{
		int tile_starty = (ytile == y_tile_start) ? (starty & 31) : 0;
		int tile_endy = (ytile == y_tile_end)   ? (endy   & 31) : 31;
		int ytile_index = ytile * horz_tiles;
		for (int xtile = x_tile_start; xtile<=x_tile_end; xtile++)
		{
			quadrangle* q = quadrangle::qpool.next();
			q->startx = (xtile == x_tile_start) ? (startx & 31) : 0;
			q->endx   = (xtile == x_tile_end)   ? (endx   & 31) : 31;
			q->starty = tile_starty;
			q->endy   = tile_endy;
			q->col = COL;
			q->Z = Z;
			quadQueue[ytile_index+xtile].push_back(q);
		}
	}

	theQSplatGUI->pts_splatted++;
}  // drawpoint_software_tiles


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
	quadrangle::qpool.release();
	delete[] quadQueue;
}


/******************************************************************************************
*  end_drawing_software_tiles().  Given the horz_tiles*vert_tiles sorted vectors of mini-  *
*  splats, draws all splats into the framebuffer in back-to-front order, and blits the   *
*  frame buffer to the screen.                                                           *
******************************************************************************************/

void end_drawing_software_tiles(bool bailed)
{
	if (bailed)
	{
		cleanup();
		return;
	}

	int i=0;
	for (int y=0; y < vert_tiles; y++)
	{
		if (y & 7 == 0)
		{
			timestamp now;
			get_timestamp(now);
			if (GUI->abort_drawing(now - renderstarttime))
			{
				cleanup();
				return;
			}
		}

		for (int x=0; x < horz_tiles; x++, i++)
		{

			if (quadQueue[i].empty()) continue;

			// row_mask specifies which pixels in this tile (still) need
			// to be drawn.  That is, if bit i in row_mask[j] is set,
			// the pixel in row j column i has not been touched
			unsigned row_mask[32];
			unsigned m = 0xffffffffu;
			if (x == horz_tiles-1 && final_tile_width != 0)
				m = binary_ones[final_tile_width];

			if (y == vert_tiles - 1) // last row
			{
				int row = 0;
				while (row < final_tile_height)
					row_mask[row++] = m;
				while (row < 32)
					row_mask[row++] = 0u;  // since the row doesn't exist, we'll pretend it's full
			}
			else
				for (int row=0; row < 32; row++)
					row_mask[row] = m;


			std::sort(quadQueue[i].begin(), quadQueue[i].end(), quadrangle::compare());
			for (std::vector<quadrangle*>::iterator p = quadQueue[i].begin(); p!=quadQueue[i].end(); ++p)
			{
				const quadrangle* q  = *p;
				unsigned char* bptr  = framebuffer + ((
						(q->starty + (y<<5)) * width +
						(q->startx + (x<<5))
					) << depthshift);
				int linewidth        = q->endx - q->startx;
				int lineskip         = (width - linewidth) << depthshift;
				unsigned tilecolmask = binary_ones[q->endx+1] ^ binary_ones[q->startx];

				for (int thisrow = q->starty; thisrow <= q->endy; thisrow++)
				{
					unsigned drawmask = row_mask[thisrow];
					if ((drawmask & tilecolmask) == 0u)
					{
						// We won't do any useful work,
						// so skip this row
						bptr += width << depthshift;
						continue;
					}
					drawmask >>= q->startx;

					if (depth == 4)
					{
						unsigned int col = q->col;
						switch(linewidth)
						{
						case 31: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 30: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 29: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 28: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 27: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 26: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 25: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 24: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 23: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 22: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 21: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 20: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 19: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 18: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 17: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 16: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 15: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 14: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 13: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 12: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 11: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case 10: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  9: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  8: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  7: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  6: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  5: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  4: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  3: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  2: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  1: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += 4; drawmask >>= 1;
						case  0: if (drawmask & 1) { *(unsigned int *)bptr = col; }  bptr += lineskip;
						}
					}
					else  // depth == 2
					{
						unsigned short col = q->col;
						switch(linewidth)
						{
						case 31: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 30: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 29: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 28: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 27: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 26: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 25: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 24: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 23: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 22: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 21: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 20: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 19: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 18: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 17: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 16: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 15: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 14: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 13: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 12: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 11: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case 10: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  9: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  8: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  7: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  6: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  5: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  4: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  3: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  2: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  1: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += 2; drawmask >>= 1;
						case  0: if (drawmask & 1) { *(unsigned short *)bptr = col; }  bptr += lineskip;
						}
					}
					row_mask[thisrow] &= ~tilecolmask;
				}
				if (!row_mask[ 0] && !row_mask[ 1] && !row_mask[ 2] && !row_mask[ 3] && !row_mask[ 4] && !row_mask[ 5] && !row_mask[ 6] && !row_mask[ 7] &&
				    !row_mask[ 8] && !row_mask[ 9] && !row_mask[10] && !row_mask[11] && !row_mask[12] && !row_mask[13] && !row_mask[14] && !row_mask[15] &&
				    !row_mask[16] && !row_mask[17] && !row_mask[18] && !row_mask[19] && !row_mask[20] && !row_mask[21] && !row_mask[22] && !row_mask[23] &&
				    !row_mask[24] && !row_mask[25] && !row_mask[26] && !row_mask[27] && !row_mask[28] && !row_mask[29] && !row_mask[30] && !row_mask[31])
					break;
			} // for each tile
		} // for x
	} // for y

	if (!use_gldrawpixels && !xshm_error)
	{
#ifdef WIN32
		BITMAPINFO info = { { sizeof(BITMAPINFOHEADER), width, height, 1, 32, BI_RGB, 0, 0, 0, 0, 0 }, NULL };
		::SetStretchBltMode(GUI->hDC,COLORONCOLOR);
		::StretchDIBits(GUI->hDC,0,0,width, height, 0, 0, width, height, framebuffer, &info, DIB_RGB_COLORS, SRCCOPY);
#else
		XShmPutImage(fl_display, GUI->theGLwindow(), fl_get_gc(), ximg, 0, 0, 0, 0, width, height, False);
#endif
		cleanup();
		return;
	}

	// Else use glDrawPixels
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRasterPos2f(-1.0f, -1.0f);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDrawPixels(width,height,GL_RGBA, GL_UNSIGNED_BYTE, framebuffer);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	if (xshm_error)
		GUI->swapbuffers();
	cleanup();
}  //  end_drawing_software_tiles

