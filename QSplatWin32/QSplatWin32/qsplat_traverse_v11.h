#ifndef QSPLAT_TRAVERSE_V11_H
#define QSPLAT_TRAVERSE_V11_H
/*
Szymon Rusinkiewicz

qsplat_traverse_v11.h
Traverse a version-11 (bsphere hierarchy, pointers, optional color)
QSplat file.

Note that the only ways this interfaces with the outside world are:
	 - Reads the OpenGL context to determine camera parameters
	 - Calls the following functions (defined in qsplat_gui*.h):
		GUI->start_drawing()
		GUI->drawpoint()
		GUI->abort_drawing()
		GUI->end_drawing()

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#define QSPLAT_FILE_VERSION 11
#define FAST_CUTOFF (2.3f*minsize)


// Local variables
static int screenwidth, screenheight;
static float pixels_per_radian, recursion_thresh;
static point campos;
static float frustum[4][4];
static float zproj[4];
static float minsize;
static bool bail;
static timestamp renderstarttime;
static bool havecolor;
static int nodesize;


// We've gotten to the lowest level of the hierarchy, and we're just going to
// draw a bunch of leaf nodes without testing their sizes
static inline void draw_hierarchy_leaves(const unsigned char *here, int numnodes,
					 float cx, float cy, float cz, float r,
					 float approx_splatsize_scale)
{
	for (int i=0; i < numnodes; i++, here += nodesize) {
		float mycx, mycy, mycz, myr;
		QSplat_SphereQuant::lookup(here,
					   cx, cy, cz, r,
					   mycx, mycy, mycz, myr);
		float splatsize = approx_splatsize_scale ?
				  myr * approx_splatsize_scale :
				  minsize;
		GUI->drawpoint(mycx, mycy, mycz,
			       myr, splatsize,
			       QSplat_NormQuant::lookup(here+2),
			       havecolor?QSplat_ColorQuant::lookup(here+4):NULL);
	}
}


// The fast version of the draw routine.  We switch to this when size gets
// down to a few pixels.
// See draw_hierarchy() for comments...
static inline void draw_hierarchy_fast(const unsigned char *here, int numnodes,
				       float cx, float cy, float cz, float r)
{
	int childoffset = UNALIGNED_DEREFERENCE_INT(here);
	FIX_LONG(childoffset);
	const unsigned char *there = here + childoffset;
	here += 4;

	int numchildren = 0;
	int grandchildren = 0;

	for (int i=0; i < numnodes; i++, here += nodesize, there += nodesize*numchildren + grandchildren) {

		numchildren = here[1] & 3;
		if (numchildren) {
			numchildren++;
			grandchildren = here[1] & 4;
		} else {
			grandchildren = 0;
		}

		float mycx, mycy, mycz, myr;
		QSplat_SphereQuant::lookup(here,
					   cx, cy, cz, r,
					   mycx, mycy, mycz, myr);

		float z = zproj[0] * mycx + zproj[1] * mycy +
			  zproj[2] * mycz + zproj[3];
		float splatsize_scale = 2.0f * pixels_per_radian / z;
		float splatsize = myr * splatsize_scale;

		if (!numchildren || (splatsize <= minsize)) {
			GUI->drawpoint(mycx, mycy, mycz,
				       myr, splatsize,
				       QSplat_NormQuant::lookup(here+2),
				       havecolor?QSplat_ColorQuant::lookup(here+4):NULL);
		} else if (!grandchildren) {
			draw_hierarchy_leaves(there, numchildren,
					      mycx, mycy, mycz, myr,
					      splatsize_scale);
		} else if (splatsize <= FAST_CUTOFF) {
			draw_hierarchy_leaves(there+4, numchildren,
					      mycx, mycy, mycz, myr,
					      0.0f);
		} else {
			draw_hierarchy_fast(there, numchildren,
					    mycx, mycy, mycz, myr);
		}
	}
}


// The main drawing routine
static void draw_hierarchy(const unsigned char *here, int numnodes,
			   float cx, float cy, float cz, float r,
			   bool backfacecull, bool frustumcull)
{
 	// Check for events, but not too often
	static unsigned counter = 0;
	if (!(counter++ & 0xff)) {
		timestamp now;
		get_timestamp(now);
		float elapsed = now - renderstarttime;
		if (GUI->abort_drawing(elapsed)) {
			bail = true;
			return;
		}
	}


	// Where are the children of these nodes stored?
	int childoffset = UNALIGNED_DEREFERENCE_INT(here);
	FIX_LONG(childoffset);
	const unsigned char *there = here + childoffset;
	here += 4;


	int numchildren = 0;
	int grandchildren = 0;

	// For each node in this group of siblings
	for (int i=0; i < numnodes; i++, here += nodesize, there += nodesize*numchildren + grandchildren) {
		// Find number of children
		numchildren = here[1] & 3;
		if (numchildren) {
			// Code: 0 really means no children, but since 1 child
			// never happens, 1 really means 2 children and so on

			numchildren++;

			// The "grandchildren" bit is set if this node has any
			// grandchildren, and hence has the extra pointer
			// Note that because this uses bit 2, we automagically
			// get the right increment to the child offset just by
			// extracting this bit.

			grandchildren = here[1] & 4;

		} else {
			grandchildren = 0;
		}


		// Determine our position and radius
		float mycx, mycy, mycz, myr;
		QSplat_SphereQuant::lookup(here,
					   cx, cy, cz, r,
					   mycx, mycy, mycz, myr);


		// Determine perpendicular distance to screen plane
		float z = zproj[0] * mycx + zproj[1] * mycy +
			  zproj[2] * mycz + zproj[3];


		// Frustum culling
		bool frustumcull_children = frustumcull;
		if (frustumcull) {
			if ((z <= -myr) ||
			    (mycx*frustum[0][0] + mycy*frustum[0][1] +
			     mycz*frustum[0][2] + frustum[0][3] <= -myr) ||
			    (mycx*frustum[1][0] + mycy*frustum[1][1] +
			     mycz*frustum[1][2] + frustum[1][3] <= -myr) ||
			    (mycx*frustum[2][0] + mycy*frustum[2][1] +
			     mycz*frustum[2][2] + frustum[2][3] <= -myr) ||
			    (mycx*frustum[3][0] + mycy*frustum[3][1] +
			     mycz*frustum[3][2] + frustum[3][3] <= -myr))
				continue;
			if ((z > myr) &&
			    (mycx*frustum[0][0] + mycy*frustum[0][1] +
			     mycz*frustum[0][2] + frustum[0][3] >= myr) &&
			    (mycx*frustum[1][0] + mycy*frustum[1][1] +
			     mycz*frustum[1][2] + frustum[1][3] >= myr) &&
			    (mycx*frustum[2][0] + mycy*frustum[2][1] +
			     mycz*frustum[2][2] + frustum[2][3] >= myr) &&
			    (mycx*frustum[3][0] + mycy*frustum[3][1] +
			     mycz*frustum[3][2] + frustum[3][3] >= myr))
				frustumcull_children = false;
		}


		// Backface culling
		bool backfacecull_children = backfacecull;
		float camdotnorm = 0.0f; // If we're not doing backface
					 // culling, this gets left set to a
					 // safe value for the sake of code
					 // later on.
		if (backfacecull && ((here[3] & 3) != 3)) {
			const float *norm = QSplat_NormQuant::lookup(here+2);
			float camx = campos[0] - mycx;
			float camy = campos[1] - mycy;
			float camz = campos[2] - mycz;
			camdotnorm = camx * norm[0] +
				     camy * norm[1] +
				     camz * norm[2];
			if (camdotnorm < -myr) {
				float camdist2 = sqr(camx) + sqr(camy) + sqr(camz);
				float cone = QSplat_NormQuant::lookup_cone(here+2);
				if (sqr(camdotnorm + myr) > camdist2 * sqr(cone)) {
					continue;
				}
			} else if (camdotnorm > myr) {
				float camdist2 = sqr(camx) + sqr(camy) + sqr(camz);
				float cone = QSplat_NormQuant::lookup_cone(here+2);
				if (sqr(camdotnorm - myr) > camdist2 * sqr(cone)) {
					backfacecull_children = false;
				}
			}
		}

		// Yes, we actually have to (gasp) do a divide.
		float splatsize_scale = 2.0f * pixels_per_radian / z;
		float splatsize = myr * splatsize_scale;

		// Check whether we recurse...
		if (!numchildren || ((z > 0.0f) && (splatsize <= minsize))) {
			// No - draw now
			if ((z > 0.0f) && (camdotnorm >= 0.0f)) {
				GUI->drawpoint(mycx, mycy, mycz,
					       myr, splatsize,
					       QSplat_NormQuant::lookup(here+2),
					       havecolor?QSplat_ColorQuant::lookup(here+4):NULL);
			}
		} else if (!grandchildren) {
			// We recurse, but children are all leaf nodes
			draw_hierarchy_leaves(there, numchildren,
					      mycx, mycy, mycz, myr,
					      splatsize_scale);
		} else if ((!frustumcull_children && !backfacecull_children) ||
			   ((splatsize <= FAST_CUTOFF) && (z > 0.0f))) {
			// We recurse, but switch to fast mode
			if (splatsize <= FAST_CUTOFF) {
				// Hack: if we're this small, the next round
				// of recursion is going to be awfully close to
				// minsize, so we use the _leaves function
				// to just draw the children...
				draw_hierarchy_leaves(there+4, numchildren,
						      mycx, mycy, mycz, myr,
						      0.0f);
			} else {
				draw_hierarchy_fast(there, numchildren,
						    mycx, mycy, mycz, myr);
			}
		} else {
			// Basic slow-mode recursion
			draw_hierarchy(there, numchildren,
				       mycx, mycy, mycz, myr,
				       backfacecull_children,
				       frustumcull_children);
			if (bail)
				return;
		}
	}
}



// Dig out the required information from the header of an individual fragment.
// For V11 files, just returns center of the highest-level sphere.
static inline void parse_header(const unsigned char *here,
				const unsigned char **drawstart,
				int *numchildren,
				float *rootcx, float *rootcy,
				float *rootcz, float *rootr)
{
	unsigned options = * (int *)(here+16);  FIX_LONG(options);
	havecolor = options & 1;
	nodesize = (havecolor ? 6 : 4);

	*rootcx = * (float *)(here+20);  FIX_FLOAT(*rootcx);
	*rootcy = * (float *)(here+24);  FIX_FLOAT(*rootcy);
	*rootcz = * (float *)(here+28);  FIX_FLOAT(*rootcz);
	*rootr  = * (float *)(here+32);  FIX_FLOAT(*rootr);
	*numchildren = * (int *)(here+36);  FIX_LONG(*numchildren);
	*drawstart = here+40;
}


// Entry to the drawing routine.
// This first digs out a bit of information from the OpenGL context, then we
// iterate over the fragments and call the routines that actually do the
// drawing.
bool QSplat_Model::draw()
{
	// Read back OpenGL matrices
	float P[16], M[16], V[4];
	glGetFloatv(GL_PROJECTION_MATRIX, P);
	glGetFloatv(GL_MODELVIEW_MATRIX, M);
	glGetFloatv(GL_VIEWPORT, V);


	// Compute some stuff
	campos[0] = -(M[0]*M[12] + M[1]*M[13] + M[2]*M[14]);
	campos[1] = -(M[4]*M[12] + M[5]*M[13] + M[6]*M[14]);
	campos[2] = -(M[8]*M[12] + M[9]*M[13] + M[10]*M[14]);
	screenwidth = int(V[2]), screenheight = int(V[3]);
	pixels_per_radian = 0.5f * screenwidth * P[0]; // Assume glFrustum only
	recursion_thresh = 0.5f * minsize / pixels_per_radian;

	zproj[0] = -M[2];
	zproj[1] = -M[6];
	zproj[2] = -M[10];
	zproj[3] = -M[14];

	float PM[16];
	MMult(P, M, PM);
	frustum[0][0] = PM[3]  - PM[1];
	frustum[0][1] = PM[7]  - PM[5];
	frustum[0][2] = PM[11] - PM[9];
	frustum[0][3] = PM[15] - PM[13];
	float tmp = 1.0f / Len(&frustum[0][0]);
	frustum[0][0] *= tmp;
	frustum[0][1] *= tmp;
	frustum[0][2] *= tmp;
	frustum[0][3] *= tmp;
	frustum[1][0] = PM[3]  + PM[1];
	frustum[1][1] = PM[7]  + PM[5];
	frustum[1][2] = PM[11] + PM[9];
	frustum[1][3] = PM[15] + PM[13];
	tmp = 1.0f / Len(&frustum[1][0]);
	frustum[1][0] *= tmp;
	frustum[1][1] *= tmp;
	frustum[1][2] *= tmp;
	frustum[1][3] *= tmp;
	frustum[2][0] = PM[3]  - PM[0];
	frustum[2][1] = PM[7]  - PM[4];
	frustum[2][2] = PM[11] - PM[8];
	frustum[2][3] = PM[15] - PM[12];
	tmp = 1.0f / Len(&frustum[2][0]);
	frustum[2][0] *= tmp;
	frustum[2][1] *= tmp;
	frustum[2][2] *= tmp;
	frustum[2][3] *= tmp;
	frustum[3][0] = PM[3]  + PM[0];
	frustum[3][1] = PM[7]  + PM[4];
	frustum[3][2] = PM[11] + PM[8];
	frustum[3][3] = PM[15] + PM[12];
	tmp = 1.0f / Len(&frustum[3][0]);
	frustum[3][0] *= tmp;
	frustum[3][1] *= tmp;
	frustum[3][2] *= tmp;
	frustum[3][3] *= tmp;

	bool backfacecull = !!glIsEnabled(GL_CULL_FACE);
	::minsize = minsize;

	// Prepare for drawing
	bail = false;
	get_timestamp(renderstarttime);
	GUI->start_drawing(havecolor);


	// Draw each fragment
	for (int f = 0; f < fragments.size(); f++) {
		float cx, cy, cz, r;
		int numchildren;
		const unsigned char *drawstart;
		parse_header(fragments[f], &drawstart, &numchildren,
			     &cx, &cy, &cz, &r);
		draw_hierarchy(drawstart, numchildren,
			       cx, cy, cz, r,
			       backfacecull, true);
		if (bail)
			break;
	}

	// That's all, folks
	GUI->end_drawing(bail);

	return bail;
}


// Trace a ray through the hierarchy, and find distance to intersection.
static void traceray_hierarchy(const unsigned char *here, int numnodes,
			       float cx, float cy, float cz, float r,
			       bool leaves,
			       const float *pt, const float *dir,
			       float cutoff, float &best)
{
	const unsigned char *there = NULL;
	if (!leaves) {
		int childoffset = UNALIGNED_DEREFERENCE_INT(here);
		FIX_LONG(childoffset);
		there = here + childoffset;
		here += 4;
	}

	int numchildren = 0;
	int grandchildren = 0;

	for (int i=0;
	     i < numnodes;
	     i++, here += nodesize, there += nodesize*numchildren + grandchildren) {

		numchildren = here[1] & 3;
		if (numchildren) {
			numchildren++;
			grandchildren = here[1] & 4;
		} else {
			grandchildren = 0;
		}

		float mycx, mycy, mycz, myr;
		QSplat_SphereQuant::lookup(here,
					   cx, cy, cz, r,
					   mycx, mycy, mycz, myr);

		vec x = { mycx-pt[0], mycy-pt[1], mycz-pt[2] };

		float t = Dot(x, dir);
		if ((t < -myr) || (t-myr > best))
			continue;
		point ipoint = { pt[0] + t * dir[0],
				 pt[1] + t * dir[1],
				 pt[2] + t * dir[2] };

		float idist2 = sqr(mycx - ipoint[0]) +
			       sqr(mycy - ipoint[1]) +
			       sqr(mycz - ipoint[2]);

		if (idist2 > sqr(myr))
			continue;

		if ((myr < t * cutoff) || !numchildren) {
			// Terminal case
			if ((t > 0) && (t < best))
				best = t;
		} else {
			// Recursive case
			traceray_hierarchy(there, numchildren,
					   mycx, mycy, mycz, myr,
					   !grandchildren,
					   pt, dir,
					   cutoff, best);
		}
	}
}


// Trace a ray through the QSplat model.
// Ray is traced starting at the camera position, and in a direction
// corresponding to pixel (x,y).  The recursion only goes down as far as
// nodes of size "cutoff" pixels
float QSplat_Model::traceray(int x, int y, float cutoff)
{
	float P[16], M[16], V[4];
	glGetFloatv(GL_PROJECTION_MATRIX, P);
	glGetFloatv(GL_MODELVIEW_MATRIX, M);
	glGetFloatv(GL_VIEWPORT, V);

	campos[0] = -(M[0]*M[12] + M[1]*M[13] + M[2]*M[14]);
	campos[1] = -(M[4]*M[12] + M[5]*M[13] + M[6]*M[14]);
	campos[2] = -(M[8]*M[12] + M[9]*M[13] + M[10]*M[14]);
	screenwidth = int(V[2]), screenheight = int(V[3]);
	pixels_per_radian = 0.5f * screenwidth * P[0];

	vec viewdir = { -M[2], -M[6], -M[10] };
	vec updir = { M[1], M[5], M[9] };
	vec rightdir = { M[0], M[4], M[8] };

	float xx = (x - 0.5f * screenwidth) / pixels_per_radian;
	float yy = (y - 0.5f * screenheight) / pixels_per_radian;
	vec raydir = { viewdir[0] + xx * rightdir[0] + yy * updir[0],
		       viewdir[1] + xx * rightdir[1] + yy * updir[1],
		       viewdir[2] + xx * rightdir[2] + yy * updir[2] };
	Normalize(raydir);

	float cc = cutoff / pixels_per_radian;

	float t = 3.3e33f;

	for (int f = 0; f < fragments.size(); f++) {
		float cx, cy, cz, r;
		int numchildren;
		const unsigned char *drawstart;
		parse_header(fragments[f], &drawstart, &numchildren,
			     &cx, &cy, &cz, &r);
		traceray_hierarchy(drawstart, numchildren,
				   cx, cy, cz, r,
				   false,
				   campos, raydir,
				   cc, t);
	}
	if (t == 3.3e33f)
		return 0;
	else
		return t;
}

#endif
