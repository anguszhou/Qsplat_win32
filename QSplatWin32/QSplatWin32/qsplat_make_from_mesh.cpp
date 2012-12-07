/*
Szymon Rusinkiewicz

qsplat_make_from_mesh.cpp

Code for reading in a triangle mesh from a ply file and initializing the
QSplat data structure.

This does not read and write general .ply files - only a very restricted
subset.  In particular, only binary big-endian ply files with just vertices
and faces or tstrips are supported.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/
#include "stdafx.h"
# include <windows.h>
# include <winbase.h>
# include <io.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "qsplat_make_from_mesh.h"

#define BIGNUM FLT_MAX
#pragma comment(lib , "ANN.lib")

// An error occurred while trying to open the file
static void Error(const char *s1, const char *s2)
{
#ifdef WIN32
	char buf[255];
	sprintf(buf, "%s%s",s1, s2);
	MessageBox(NULL, buf, "QSplat", MB_OK | MB_ICONEXCLAMATION);
#else
	fprintf(stderr, "%s%s\n", s1, s2);
#endif
}

// Unpack tstrips into faces
static void unpack_tstrips(int tstripdatalen, const int *tstrips,
						   int &numfaces, face * &faces)
{
	int i;
	if (!tstrips || tstripdatalen < 4)
		return;
	//int fflush(FILE* stream):flush buffer or write buffer into file if it open by write way
	printf("Unpacking triangle strips... "); fflush(stdout);

	// Count number of faces
	numfaces = 0;
	int this_tstrip_len = 0;
	for (i=0; i < tstripdatalen; i++) {
		if (tstrips[i] == -1) {
			this_tstrip_len = 0;
			continue;
		}
		this_tstrip_len++;
		if (this_tstrip_len >= 3)
			numfaces++;
	}
	printf("%d triangles... ", numfaces); fflush(stdout);

	faces = new face[numfaces];//faces is an instance of int[numberfaces][3]

	int whichface = 0;
	this_tstrip_len = 0;
	for (i=0; i < tstripdatalen; i++) {
		if (tstrips[i] == -1) {
			this_tstrip_len = 0;
			continue;
		}
		this_tstrip_len++;
		if (this_tstrip_len < 3)
			continue;
		if (this_tstrip_len % 2) {
			faces[whichface][0] = tstrips[i-2];
			faces[whichface][1] = tstrips[i-1];
		} else {
			faces[whichface][0] = tstrips[i-1];
			faces[whichface][1] = tstrips[i-2];
		}
		faces[whichface++][2] = tstrips[i];
	}

	printf("Done.\n");
}


// Try to read a plyfile, returning vertices and faces
bool read_ply(const char *plyfile,
			  int &numleaves, QTree_Node * &leaves,
			  int &numfaces, face * &faces,
			  bool &have_colors,
			  std::string &comments)
{
	//initialize variables
	bool isbig_endian = false , pointcloud = false ;
	bool have_faces=false, have_tstrips=false;
	int tstripdatalen=0, *tstripdata = NULL;
	int other_prop_len, color_offset;
	char buf[255];
	int i, result;
	/*
	FILE *f2 = fopen(plyfile, "r");
	char buf2[255];
	while(!feof(f2))
	{
		fgets(buf2 , sizeof(buf2)-1 , f2);
		printf("%s",buf2);
	}
	fclose(f2);
	
	char buf2[255];
	std::ifstream file(plyfile);
	while(! file.eof())
	{
		file.getline(buf2,255);
		printf("%s",buf2);
	}
	file.close();
*/
	have_colors = false;  numleaves = numfaces = 0;
	leaves = NULL;  faces = NULL;

	//open .ply file 
	FILE *f = fopen(plyfile, "r");
	if (!f) {
		fprintf(stderr, "Can't open plyfile %s\n", plyfile);
		return false;
	}

	printf("Reading %s...\n", plyfile);


	// Read header
	//char* fgets(char *s,int n,FILE *file):read first n-1 chars of file, store in s and return s
	//only if read a line break char return s with one line data,otherwise return null.
	//int strncmp(char*s1,char*s2,int n):compare the first n chars of s1 and s2
	//if s1<s2 return <0 ; s1==s2 return =0 ; s1>s2 return >0
	//(value is ASCLL value of difference of first different char)
	if (!fgets(buf, 255, f) || strncmp(buf, "ply", 3)) {
		//int fprintf(FILE* stream,const char*format):output format to stream
		//if succeed return num of chars else return a negative value. 
		fprintf(stderr, "Not a ply file.\n");
		return false;
	}
	//GET_LINE():read one line data and store in buf
#define GET_LINE() if (!fgets(buf, 255, f)) goto plyreaderror

	//int strncasecmp(char*si,char*s2,int n):cmp the first n chars of s1 and s2 ignore case
	//if s1<s2 return <0 ; s1==s2 return =0 ; s1>s2 return >0
	//LINE_IS(text):cmp first len(text) of buf and text,if equel return not 0 else return 0
#define LINE_IS(text) !strncasecmp(buf, text, strlen(text))

	GET_LINE();
	//only LINE_IS return not 0 can go though means buf equal with text
	if (LINE_IS("format binary_big_endian 1.0")) {
		//fprintf(stderr, "Can only read binary big-endian ply files.\n");
		isbig_endian = true;
	}
	if (LINE_IS("format ascii 1.0")) {
		//fprintf(stderr, "Can only read binary big-endian ply files.\n");
		pointcloud = true;
	}

	while (1) {
		GET_LINE();
		if (LINE_IS("obj_info")) {
			continue;
		} else if (LINE_IS("comment")) {
			comments += buf+8;
			continue;
		} else {
			break;
		}
	}
	//int sscanf(char*s1,const char*format,char*s2):store s1 to s2 with format style
	//if succedd return num of valid data read else return 0. (return 1/0 here)
	result = sscanf(buf, "element vertex %d\n", &numleaves);
	if (result != 1) {
		fprintf(stderr, "Expected \"element vertex\"\n");
		goto plyreaderror;
	}

	GET_LINE();
	//only buff equal with text can go though
	if (!LINE_IS("property float x")) {
		fprintf(stderr, "Expected \"property float x\"\n");
		goto plyreaderror;
	}

	GET_LINE();
	//only buff equal with text can go though
	if (!LINE_IS("property float y")) {
		fprintf(stderr, "Expected \"property float y\"\n");
		goto plyreaderror;
	}

	GET_LINE();
	//only buff equal with text can go though
	if (!LINE_IS("property float z")) {
		fprintf(stderr, "Expected \"property float z\"\n");
		goto plyreaderror;
	}

	other_prop_len = 0;
	GET_LINE();
	//get total of property's byte and store in other_prop_len
	while (LINE_IS("property")) {
		if (LINE_IS("property char") ||
			LINE_IS("property uchar")) {
				other_prop_len += 1;
		} else if (LINE_IS("property int") ||
			LINE_IS("property uint") ||
			LINE_IS("property float")) {
				other_prop_len += 4;
		} else {
			fprintf(stderr, "Unsupported vertex property: %s\n", buf);
			goto plyreaderror;
		}
		//if have color information
		if (LINE_IS("property uchar diffuse_red") || LINE_IS("property uchar red")) {
			have_colors = true;
			color_offset = other_prop_len - 1;
		}

		GET_LINE();
	}

	//output a valid decimal to numfaces
	result = sscanf(buf, "element face %d", &numfaces);
	if (result == 1) {
		have_faces = true;
		GET_LINE();
		//only equal can go though
		if (!LINE_IS("property list uchar int vertex_indices"))
			goto plyreaderror;
		GET_LINE();
	} else if (LINE_IS("element tristrips 1")) {
		have_tstrips = true;
		GET_LINE();
		if (!LINE_IS("property list int int vertex_indices"))
			goto plyreaderror;
		GET_LINE();
	}else
	{
		result = sscanf(buf, "element range_grid %d", &numfaces);
		if(result == 1)
		{
			GET_LINE();
			//only equal can go though
			if (!LINE_IS("property list uchar int vertex_indices"))
				goto plyreaderror;
			GET_LINE();
		}
	}

	if (!LINE_IS("end_header")) {
		fprintf(stderr, "Expected \"end_header\"\n");
		goto plyreaderror;
	}


	// OK, we think we've parsed the header. Slurp in the actual data...
	//QTree_Node is a kind of struct defined in qsplat_make_qtree_v11.h
	//QTree_Node *root, *leaves;
	leaves = new QTree_Node[numleaves];

	printf(" Reading %d vertices... ", numleaves); fflush(stdout);
	if(pointcloud)
	{
		for (i=0; i < numleaves; i++) {
			
			GET_LINE();
			char delims[] = " ";
			//avoid corrupt stack 
			char *p[10];
			char *buffer = buf;
			int j  = 0;
			p[j] = strtok(buffer,delims);
			while(p[j] != NULL)
			{
				j++;
				p[j] = strtok(NULL,delims);
			}
			for(int k = 0 ; k < 3 ; k++)
			{
				//leaves[i].pos[k] = *(float *)p[k];
				leaves[i].pos[k] = atof(p[k]);
				if(have_colors)
				{
					leaves[i].norm[k] = atof(p[k+3]);
					leaves[i].col[k] = atoi(p[k+6]);
				}				
			}
		}
	}
	else
	{
		for (i=0; i < numleaves; i++) {

			//size_t fread(void*buffer,size_t size,size_t count,FILE*stream):
			//read number of count elements everyone's size is size bytes from stream to buffer
			//succedd return count(count*size) 
			
			if (!fread((void *)&(leaves[i].pos[0]), 12, 1, f))
				goto plyreaderror;
			//convert big-endian to small-endian
			if(isbig_endian)
			{
				FIX_FLOAT(leaves[i].pos[0]);
				FIX_FLOAT(leaves[i].pos[1]);
				FIX_FLOAT(leaves[i].pos[2]);
			}
			
			if (other_prop_len && !fread((void *)buf, other_prop_len, 1, f))
			//if (other_prop_len && !fread((void *)&(temp[0].col[0]), 3, 1, f))
				goto plyreaderror;

			if (have_colors) {
				//void*memcpy(void*dest,const void*src,int n):
				//copy n bytes from src to dest
				memcpy((void *)&(leaves[i].col[0]),
					buf + color_offset,
					sizeof(color));
			}
		}
	}
	
	printf("Done.\n");

	if (have_tstrips) {
		printf(" Reading triangle strips... "); fflush(stdout);

		if (!fread((void *)&tstripdatalen, 4, 1, f))
			goto plyreaderror;
		FIX_LONG(tstripdatalen);

		tstripdata = new int[tstripdatalen];
		if (!fread((void *)tstripdata, 4*tstripdatalen, 1, f))
			goto plyreaderror;
		for (int t=0; t < tstripdatalen; t++)
			FIX_LONG(tstripdata[t]);
	} else if (have_faces) {
		printf(" Reading %d faces... ", numfaces); fflush(stdout);
		faces = new face[numfaces];
		for (i=0; i < numfaces; i++) {
			if (!fread((void *)buf, 1, 1, f))
				goto plyreaderror;
			if (buf[0] != 3) {
				fprintf(stderr, "Non-triangle found in mesh.\n");
			}
			if (!fread((void *)faces[i], 12, 1, f))
				goto plyreaderror;
			if(isbig_endian)
			{
				FIX_LONG(faces[i][0]);
				FIX_LONG(faces[i][1]);
				FIX_LONG(faces[i][2]);
			}
		}
	}
	printf("Done.\n");
	if (tstripdatalen) {
		unpack_tstrips(tstripdatalen, tstripdata, numfaces, faces);
		delete [] tstripdata;
	}

	fgets(buf, 2, f);
	//int feof(FILE*stream):test the end of file indicator for the stream
	//if it is set return 1 else return 0
	if (!feof(f)) {
		fprintf(stderr, "Warning: ignored excess garbage at end of ply file.\n");
	}
	//int fclose(FILE*stream):free file pointer and buffer.
	//if close successfully return 0 ,else return EOF(-1)
	fclose(f);
	
	for(int i = 0 ; i < numleaves ; i++)
	{
		float p1,p2,p3,r,n1,n2,n3,c1,c2,c3;
		p1 = leaves[i].pos[0];
		p2 = leaves[i].pos[1];
		p3 = leaves[i].pos[2];
		r = leaves[i].r;
		n1 = leaves[i].norm[0];
		n2 = leaves[i].norm[1];
		n3 = leaves[i].norm[2];
		c1 = leaves[i].col[0];
		c2 = leaves[i].col[1];
		c3 = leaves[i].col[2];
		printf(" ");
	}

	return true;

plyreaderror:
	fclose(f);
	fprintf(stderr, "Error reading plyfile.\n");
	if (leaves) delete [] leaves;
	if (faces) delete [] faces;
	if (tstripdata) delete [] tstripdata;
	return false;
}

void printPoints()
{

}

// Find normal of a triangle with the given vertices
//point &p1 : float[3] &p1
static inline void FindNormal(const point &p1, const point &p2,
							  const point &p3, vec &n)
{
	//vec u:float[3] u
	vec u = { p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2] };
	vec v = { p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2] };

	//define in qsplat_util.h of 181 line                                      
	CrossProd(u, v, n);
}


// Find per-vertex normals
void find_normals(int numleaves, QTree_Node *leaves,
				  int numfaces, const face *faces)
{
	int i;
	printf("Computing normals... "); fflush(stdout);

	for (i=0; i < numleaves; i++)
		leaves[i].norm[0] = leaves[i].norm[1] = leaves[i].norm[2] = 0.0f;

	// For each face...
	for (i=0; i < numfaces; i++) {
		// Find normal ,vac:float[3]
		vec facenormal;
		FindNormal(leaves[faces[i][0]].pos,
			leaves[faces[i][1]].pos,
			leaves[faces[i][2]].pos,
			facenormal);

		// Accumulate. Note that facenormal is not unit-length, so
		// it is really an *area-weighted* normal.
		vec &n0 = leaves[faces[i][0]].norm;
		n0[0] += facenormal[0];
		n0[1] += facenormal[1];
		n0[2] += facenormal[2];

		vec &n1 = leaves[faces[i][1]].norm;
		n1[0] += facenormal[0];
		n1[1] += facenormal[1];
		n1[2] += facenormal[2];

		vec &n2 = leaves[faces[i][2]].norm;
		n2[0] += facenormal[0];
		n2[1] += facenormal[1];
		n2[2] += facenormal[2];
	}

	printf("Done.\n");
}


// Find a sphere that encloses the given triangle
// Based on GGems III V.1, by Fernando Lopez-Lopez
// and GGems I "Triangles" by Ronald Goldman
static inline void TriBoundingSphere(const float *p1,
									 const float *p2,
									 const float *p3,
									 //float *cent,
									 float &r)
{
	vec a = { p2[0] - p3[0], p2[1] - p3[1], p2[2] - p3[2] };
	vec b = { p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2] };
	vec c = { p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2] };

	//Dot(x,y) = return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
	float d1 = -Dot(b,c);
	float d2 = -Dot(c,a);
	float d3 = -Dot(a,b);

	// If triangle is obtuse, just want midpt of longest side
	if (d1 <= 0.0f) {

		//cent[0] = 0.5f * (p2[0] + p3[0]);
		//cent[1] = 0.5f * (p2[1] + p3[1]);
		//cent[2] = 0.5f * (p2[2] + p3[2]);
		r = 0.5f * Len(a);
		return;

	} else if (d2 <= 0.0f) {

		//cent[0] = 0.5f * (p3[0] + p1[0]);
		//cent[1] = 0.5f * (p3[1] + p1[1]);
		//cent[2] = 0.5f * (p3[2] + p1[2]);
		r = 0.5f * Len(b);
		return;

	} else if (d3 <= 0.0f) {

		//cent[0] = 0.5f * (p1[0] + p2[0]);
		//cent[1] = 0.5f * (p1[1] + p2[1]);
		//cent[2] = 0.5f * (p1[2] + p2[2]);
		r = 0.5f * Len(c);
		return;

	}

	// Else compute circumcircle
	float e1 = d2*d3;
	float e2 = d3*d1;
	float e3 = d1*d2;
	float e = e1+e2+e3;
	if (e == 0.0f) { r = 0.0f; return; }
	//float tmp = 0.5f / e;
	//cent[0] = ((e2+e3)*p1[0] + (e3+e1)*p2[0] + (e1+e2)*p3[0]) * tmp;
	//cent[1] = ((e2+e3)*p1[1] + (e3+e1)*p2[1] + (e1+e2)*p3[1]) * tmp;
	//cent[2] = ((e2+e3)*p1[2] + (e3+e1)*p2[2] + (e1+e2)*p3[2]) * tmp;
	r = 0.5f * sqrtf((d1+d2)*(d2+d3)*(d3+d1)/e);
}


// Figure out how big the splat at each point has to be.
void find_splat_sizes(int numleaves, QTree_Node *leaves,
					  int numfaces, const face *faces)
{
	int i;
	printf("Computing splat sizes... "); fflush(stdout);

	for (i=0; i < numleaves; i++)
		leaves[i].r = 0.0f;

	for (i=0; i < numfaces; i++) {
		float r;
		int i1 = faces[i][0];
		int i2 = faces[i][1];
		int i3 = faces[i][2];
		TriBoundingSphere(leaves[i1].pos,
			leaves[i2].pos,
			leaves[i3].pos,
			r);
		leaves[i1].r = max(leaves[i1].r, r);
		leaves[i2].r = max(leaves[i2].r, r);
		leaves[i3].r = max(leaves[i3].r, r);
	}
	printf("Done.\n");
}

// Figure out how big the splat at each point has to be.
void compute_splat_sizes(int numleaves, QTree_Node *leaves)
{	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	int dim = 3;
	int k = 1;
	int maxPts = numleaves;
	double eps = 0;

	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists
	nPts = 0 ;

	while(nPts < numleaves)
	{
		dataPts[nPts][0] = leaves[nPts].pos[0];
		dataPts[nPts][1] = leaves[nPts].pos[1];
		dataPts[nPts][2] = leaves[nPts].pos[2];
		leaves[nPts].r = 0.0f;
		nPts++;
	}

	kdTree = new ANNkd_tree(dataPts,nPts,dim);
	
	nPts = 0;
	while(nPts < numleaves)
	{
		queryPt[0] = dataPts[nPts][0];
		queryPt[1] = dataPts[nPts][1];
		queryPt[2] = dataPts[nPts][2];
		
		swap_point(nPts,kdTree,numleaves,leaves,true);
		kdTree->annkSearch(queryPt,k,nnIdx,dists,eps);
		leaves[nPts].r = sqrtf(dists[0]);
		swap_point(nPts,kdTree,numleaves,leaves,false);

		nPts++;
	}

	delete [] nnIdx;							// clean things up
    delete [] dists;
    delete kdTree;
	annClose();

	printf("Done.\n");
}

void swap_point(int nPts, ANNkd_tree *kdTree, int numleaves, QTree_Node *leaves, bool side)
{
	//if side ==true nPts_th point of kdTree.pts instead by nPts+1_th point of leaves
	//else nPts_th point restore within nPts_th point of leaves.
	if(side)
	{
		if((nPts+1) < numleaves)
		{
			kdTree->pts[nPts][0] = leaves[nPts+1].pos[0];
			kdTree->pts[nPts][1] = leaves[nPts+1].pos[1];
			kdTree->pts[nPts][2] = leaves[nPts+1].pos[2];
		}
		else
		{
			kdTree->pts[nPts][0] = leaves[0].pos[0];
			kdTree->pts[nPts][1] = leaves[0].pos[1];
			kdTree->pts[nPts][2] = leaves[0].pos[2];
		}
	}
	else
	{		
		kdTree->pts[nPts][0] = leaves[nPts].pos[0];
		kdTree->pts[nPts][1] = leaves[nPts].pos[1];
		kdTree->pts[nPts][2] = leaves[nPts].pos[2];	
	}
	
}

// Find really short edges in the mesh and merge their endpoints
void merge_nodes(int &numleaves, QTree_Node *leaves,
				 int &numfaces, face *faces,
				 bool havecolor, float thresh)
{
	int i;
	for (i=0; i < numleaves; i++) {
		leaves[i].m.refcount = 0;
		leaves[i].m.remap = i;
	}

	for (i=0; i < numfaces; i++) {
		int v1 = faces[i][0], v2 = faces[i][1], v3 = faces[i][2];
		leaves[v1].m.refcount = leaves[v2].m.refcount =
			leaves[v3].m.refcount = 1;
		if (thresh <= 0.0f)
			continue;
		while (v1 != leaves[v1].m.remap) v1 = leaves[v1].m.remap;
		while (v2 != leaves[v2].m.remap) v2 = leaves[v1].m.remap;
		while (v3 != leaves[v3].m.remap) v3 = leaves[v1].m.remap;

		float d12 = Dist(leaves[v1].pos, leaves[v2].pos);
		float d23 = Dist(leaves[v2].pos, leaves[v3].pos);
		float d31 = Dist(leaves[v3].pos, leaves[v1].pos);

		if ((d12 < thresh) + (d23 < thresh) + (d31 < thresh) >= 2) {
			// This entire triangle goes away...
			leaves[v1].m.remap = leaves[v2].m.remap =
				leaves[v3].m.remap = min(min(v1, v2), v3);
		} else if (d12 < thresh) {
			leaves[v1].m.remap = leaves[v2].m.remap = min(v1,v2);
		} else if (d23 < thresh) {
			leaves[v2].m.remap = leaves[v3].m.remap = min(v2,v3);
		} else if (d31 < thresh) {
			leaves[v3].m.remap = leaves[v1].m.remap = min(v3,v1);
		}
	}

	if (thresh <= 0.0f)
		return;

	for (i=0; i < numleaves; i++) {
		if (leaves[i].m.refcount == 0)
			continue;
		if (leaves[i].m.remap == i) {
			if (havecolor) {
				leaves[i].m.col_tmp[0] = leaves[i].col[0];
				leaves[i].m.col_tmp[1] = leaves[i].col[1];
				leaves[i].m.col_tmp[2] = leaves[i].col[2];
			}
			continue;
		}
		int j = leaves[i].m.remap;
		while (leaves[j].m.remap != j)
			j = leaves[j].m.remap;
		leaves[j].m.refcount++;
		leaves[j].pos[0] += leaves[i].pos[0];
		leaves[j].pos[1] += leaves[i].pos[1];
		leaves[j].pos[2] += leaves[i].pos[2];
		leaves[j].norm[0] += leaves[i].norm[0];
		leaves[j].norm[1] += leaves[i].norm[1];
		leaves[j].norm[2] += leaves[i].norm[2];
		if (havecolor) {
			leaves[j].m.col_tmp[0] += leaves[i].col[0];
			leaves[j].m.col_tmp[1] += leaves[i].col[1];
			leaves[j].m.col_tmp[2] += leaves[i].col[2];
		}
		leaves[i].m.refcount = 0;
	}

	for (i=0; i < numleaves; i++) {
		if (leaves[i].m.refcount < 2)
			continue;
		float x = 1.0f / leaves[i].m.refcount;
		leaves[i].pos[0] *= x;
		leaves[i].pos[1] *= x;
		leaves[i].pos[2] *= x;
		// Normal will get fixed later
		if (havecolor) {
			leaves[i].col[0] = min(max((unsigned char)(leaves[i].m.col_tmp[0] * x + 0.5), (unsigned char)0), (unsigned char)255);
			leaves[i].col[1] = min(max((unsigned char)(leaves[i].m.col_tmp[1] * x + 0.5), (unsigned char)0), (unsigned char)255);
			leaves[i].col[2] = min(max((unsigned char)(leaves[i].m.col_tmp[2] * x + 0.5), (unsigned char)0), (unsigned char)255);
		}
	}

	for (i=0; i < numfaces; i++) {
		while (faces[i][0] != leaves[faces[i][0]].m.remap)
			faces[i][0] = leaves[faces[i][0]].m.remap;
		while (faces[i][1] != leaves[faces[i][1]].m.remap)
			faces[i][1] = leaves[faces[i][1]].m.remap;
		while (faces[i][2] != leaves[faces[i][2]].m.remap)
			faces[i][2] = leaves[faces[i][2]].m.remap;
	}
}

