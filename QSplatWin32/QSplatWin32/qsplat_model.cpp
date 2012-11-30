/*
Szymon Rusinkiewicz

qsplat_model.cpp
The QSplat model class.  This knows all about how to read and
display .qs files.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include "qsplat_util.h"
#include "qsplat_model.h"
#include "qsplat_normquant.h"
#include "qsplat_spherequant.h"
#include "qsplat_colorquant.h"

#ifdef WIN32
# include <windows.h>
# include <winbase.h>
# include <io.h>
# include "qsplat_gui_win32.h"
#else
# include <unistd.h>
# include <sys/mman.h>
# define CloseHandle close
# include "qsplat_gui_xforms.h"
#endif

#include <GL/gl.h>


// This .h file defines the actual implementation of traversing and
// drawing.
#include "qsplat_traverse_v11.h"


// A few random #defines
#define QSPLAT_MAGIC "QSplat"
#define MINSIZE_MIN 1.0f
#define MINSIZE_MAX 50.0f
#define MINSIZE_REFINE_MULTIPLIER 0.7f


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


// Try to open a file.  Returns a new QSplat_Model, or NULL if can't open it.
QSplat_Model *QSplat_Model::Open(const char *modelfilename)
{
	static bool initialized = false;
	if (!initialized) {
		Init();
		initialized = true;
	}

#ifdef WIN32
	OFSTRUCT fdOFSTRUCT;
	//OpenFile:return file handle if success else return HFILE_ERROR.
	HFILE hfd = OpenFile(modelfilename, &fdOFSTRUCT, OF_READ);
	if (hfd == HFILE_ERROR) {
		Error("Couldn't open ", modelfilename);
		return NULL;
	}
	//FileLen:return file length bytes
	off_t len = FileLen(hfd);
	if (len <= 0) {
		_lclose(hfd);
		Error("Couldn't open ", modelfilename);
		return NULL;
	} else if (len < 40) {
		_lclose(hfd);
		Error(modelfilename, " is not a QSplat file");
		return NULL;
	}
	_lclose(hfd);
//createFile:could open,create file,pipeline,service...
	//return file handle if success else return INVALID_HANDLE_VALUE
	HANDLE fd = CreateFile(modelfilename,
			GENERIC_READ, 0, NULL, OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL | FILE_FLAG_RANDOM_ACCESS, NULL);
#else
	fprintf(stderr, "Opening file %s... ", modelfilename); fflush(stderr);
	int fd = open(modelfilename, O_RDONLY);
	if (fd == -1) {
		perror("open");
		return NULL;
	}
	off_t len = FileLen(fd);
	if (len < 0) {
		close(fd);
		Error("Couldn't open ", modelfilename);
		return NULL;
	} else if (len < 40) {
		close(fd);
		Error(modelfilename, " is not a QSplat file");
		return NULL;
	}
#endif

	unsigned char *mem_start = NULL, *map_start = NULL;
	//MapFile:set map_start which is start address of file mapping object in memory
	//and return true if success else return false.
	if (!MapFile(fd, len, &mem_start, &map_start)) {
		CloseHandle(fd);
		return NULL;
	}


	// Build the new object
	QSplat_Model *q = new QSplat_Model(std::string(modelfilename),
					   mem_start, map_start,
					   fd, len);

	//BuildFragmentList:set Center and radius of all the fragments together
	if (!q->BuildFragmentList(modelfilename)) {
		delete q;
		return NULL;
	}
	return q;
}


// Reset internal state so that next time the model will be drawn at lowest res
void QSplat_Model::reset_rate()
{
	minsize = minsize_save = MINSIZE_MAX;
}


// Adjust resolution so that the next drawing pass will be faster by
// approximately the given factor.
void QSplat_Model::adjust_rate(float factor)
{
	minsize *= sqrtf(factor);
	minsize = min(max(minsize, MINSIZE_MIN), MINSIZE_MAX);
}


// Can we refine any further?
bool QSplat_Model::can_refine()
{
	return (minsize > MINSIZE_MIN);
}


// Are we already at the coarsest (reasonable) level of detail?
bool QSplat_Model::coarsest()
{
	return (minsize >= MINSIZE_MAX - 0.1f);
}


// We're going to start auto-refining the model now
void QSplat_Model::start_refine()
{
	minsize_save = minsize;
}


// Set up state so that the next time this model is drawn it is at a
// higher resolution
void QSplat_Model::refine()
{
	minsize *= MINSIZE_REFINE_MULTIPLIER;
	minsize = floor(minsize);
	minsize = min(max(minsize, MINSIZE_MIN), MINSIZE_MAX);
}


// Stop refining - next time we go back to the last "interactive" resolution
void QSplat_Model::stop_refine()
{
	minsize = minsize_save;
}


// Destructor - unmap the file
QSplat_Model::~QSplat_Model()
{
	fragments.clear();
#ifdef WIN32
	UnmapViewOfFile(map_start);
#else
	munmap((char *)map_start, len);
#endif
	if (mem_start)
		delete [] mem_start;
	CloseHandle(fd);
}


// Initialize global data structures for QSplat
void QSplat_Model::Init()
{
	QSplat_ColorQuant::Init();
	QSplat_NormQuant::Init();
	QSplat_SphereQuant::Init();
}


// Determine length of an open file
off_t QSplat_Model::FileLen(HFILE fd)
{
#ifdef WIN32
	BY_HANDLE_FILE_INFORMATION fdInfo;
	if (!GetFileInformationByHandle((HANDLE)fd, &fdInfo))
		return -1;
	return fdInfo.nFileSizeLow;
#else
	struct stat statbuf;
	if (fstat(fd, &statbuf) == -1)
		return -1;
	return statbuf.st_size;
#endif
}


// Memory map a file, returning pointers to the start of the allocated
// memory region and the start of the map
// This returns true if we were able to do the mmap
bool QSplat_Model::MapFile(HANDLE fd, off_t len,
			   unsigned char **mem_start,
			   unsigned char **map_start)
{
#ifdef WIN32
	static HANDLE fdMapping;
	char mapName[16];
	int mapNum = 0;
	UINT error;
	do {
		if (fdMapping)
			CloseHandle(fdMapping);
		sprintf(mapName, "QSplat%d", mapNum++);
		//CreateFileMapping:create a new file mapping core object
		//return new file mapping object handle(long) if success else return 0
		//if handle returned is exist(success status),GetLastError set ERROR_ALREADY_EXISTS.
		fdMapping = CreateFileMapping(fd, NULL, PAGE_READONLY,
					      0, len, mapName);
	} while ((error = GetLastError()) == ERROR_ALREADY_EXISTS);

	if (!fdMapping || error != NO_ERROR) {
		char fdError[32];
		sprintf(fdError, "Mapping Error: %d", error);
		MessageBox(NULL, fdError, NULL, MB_OK);
		return false;
	}
	//MapViewOfFile:return start address of file mapping object if success else return 0
	//and set GetLastError.
	*map_start = (unsigned char *) MapViewOfFile(fdMapping, FILE_MAP_READ,
						     0, 0, len);
	if (*map_start)
		return true;

	char errorstr[32];
	sprintf(errorstr, "Can't Allocate Memory! (%d)\n", GetLastError());
	MessageBox(NULL, errorstr, NULL, MB_OK | MB_ICONHAND);
	return false;

#else

	// First, we just try the mmap
	*map_start = (unsigned char *)
			mmap(0, len, PROT_READ, MAP_SHARED, fd, 0);

	// If that worked, we're done
	if (*map_start != MAP_FAILED)
		return true;


	// OK, that didn't work.  However, on certain operating systems
	// (cough, cough, IRIX, cough, cough), it is sometimes the case
	// that for some bizarre reason the mmap doesn't work right off
	// even though we officially do have a large-enough virtual address
	// space to map in everything.  To try to work around this, we'll
	// try to allocate a large-enough chunk of memory, and mmap there

	// Figure out how much memory to allocate
#ifdef sgi
	int align = sysconf(_SC_MMAP_FIXED_ALIGNMENT);
#else
	int align = sysconf(_SC_PAGE_SIZE);
#endif

	// To work around yet another bug, we temporarily allocate some memory
	// here, else sometimes we don't get to use the heap (!) after the mmap
	unsigned char *ugly_hack = new unsigned char[16*1024*1024];

	// Allocate memory and align the pointer correctly
	*mem_start = new unsigned char[len + align];
	long tmp = long(*mem_start);
	tmp = align * long(ceil((double) tmp / align));
	*map_start = (unsigned char *)tmp;


	// Do the mmap
	*map_start = (unsigned char *)
			mmap( (char *)(*map_start), len,
			      PROT_READ, MAP_SHARED | MAP_FIXED,
			      fd, 0 );
	delete [] ugly_hack;
	if (*map_start == MAP_FAILED) {
		// Give up
		perror("mmap");
		delete [] (*mem_start);
		*mem_start = *map_start = 0;
		return false;
	}

	return true;
#endif
}


// A single file can have multiple fragments - the file just looks like
// the files for the individual fragments catted together
bool QSplat_Model::BuildFragmentList(const char *filename)
{
	float xmin=3.3e33f, xmax=-3.3e33f;
	float ymin=3.3e33f, ymax=-3.3e33f;
	float zmin=3.3e33f, zmax=-3.3e33f;

	comments = "File "; comments += filename; comments += "\n";

	const unsigned char *here = map_start;
	while (here < map_start+len) {

		if (map_start+len-here < 40) {
			Error("Couldn't read header of ", filename);
			return false;
		}
	//int strncmp(char*s1,char*s2,int n):compare the first n chars of s1 and s2
	//if s1<s2 return <0 ; s1==s2 return =0 ; s1>s2 return >0
	//(value is ASCLL value of difference of first different char)
		//QSPLAT_MAGIC "QSplat"
		if (strncmp((const char *)here, QSPLAT_MAGIC, 6) != 0) {
			Error(filename, " is not a QSplat file");
			return false;
		}
		char buf[3];
		//QSPLAT_FILE_VERSION 11
		sprintf(buf, "%02d", QSPLAT_FILE_VERSION);
		if (here[6] != buf[0] || here[7] != buf[1]) {
			Error(filename, " was made for a different version of QSplat");
			return false;
		}

		int fraglen = * (int *)(here+8);
		FIX_LONG(fraglen);
		if (here+fraglen > map_start+len) {
			Error(filename, " is truncated");
			return false;
		}

		if ((*(unsigned char *)(here+19)) & 2) {
			comments.append((const char *)(here+20), fraglen-20);
			here += fraglen;
			continue;
		}

		int points = * (int *)(here+12);
		FIX_LONG(points);
		leaf_points += points;

		float x = * (float *)(here+20); FIX_FLOAT(x);
		float y = * (float *)(here+24); FIX_FLOAT(y);
		float z = * (float *)(here+28); FIX_FLOAT(z);
		float r = * (float *)(here+32); FIX_FLOAT(r);
		xmin = min(xmin, x-r);  xmax = max(xmax, x+r);
		ymin = min(ymin, y-r);  ymax = max(ymax, y+r);
		zmin = min(zmin, z-r);  zmax = max(zmax, z+r);

		//std::vector<const unsigned char *> fragments;
		fragments.push_back(here);
		here += fraglen;

	}

	center[0] = 0.5f * (xmin + xmax);
	center[1] = 0.5f * (ymin + ymax);
	center[2] = 0.5f * (zmin + zmax);
	radius = 0.5f*sqrtf(sqr(xmax-xmin) + sqr(ymax-ymin) + sqr(zmax-zmin));

	char buf[255];
	sprintf(buf, "%d leaf points\n", leaf_points);
	comments += buf;
#ifndef WIN32
	fprintf(stderr, buf);
#endif
	return true;
}

