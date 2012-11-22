#ifndef QSPLAT_MODEL_H
#define QSPLAT_MODEL_H
/*
Szymon Rusinkiewicz

qsplat_model.h
The QSplat model class.  This knows all about how to read and
display .qs files.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#ifdef WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
# include <commctrl.h>//common control header file
# define off_t int
#else
# define HANDLE int
# define HFILE int
#endif
#include "qsplat_util.h"
#include <vector>
#include <string>


class QSplat_Model {
private:
	unsigned char *mem_start;
	unsigned char *map_start;
	HANDLE fd;
	off_t len;

	static off_t FileLen(HFILE f);
	static bool MapFile(HANDLE f, off_t len,
			    unsigned char **, unsigned char **);

	std::vector<const unsigned char *> fragments;
	bool BuildFragmentList(const char *filename);

	static void Init();
	QSplat_Model(const std::string &filename_,
		     unsigned char *mem_start_,
		     unsigned char *map_start_,
		     HANDLE fd_,
		     off_t len_) :
		filename(filename_), leaf_points(0),
		mem_start(mem_start_), map_start(map_start_),
		fd(fd_), len(len_)
	{
		Init();
		reset_rate();
	}

public:
	static QSplat_Model *Open(const char *modelfilename);
	std::string filename;
	std::string comments;

	void reset_rate();
	void adjust_rate(float factor);
	bool can_refine();
	bool coarsest();
	void start_refine();
	void refine();
	void stop_refine();

	bool draw();
	float traceray(int x, int y, float cutoff);

	// Center and radius of all the fragments together
	float center[3];
	float radius;

	// Total number of points at the leaf nodes
	int leaf_points;

	// Minimum splat size
	float minsize;
	float minsize_save;

	~QSplat_Model();
};

#endif
