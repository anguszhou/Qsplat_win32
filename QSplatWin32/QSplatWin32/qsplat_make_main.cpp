// QSlpatMake.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include "qsplat_make_qtree_v11.h"
#include "qsplat_make_from_mesh.h"
#include "qsplat_make_main.h"


// Version stamp
const char *QSPLATMAKE_VERSION = "1.0";


static void usage(const char *myname)
{
	fprintf(stderr, "Usage: %s [-m threshold] point/mesh[type] in.ply out.qs\n", myname);
	system("PAUSE");
	exit(1);
}

int build_ply_to_qs(const char* para,const char *buildFileName)
{
	
#ifdef WIN32
	_fmode = _O_BINARY;
#endif
	printf("This is QSplatMake version %s.\n", QSPLATMAKE_VERSION);
	/*
	// Parse command-line params
	//strncasecmp(char*s1,char*s2,size_t n):compare first n chars of s1 and s2
	//ignore case and return 0 if equal else if s1>s2 return >0 else if s1<s2 return <0;
	if ((argc < 4) ||
		!strncasecmp(argv[1], "-h", 2) ||
		!strncasecmp(argv[1], "--h", 3))
		usage(argv[0]);
	bool is_pointfile = false;
	int i = strncasecmp(argv[1], "point" , 5);
	if(!strncasecmp(argv[1], "point" , 5))
		is_pointfile = true;
	const char *infilename = argv[2];
	const char *outfilename = argv[3];
	float threshold = 0;
	printf("argv2:%s\n",infilename);
	printf("argv3:%s\n",outfilename);

	if (argc >= 5 && !strncasecmp(argv[1], "-m", 2)) {
		threshold = atof(argv[2]);
		infilename = argv[3];
		outfilename = argv[4];
	}
	*/
	float threshold = 0;
	bool is_pointfile = false;
	if(!strncasecmp(para, "point" , 5))
		is_pointfile = true;

	const char *infilename = buildFileName;
	char str = '.' ,  qsFilename[256]={' '};
	const char* ptr = strrchr(buildFileName, str);
	int len = strlen(buildFileName)-strlen(ptr)+1;
	memcpy(qsFilename , buildFileName , len);
	strcat(qsFilename , "qs"); 
	const char *outfilename = qsFilename;

	// Read the .ply file
	int numleaves, numfaces;
	face *faces;
	bool havecolor;
	QTree_Node *leaves;
	std::string comments;

	if (!read_ply(infilename, numleaves, leaves, numfaces, faces, havecolor, comments)) {
		fprintf(stderr, "Couldn't read input file %s\n", infilename);
		return 0;
	}
	if (numleaves < 4) {
		fprintf(stderr, "Ummm...  That's an awfully small mesh you've got there...\n");
		return 0;
	}
	if(!is_pointfile)
		if (numfaces < 4) {
			fprintf(stderr, "Ummm... I need a *mesh* as input.  That means triangles 'n stuff...\n");
			return 0;
		}
	
	// Compute per-vertex normals
	if(!is_pointfile)
	{
		find_normals(numleaves, leaves, numfaces, faces);
		// Merge nodes
		merge_nodes(numleaves, leaves, numfaces, faces, havecolor, threshold);

		// Compute initial splat sizes
		find_splat_sizes(numleaves, leaves, numfaces, faces);
	}
	else
	{
		compute_splat_sizes(numleaves, leaves);
	}
	
	

	// Don't need face data any more
	delete [] faces;

	// Initialize the tree
	QTree qt(numleaves, leaves, havecolor);

	// Build the tree...
	qt.BuildTree();

	// ... and write it out
	qt.Write(outfilename, comments);

	return 1;
}

