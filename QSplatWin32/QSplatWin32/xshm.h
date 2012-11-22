#ifndef XSHM_H
#define XSHM_H
/*
Szymon Rusinkiewicz

xshm.h
Routines for dealing with the X SHM (shared memory) extension.

Based on public-domain sample code (xshm.c) by Brian Paul.
*/

#include <X11/Xlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <X11/extensions/XShm.h>


static XShmSegmentInfo shminfo;


/*
 * Check if the X Shared Memory extension is available.
 * Return:  0 = not available
 *          1 = shared XImage support available
 *          2 = shared Pixmap support available also
 */
static int check_for_xshm( Display *display )
{
   int major, minor, ignore;
   Bool pixmaps;

   if (XQueryExtension( display, "MIT-SHM", &ignore, &ignore, &ignore )) {
      if (XShmQueryVersion( display, &major, &minor, &pixmaps )==True) {
	 return (pixmaps==True) ? 2 : 1;
      }
      else {
	 return 0;
      }
   }
   else {
      return 0;
   }
}



/*
 * Error handling.
 */
static int ErrorFlag = 0;

static int HandleXError( Display *dpy, XErrorEvent *event )
{
    ErrorFlag = 1;
    return 0;
}


/*
 * Allocate a shared memory XImage.
 */
static XImage *alloc_xshm_image( Display *dpy, Visual *vis,
				 int width, int height, int depth )
{
   XImage *img;

   /*
    * We have to do a _lot_ of error checking here to be sure we can
    * really use the XSHM extension.  It seems different servers trigger
    * errors at different points if the extension won't work.  Therefore
    * we have to be very careful...
    */

   img = XShmCreateImage( dpy, vis, depth,
                          ZPixmap, NULL, &shminfo,
                          width, height );
   if (img == NULL) {
      printf("XShmCreateImage failed!\n");
      return NULL;
   }

   shminfo.shmid = shmget( IPC_PRIVATE, img->bytes_per_line
			   * img->height, IPC_CREAT|0777 );
   if (shminfo.shmid < 0) {
      perror("shmget");
      printf( "alloc_xshm_image: Shared memory error (shmget), disabling.\n" );
      XDestroyImage( img );
      img = NULL;
      return NULL;
   }

   shminfo.shmaddr = img->data
                      = (char*)shmat( shminfo.shmid, 0, 0 );
   if (shminfo.shmaddr == (char *) -1) {
      perror("alloc_back_buffer");
      XDestroyImage( img );
      img = NULL;
      printf("shmat failed\n");
      return NULL;
   }

   shminfo.readOnly = False;
   ErrorFlag = 0;
   XSetErrorHandler( HandleXError );
   /* This may trigger the X protocol error we're ready to catch: */
   XShmAttach( dpy, &shminfo );
   XSync( dpy, False );

   if (ErrorFlag) {
      /* we are on a remote display, this error is normal, don't print it */
      XFlush( dpy );
      ErrorFlag = 0;
      XDestroyImage( img );
      shmdt( shminfo.shmaddr );
      shmctl( shminfo.shmid, IPC_RMID, 0 );
      return NULL;
   }

   shmctl( shminfo.shmid, IPC_RMID, 0 ); /* nobody else needs it */

   return img;
}



static void destroy_xshm_image( Display *display, XImage *img )
{
   XShmDetach( display, &shminfo );
   XDestroyImage( img );
   shmdt( shminfo.shmaddr );
}

#endif
