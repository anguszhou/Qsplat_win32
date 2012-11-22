/** Header file generated with fdesign on Sat Jul 22 01:27:30 2000.**/

#ifndef FD_qsplat_gui_h_
#define FD_qsplat_gui_h_

/** Callbacks, globals and object handlers **/
extern void DesiredRateCB(FL_OBJECT *, long);
extern void FileMenuCB(FL_OBJECT *, long);
extern void OptionsMenuCB(FL_OBJECT *, long);
extern void CommandsMenuCB(FL_OBJECT *, long);
extern void ShortcutCB(FL_OBJECT *, long);
extern void DriverMenuCB(FL_OBJECT *, long);
extern void HelpMenuCB(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *qsplat_gui;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *theGLCanvas;
	FL_OBJECT *DesiredRate;
	FL_OBJECT *FileMenu;
	FL_OBJECT *OptionsMenu;
	FL_OBJECT *CommandsMenu;
	FL_OBJECT *ShortcutButton;
	FL_OBJECT *DriverMenu;
	FL_OBJECT *HelpMenu;
	FL_OBJECT *RateBar;
	FL_OBJECT *StatusBar;
} FD_qsplat_gui;

extern FD_qsplat_gui * create_form_qsplat_gui(void);

#endif /* FD_qsplat_gui_h_ */
