/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "xforms_gui.h"

FD_qsplat_gui *create_form_qsplat_gui(void)
{
  FL_OBJECT *obj;
  FD_qsplat_gui *fdui = (FD_qsplat_gui *) fl_calloc(1, sizeof(*fdui));

  fdui->qsplat_gui = fl_bgn_form(FL_NO_BOX, 540, 600);
  obj = fl_add_box(FL_FRAME_BOX,0,0,540,600,"");
  fdui->theGLCanvas = obj = fl_add_glcanvas(FL_NORMAL_CANVAS,10,40,520,520,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_gravity(obj, FL_NorthWest, FL_SouthEast);
  fdui->DesiredRate = obj = fl_add_valslider(FL_HOR_NICE_SLIDER,390,570,140,20,"Frame Rate");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT);
    fl_set_object_gravity(obj, FL_South, FL_SouthEast);
    fl_set_object_resize(obj, FL_RESIZE_X);
    fl_set_object_callback(obj,DesiredRateCB,0);
    fl_set_slider_precision(obj, 1);
    fl_set_slider_bounds(obj, 1, 20);
    fl_set_slider_value(obj, 8);
    fl_set_slider_step(obj, 0.5);
    fl_set_slider_increment(obj, 0.5, 0.5);
  fdui->FileMenu = obj = fl_add_menu(FL_PULLDOWN_MENU,10,10,50,20,"File");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_gravity(obj, FL_NorthWest, FL_NorthWest);
    fl_set_object_callback(obj,FileMenuCB,0);
    fl_addto_menu(obj, "Open...");
    fl_addto_menu(obj, "Exit");
  fdui->OptionsMenu = obj = fl_add_menu(FL_PULLDOWN_MENU,60,10,70,20,"Options");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_gravity(obj, FL_NorthWest, FL_NorthWest);
    fl_set_object_callback(obj,OptionsMenuCB,0);
    fl_addto_menu(obj, "Shiny");
    fl_set_menu_item_mode(obj, 1, FL_PUP_CHECK);
    fl_addto_menu(obj, "Backface Cull");
    fl_set_menu_item_mode(obj, 2, FL_PUP_CHECK);
    fl_addto_menu(obj, "Show Light");
    fl_set_menu_item_mode(obj, 3, FL_PUP_BOX);
    fl_addto_menu(obj, "Show Refinement");
    fl_set_menu_item_mode(obj, 4, FL_PUP_CHECK);
    fl_addto_menu(obj, "Auto-spin");
    fl_set_menu_item_mode(obj, 5, FL_PUP_CHECK);
  fdui->CommandsMenu = obj = fl_add_menu(FL_PULLDOWN_MENU,200,10,90,20,"Commands");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_gravity(obj, FL_NorthWest, FL_NorthWest);
    fl_set_object_callback(obj,CommandsMenuCB,0);
    fl_addto_menu(obj, "Fullscreen");
    fl_set_menu_item_mode(obj, 1, FL_PUP_BOX);
    fl_addto_menu(obj, "Reset Viewer");
  fdui->ShortcutButton = obj = fl_add_button(FL_HIDDEN_BUTTON,270,570,60,20,"");
    fl_set_button_shortcut(obj,"bdfqsQ^Q^X\033\040",1);
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_callback(obj,ShortcutCB,0);
  fdui->DriverMenu = obj = fl_add_menu(FL_PULLDOWN_MENU,130,10,70,20,"Driver");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_gravity(obj, FL_NorthWest, FL_NorthWest);
    fl_set_object_callback(obj,DriverMenuCB,0);
    fl_addto_menu(obj, "OpenGL - Points");
    fl_set_menu_item_mode(obj, 1, FL_PUP_RADIO);
    fl_addto_menu(obj, "OpenGL - Round Points");
    fl_set_menu_item_mode(obj, 2, FL_PUP_RADIO);
    fl_addto_menu(obj, "OpenGL - Quads");
    fl_set_menu_item_mode(obj, 3, FL_PUP_RADIO);
    fl_addto_menu(obj, "OpenGL - Circles");
    fl_set_menu_item_mode(obj, 4, FL_PUP_RADIO);
    fl_addto_menu(obj, "OpenGL - Ellipses");
    fl_set_menu_item_mode(obj, 5, FL_PUP_RADIO);
    fl_addto_menu(obj, "OpenGL - Small Ellipses");
    fl_set_menu_item_mode(obj, 6, FL_PUP_RADIO);
    fl_addto_menu(obj, "OpenGL - Spheres (slow!)");
    fl_set_menu_item_mode(obj, 7, FL_PUP_RADIO);
    fl_addto_menu(obj, "Software - Z Buffer (GL blit)");
    fl_set_menu_item_mode(obj, 8, FL_PUP_RADIO);
    fl_addto_menu(obj, "Software - Z Buffer");
    fl_set_menu_item_mode(obj, 9, FL_PUP_RADIO);
    fl_addto_menu(obj, "Software - Tiled (GL blit)");
    fl_set_menu_item_mode(obj, 10, FL_PUP_RADIO);
    fl_addto_menu(obj, "Software - Tiled");
    fl_set_menu_item_mode(obj, 11, FL_PUP_RADIO);
    fl_addto_menu(obj, "Software - Optimal (GL blit)");
    fl_set_menu_item_mode(obj, 12, FL_PUP_RADIO);
    fl_addto_menu(obj, "Software - Optimal");
    fl_set_menu_item_mode(obj, 13, FL_PUP_RADIO);
  fdui->HelpMenu = obj = fl_add_menu(FL_PULLDOWN_MENU,290,10,50,20,"Help");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_gravity(obj, FL_NorthWest, FL_NorthWest);
    fl_set_object_callback(obj,HelpMenuCB,0);
    fl_addto_menu(obj, "About QSplat");
    fl_addto_menu(obj, "About this model");
  fdui->RateBar = obj = fl_add_text(FL_NORMAL_TEXT,10,570,150,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_gravity(obj, FL_SouthWest, FL_South);
    fl_set_object_resize(obj, FL_RESIZE_X);
  fdui->StatusBar = obj = fl_add_text(FL_NORMAL_TEXT,160,570,150,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_gravity(obj, FL_South, FL_South);
    fl_set_object_resize(obj, FL_RESIZE_X);
  fl_end_form();

  fdui->qsplat_gui->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

