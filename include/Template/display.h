#pragma once
#include "main.h"
using namespace pros;

void simple_screen_task(bool rainbow);

void print_curvepoints(std::vector<CurvePoint> points);
void print_point(Point point, int line);

void init_map();
void map_curvepoints(std::vector<CurvePoint> points);
void update_map();
void map_task();

// Dashboard: motor temps/connection, robot coordinates, autonomous selector.

enum class DisplayTab {
    MOTORS = 0,
    POSITION = 1,
    AUTON_SELECT = 2,
    // SAO = 3, // SAO tab disabled, see sao_gallery.cpp
};

enum class AutonRoutine {
    left = 0,
    left2 = 1,
    right = 2,
    right2 = 3
};

extern AutonRoutine selected_auton;

void dashboard_task();
void start_dashboard();
void dashboard_handle_touch();
void dashboard_draw_tab_bar();
void dashboard_draw_motors_tab();
void dashboard_draw_position_tab();
// clear_first=false skips the black fills and just repaints the boxes on top of
// themselves -- used by the periodic self-heal redraw so it doesn't flicker.
void dashboard_draw_auton_tab(bool clear_first = true);
// void dashboard_draw_sao_tab(); // SAO tab disabled, see sao_gallery.cpp

// SAO gallery: desktop screenshots you can flip through with Back/Next (see sao_gallery.cpp).
// Disabled -- sao_gallery.cpp body is wrapped in #if 0.
// extern const int SAO_IMAGE_W;
// extern const int SAO_IMAGE_H;
// extern const int SAO_IMAGE_COUNT;
// extern const uint32_t* const sao_images[];