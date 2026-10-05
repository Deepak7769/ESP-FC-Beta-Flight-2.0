#include <ModelState.h>
#include <Control/GpsRescue.h>
#include <Control/Controller.h>
#define STR2(x) #x
#define STR(x) STR2(x)
#pragma message("SIZE_GpsRescue=" STR(sizeof(Espfc::Control::GpsRescue)))
#pragma message("SIZE_FailsafeState=" STR(sizeof(Espfc::FailsafeState)))
#pragma message("SIZE_GpsRescueState=" STR(sizeof(Espfc::GpsRescueState)))
#pragma message("SIZE_Controller=" STR(sizeof(Espfc::Control::Controller)))
#pragma message("SIZE_ModelState=" STR(sizeof(Espfc::ModelState)))
int esp32s2_size_diag = 0;
