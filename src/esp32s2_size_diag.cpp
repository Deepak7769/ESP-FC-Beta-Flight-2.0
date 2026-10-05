#include <ModelState.h>
#include <Control/GpsRescue.h>
#include <Control/Controller.h>
#define CAT2(a,b) a##b
#define CAT(a,b) CAT2(a,b)
char CAT(diag_size_gpsrescue_, sizeof(Espfc::Control::GpsRescue)) [sizeof(Espfc::Control::GpsRescue)];
char CAT(diag_size_failsafe_, sizeof(Espfc::FailsafeState)) [sizeof(Espfc::FailsafeState)];
char CAT(diag_size_gpsstate_, sizeof(Espfc::GpsRescueState)) [sizeof(Espfc::GpsRescueState)];
char CAT(diag_size_controller_, sizeof(Espfc::Control::Controller)) [sizeof(Espfc::Control::Controller)];
char CAT(diag_size_modelstate_, sizeof(Espfc::ModelState)) [sizeof(Espfc::ModelState)];
