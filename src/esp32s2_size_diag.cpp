#include <ModelState.h>
#include <Control/GpsRescue.h>
#include <Control/Controller.h>

char diag_size_gpsrescue[sizeof(Espfc::Control::GpsRescue)];
char diag_size_failsafe[sizeof(Espfc::FailsafeState)];
char diag_size_gpsstate[sizeof(Espfc::GpsRescueState)];
char diag_size_controller[sizeof(Espfc::Control::Controller)];
char diag_size_modelstate[sizeof(Espfc::ModelState)];
