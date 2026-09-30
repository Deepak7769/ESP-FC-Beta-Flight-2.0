#pragma once

#include "Model.h"

namespace Espfc::Control {



class Actuator
{
public:
  Actuator(Model& model);

  int begin();
  int update();

#ifndef UNIT_TEST
private:
#endif

  void updateScaler();
  void updateArmingDisabled();
  void updateModeMask();
  void updateFailsafeLand();
  bool canActivateMode(FlightMode mode);
  bool attitudeEstimateHealthy() const;
  bool altitudeEstimateHealthy() const;
  void updateArmed();
  void updateAirMode();
  void updateBuzzer();
  void updateDynLpf();
  void updateRescueConfig();
  void updateLed();

  Model& _model;
bool _angleFaultLatched{false};
bool _altHoldFaultLatched{false};
};

} // namespace Espfc::Control
