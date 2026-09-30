#pragma once

#include "Control/Rates.h"
#include "Model.h"

namespace Espfc::Control {

class Controller
{
public:
  Controller(Model& model);
  int begin();
  int reload(ModelChangeEvent event);
  int update();

  void outerLoopRobot();
  void innerLoopRobot();
  void outerLoop();
  void innerLoop();

  inline float getTpaFactor() const;
  inline void resetIterm();
  float calculateSetpointRate(int axis, float input) const;
  float calcualteAltHoldSetpoint() const;


private:
  void reloadFilter();
  void reloadPid();

  // Shared assisted-controller update.
  // Angle V2 is authoritative; AltHold/LAND remain
  // feature-gated through AssistedModeV2.h.
  void updateAssistedModes();

  float calculatePilotClimbRate() const;
  void updateAntiGravity();

  // Core controller dependencies.
  Model& _model;
  Rates _rates;
  Utils::Filter _speedFilter;
  Utils::Filter _antiGravityFilter;

  bool _antiGravityPrimed = false;
  float _antiGravityPrevThrottle = 0.0f;

  // Angle V2 transition state.
  bool _angleV2WasActive = false;

  // AltHold V2 transition state.
  bool _altHoldWasActive = false;

  // Tracks ownership of thrust output so AltHold V2
  // can enter without a thrust discontinuity.
  bool _altHoldV2OutputWasActive = false;

  float _altHoldAltitudeTarget = 0.0f;
  float _altHoldVerticalRateTarget = 0.0f;

  uint32_t _assistedLastUpdateUs = 0;
};

} // namespace Espfc::Control
