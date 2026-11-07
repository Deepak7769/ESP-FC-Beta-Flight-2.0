#include "Pid.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

Pid::Pid()
    : rate(1.0f), dt(1.0f), Kp(0.1), Ki(0.f), Kd(0.f), Kf(0.0f), iLimitLow(-0.3f), iLimitHigh(0.3f), iReset(0.0f),
      oLimitLow(-1.f), oLimitHigh(1.f), pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f), error(0.f), iTermError(0.f),
      pTerm(0.f), iTerm(0.f), dTerm(0.f), fTerm(0.f), prevMeasurement(0.f), prevError(0.f), prevSetpoint(0.f),
      ftermDerivative(true), outputSaturated(false), itermRelax(ITERM_RELAX_OFF), itermRelaxFactor(1.0f),
      itermRelaxBase(0.f)
{
}

void Pid::begin()
{
  dt = 1.f / std::max(rate, 1.f);
}

void Pid::resetIterm()
{
  iTerm = iReset;
}

float FAST_CODE_ATTR Pid::update(
    float setpoint,
    float measurement,
    float tpaFactor,
    bool tpaP,
    float pMultiplier,
    float iGainAdd)
{
  // The scheduler supplies bounded measured sample time through dt.
  // Keep rate unchanged: filter coefficients are configured at nominal Hz.
  const float sampleDt = (std::isfinite(dt) && dt > 0.f)
      ? dt : 1.f / std::max(rate, 1.f);
  const float derivativeRate = 1.f / sampleDt;

  error = setpoint - measurement;
   const float tpa =
    std::clamp(tpaFactor, 0.0f, 1.0f);
    
  // P-term
  pTerm = Kp * error * pScale;
    
  pTerm = ptermFilter.update(pTerm);
    
  if (tpaP)
{
  pTerm *= tpa;
}

  // Optional P boost used by the guarded Anti-Gravity bench path.
  pTerm *= std::max(pMultiplier, 0.0f);

  // I-term
  iTermError = error;

  // Anti-Gravity uses an additive I coefficient, matching the
  // (Ki + itermAccelerator) structure used by Betaflight.
  const float effectiveIGain =
      Ki * iScale +
      std::max(iGainAdd, 0.0f);

  if (effectiveIGain > 0.f)
  {
    // Keep I-term relax filter history advancing even at mixer saturation.
    if (itermRelax)
    {
      const bool increasing = (iTerm > 0 && iTermError > 0) || (iTerm < 0 && iTermError < 0);
      const bool incrementOnly = itermRelax == ITERM_RELAX_RP_INC || itermRelax == ITERM_RELAX_RPY_INC;
      itermRelaxBase = setpoint - itermRelaxFilter.update(setpoint);
      itermRelaxFactor = std::max(0.0f, 1.0f - std::abs(Utils::toDeg(itermRelaxBase)) * 0.025f);
      if (!incrementOnly || increasing) iTermError *= itermRelaxFactor;
    }

    const float proposedIterm = iTerm + effectiveIGain * iTermError * sampleDt;
    // Actuator saturation blocks further windup, but never blocks unwinding.
    // Clamping to [min(old I, 0), max(old I, 0)] also prevents crossing zero.
    iTerm = outputSaturated
        ? std::clamp(proposedIterm, std::min(iTerm, 0.f), std::max(iTerm, 0.f))
        : proposedIterm;
    iTerm = std::clamp(iTerm, iLimitLow, iLimitHigh);
  }
  else
  {
    iTerm = 0; // zero integral
  }

  // D-term
  if (Kd > 0.f && dScale > 0.f)
  {
    // dTerm = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasurement - measure) * (1.f - dGamma)) / dt);
    dTerm = Kd * dScale * ((prevMeasurement - measurement) * derivativeRate);
    dTerm = dtermNotchFilter.update(dTerm);
    dTerm = dtermFilter.update(dTerm);
    dTerm = dtermFilter2.update(dTerm);
      dTerm *= tpa;
  }
  else
  {
    dTerm = 0;
  }

  // F-term
  if (Kf > 0.f && fScale > 0.f)
  {
    if (ftermDerivative)
    {
      fTerm = Kf * fScale * (setpoint - prevSetpoint) * derivativeRate;
    }
    else
    {
      fTerm = Kf * fScale * setpoint;
    }
    fTerm = ftermFilter.update(fTerm);
  }
  else
  {
    fTerm = 0;
  }

  prevMeasurement = measurement;
  prevError = error;
  prevSetpoint = setpoint;

  return std::clamp(pTerm + iTerm + dTerm + fTerm, oLimitLow, oLimitHigh);
}

} // namespace Espfc::Control
