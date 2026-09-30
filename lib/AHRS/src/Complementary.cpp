#include "Complementary.hpp"

#include <cmath>

void Complementary::begin(
    float sampleRate,
    float tau,
    float state)
{
  if (!std::isfinite(sampleRate) ||
      sampleRate <= 0.0f)
  {
    sampleRate = 1000.0f;
  }

  if (!std::isfinite(tau) ||
      tau <= 0.0f)
  {
    tau = 0.5f;
  }

  _dt =
      1.0f / sampleRate;

  _tau =
      tau;

  _alpha =
      _tau /
      (_tau + _dt);

  _state =
      std::isfinite(state)
          ? state
          : 0.0f;
}

float Complementary::update(
    float rate,
    float position)
{
  return update(
      rate,
      position,
      _dt);
}

float Complementary::update(
    float rate,
    float position,
    float dt)
{
  if (!std::isfinite(dt) ||
      dt <= 0.0f)
  {
    dt =
        _dt;
  }

  if (!std::isfinite(rate))
  {
    rate =
        0.0f;
  }

  if (!std::isfinite(position))
  {
    position =
        _state;
  }

  const float alpha =
      _tau /
      (_tau + dt);

  const float nextState =
      alpha *
          (_state + rate * dt) +
      (1.0f - alpha) *
          position;

  if (std::isfinite(nextState))
  {
    _state =
        nextState;
  }

  return _state;
}
