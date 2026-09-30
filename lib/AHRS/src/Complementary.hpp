#pragma once

class Complementary
{
public:
  void begin(
      float sampleRate,
      float tau,
      float state = 0.0f);

  float update(
      float rate,
      float position);

  float update(
      float rate,
      float position,
      float dt);

private:
  float _dt{0.001f};
  float _tau{0.5f};
  float _alpha{0.998f};
  float _state{0.0f};
};
