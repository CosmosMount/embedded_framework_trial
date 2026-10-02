#pragma once

namespace filter
{

class iir
{
public:
    explicit iir(float omega_n, float zeta, float dt);

    float update(float input);
    void reset();

private:
    float b0_ = 0.0f;
    float b1_ = 0.0f;
    float a1_ = 0.0f;
    float a2_ = 0.0f;
    float x1_ = 0.0f;
    float x2_ = 0.0f;
    float y1_ = 0.0f;
    float y2_ = 0.0f;
};

} // namespace filter
