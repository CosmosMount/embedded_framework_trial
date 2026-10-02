#pragma once

namespace filter
{

class kalman_1d
{
public:
    void reset();
    void set_process_noise(float q);
    void set_measurement_noise(float r);
    float update(float input);
    float value() const { return value_; }

private:
    float last_p_ = 0.02f;
    float value_ = 0.0f;
    float q_ = 0.001f;
    float r_ = 0.543f;
};

} // namespace filter
