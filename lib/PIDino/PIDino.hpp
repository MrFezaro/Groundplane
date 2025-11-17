#ifndef PIDINO_HPP
#define PIDINO_HPP

// Welcome to my humble PIDino library. It ain't much, but it's honest work

class PIDino {
public:
    PIDino(float setPoint, float minValue, float maxValue, float Kp, float Ki, float Kd);

    float compute(float input);

    void setpoint(float newSetpoint);
    void Kp(float newKp);
    void Ki(float newKi);
    void Kd(float newKd);

private:
    float setPoint_;
    float Kp_;
    float Ki_;
    float Kd_;
    float minValue_;
    float maxValue_;
    float last_error_;
    float integral_;
    float derivative_;
    unsigned long lastTime_;
};



#endif //PIDINO_HPP