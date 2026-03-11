#ifndef _Motor_h
#define _Motor_h
#include <Arduino.h>

enum GM_driver {
    DRIVER2WIRE,    
    DRIVER2WIRE_NO_INVERT,
    DRIVER2WIRE_PWM,
    DRIVER3WIRE,
    RELAY2WIRE,
};

#define _GM_SMOOTH_PRD 50

template <GM_driver GM_TYPE, uint8_t GM_RES = 8>
class GMotor2 {
public:
    GMotor2(uint8_t pa, uint8_t pb, uint8_t pc = 255) : pinA(pa), pinB(pb), pinC(pc) {
        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
        if (pinC != 255) pinMode(pinC, OUTPUT);
        setAll(0);
    }
    
    void setMinDuty(uint16_t mduty) {
        minD = mduty;
    }
    
    void setMinDutyPerc(uint16_t mduty) {
        mduty = constrain(mduty, 0, 100);
        minD = (int32_t)getMax() * mduty / 100;
    }
    
    void reverse(bool r) {
        rev = r ? -1 : 1;
    }
    
    void stop() {
        setSpeed(0);
    }
    
    void brake() {
        setAll(1);
        speed = duty = 0;
    }
    
    void setSpeed(int16_t s) {
        speed = s;
        if (!smooth) duty = speed;
        run(duty);
    }
    
    void setSpeedPerc(int16_t s) {
        s = constrain(s, -100, 100);
        setSpeed((int32_t)getMax() * s / 100);
    }

    void setDeadtime(uint16_t us) {
        dead = us;
    }
    
    void tick() {
        if ((speed || duty) && smooth && millis() - tmr >= _GM_SMOOTH_PRD) {
            tmr = millis();
            if (abs(duty - speed) > ds) duty += (duty < speed) ? ds : -ds;
            else duty = speed;
            run(duty);
        }
    }
    
    void setSmoothSpeed(uint8_t s) {
        ds = s;
    }
    
    void setSmoothSpeedPerc(uint8_t s) {
        s = constrain(s, 0, 100);
        ds = (int32_t)getMax() * s / 100;
    }
    
    void smoothMode(bool mode) {
        smooth = mode;
    }
    
    int8_t getState() {
        return dir;
    }
    
    int16_t getSpeed() {
        return duty;
    }
    
private:
    int16_t getMax() {
        return (1 << GM_RES) - 1;
    }
    
    void run(int16_t sp) {
        if (!sp) return setAll(0);
        int8_t ndir = (sp > 0) ? rev : -rev;
        if (dead && ndir != dir) {
            setAll(0);
            delayMicroseconds(dead);
        }
        dir = ndir;
        int16_t maxD = getMax();
        sp = constrain(sp, -maxD, maxD);
        if (minD) sp = (int32_t)sp * (maxD - minD) >> GM_RES;
        sp = abs(sp) + minD;
        if (GM_RES > 8 && sp == 255) sp++;

        switch (GM_TYPE) {
        case DRIVER2WIRE:
            digitalWrite(pinA, dir < 0);
            analogWrite(pinB, (dir > 0) ? sp : (maxD - sp));
            break;
            
        case DRIVER2WIRE_NO_INVERT:
            digitalWrite(pinA, dir < 0);
            analogWrite(pinB, sp);
            break;
            
        case DRIVER2WIRE_PWM:
            if (dir > 0) {
                digitalWrite(pinA, 0);
                analogWrite(pinB, sp);
            } else {
                analogWrite(pinA, sp);
                digitalWrite(pinB, 0);
            }
            break;
            
        case DRIVER3WIRE:
            digitalWrite(pinA, dir < 0);
            digitalWrite(pinB, dir > 0);
            analogWrite(pinC, sp);
            break;
            
        case RELAY2WIRE:
            digitalWrite(pinA, dir < 0);
            digitalWrite(pinB, dir > 0);
            break;
        }
    }
    
    void setAll(uint8_t val) {
        digitalWrite(pinA, val);
        digitalWrite(pinB, val);
        if (pinC != 255) digitalWrite(pinC, val);
        dir = 0;
    }
    
    const uint8_t pinA, pinB, pinC;
    bool smooth = 0;
    int8_t dir = 0, rev = 1;
    uint8_t dead = 0;
    int16_t minD = 0, speed = 0, duty = 0, ds = 20;
    uint32_t tmr = 0;
};
#endif
