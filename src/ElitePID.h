#ifndef ELITE_PID_H
#define ELITE_PID_H

#include <Arduino.h>

/**
 * @macro ELITEPID_THREAD_SAFE
 * @brief If defined, ElitePID uses std::atomic variables and memory order constraints
 *        to ensure thread-safe execution in multi-core (e.g., FreeRTOS) or ISR environments.
 *        If undefined (default), standard floats are used for zero overhead on single-core MCUs.
 */
// #define ELITEPID_THREAD_SAFE

#ifdef ELITEPID_THREAD_SAFE
#include <atomic>
#define ELITE_ATOMIC(type) std::atomic<type>
#define ELITE_LOAD(var) var.load(std::memory_order_relaxed)
#define ELITE_STORE(var, val) var.store(val, std::memory_order_relaxed)
#define ELITE_EXCHANGE(var, val) var.exchange(val, std::memory_order_relaxed)
#else
#define ELITE_ATOMIC(type) volatile type
#define ELITE_LOAD(var) var
#define ELITE_STORE(var, val) (var = val)
#define ELITE_EXCHANGE(var, val) ([&](){ auto tmp = var; var = val; return tmp; }())
#endif

// Forward declaration of the Tuner template class
template <size_t TuningCycles>
class ElitePIDTuner;

class ElitePID {
public:
    enum class Mode : uint8_t {
        MANUAL = 0,
        AUTOMATIC = 1
    };

    enum class Action : uint8_t {
        FORWARD = 0, // Direct Acting: Cooling (PV increase -> Output increase)
        REVERSE = 1  // Reverse Acting: Heating (PV increase -> Output decrease)
    };

    enum class TuningRule : uint8_t {
        ZIEGLER_NICHOLS_PID = 0,
        ZIEGLER_NICHOLS_PI = 1,
        ZIEGLER_NICHOLS_P = 2
    };

    // Gains structure returned by the gain scheduler
    struct PIDGains {
        float kp;
        float ki;
        float kd;
    };

    // Telemetry structure for advanced Serial Plotter debugging
    struct PIDTelemetry {
        float pTerm;
        float iTerm;
        float dTerm;
        float ffTerm;
        float rawOutput;
        float clampedOutput;
        bool gainsScheduled; // Tells the user if default gains were overridden by the scheduler
        bool sensorFault;    // Tells the operator of NaN/Infinity input/setpoint fault
    };

    // Gain scheduling callback signature
    using GainScheduler = PIDGains (*)(float error, float input, float kp, float ki, float kd);

    /**
     * @brief Construct a new ElitePID controller.
     * 
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param action Controller action (FORWARD/REVERSE).
     */
    ElitePID(float kp, float ki, float kd, Action action = Action::REVERSE);

    /**
     * @brief Run the fast-path control loop calculation. Designed to be branchless 
     *        and FPU-optimized. Safe to call inside high-frequency ISRs.
     * 
     * @param input Current process variable (PV).
     * @param setpoint Desired target setpoint (SP).
     * @param feedforward Predictive feedforward control signal (FF).
     * @return Clamped control output (CO).
     */
    float Compute(float input, float setpoint, float feedforward = 0.0f);

    // Mode and Direction Settings
    void SetMode(Mode mode);
    Mode GetMode() const { return ELITE_LOAD(_mode); }
    void SetControllerDirection(Action action);
    Action GetControllerDirection() const { return ELITE_LOAD(_action); }

    // Gain & Parameter Setters/Getters
    void SetTunings(float kp, float ki, float kd);
    float GetKp() const { return ELITE_LOAD(_kp); }
    float GetKi() const { return ELITE_LOAD(_ki); }
    float GetKd() const { return ELITE_LOAD(_kd); }

    // Set sample time in microseconds (Compute will skip if time has not elapsed)
    void SetSampleTimeUs(uint32_t sampleTimeUs);
    uint32_t GetSampleTimeUs() const { return ELITE_LOAD(_sampleTimeUs); }

    // Dynamic output clamping limits
    void SetOutputLimits(float minLimit, float maxLimit);
    float GetOutputMin() const { return ELITE_LOAD(_outMin); }
    float GetOutputMax() const { return ELITE_LOAD(_outMax); }

    // 2-DOF Setpoint Weighting Configuration (Beta for P, Gamma for D)
    void SetSetpointWeighting(float beta, float gamma);
    float GetBeta() const { return ELITE_LOAD(_beta); }
    float GetGamma() const { return ELITE_LOAD(_gamma); }

    // Derivative Filtering time constant (in seconds)
    void SetDerivativeFilter(float tauD);
    float GetDerivativeFilter() const { return ELITE_LOAD(_tauD); }

    // Dynamic Back-calculation Tracking Gain Configuration
    void SetTrackingGain(float kt);
    void SetTrackingGainAuto(bool autoCalc);
    float GetTrackingGain() const { return ELITE_LOAD(_kt); }

    // Dynamic Gain Scheduler Registration
    void SetGainScheduler(GainScheduler scheduler);

    // Telemetry Config & Reading
    void EnableTelemetry(bool enable) { ELITE_STORE(_telemetryEnabled, enable); }
    bool IsTelemetryEnabled() const { return ELITE_LOAD(_telemetryEnabled); }
    PIDTelemetry GetTelemetry() const;

    // Dynamic Variable accessors for background logging / async tuner
    float GetInput() const { return ELITE_LOAD(_input); }
    float GetOutput() const { return ELITE_LOAD(_output); }
    float GetSetpoint() const { return ELITE_LOAD(_setpoint); }
    void SetOutput(float outputVal) { ELITE_STORE(_output, outputVal); }

    // Manual Reset of States
    void Reset();

private:
    template <size_t TuningCycles>
    friend class ElitePIDTuner;

    // Fast-path parameters (ISR-safe atomics if thread safety is enabled)
    ELITE_ATOMIC(float) _kp;
    ELITE_ATOMIC(float) _ki;
    ELITE_ATOMIC(float) _kd;
    
    ELITE_ATOMIC(float) _beta;  // SP proportional weight
    ELITE_ATOMIC(float) _gamma; // SP derivative weight
    
    ELITE_ATOMIC(float) _tauD;  // Filter time constant (sec)
    ELITE_ATOMIC(float) _kt;    // Back-calculation gain
    ELITE_ATOMIC(bool)  _ktAuto;

    ELITE_ATOMIC(Action) _action;
    ELITE_ATOMIC(Mode)   _mode;

    ELITE_ATOMIC(uint32_t) _sampleTimeUs;
    ELITE_ATOMIC(uint32_t) _lastTime;

    ELITE_ATOMIC(float) _outMin;
    ELITE_ATOMIC(float) _outMax;

    // Process State Variables
    ELITE_ATOMIC(float) _input;
    ELITE_ATOMIC(float) _output;
    ELITE_ATOMIC(float) _setpoint;

    // Hot-path local variables (Only written/read inside Compute() context)
    float _integralSum;
    float _lastDerivState;
    float _dTerm;
    bool  _isFirstRun;

    ELITE_ATOMIC(GainScheduler) _scheduler;

    // Telemetry storage variables (Lightweight and thread-safe if enabled)
    ELITE_ATOMIC(bool)  _telemetryEnabled;
    ELITE_ATOMIC(uint32_t) _telemetrySeq;
    ELITE_ATOMIC(float) _telemetryPTerm;
    ELITE_ATOMIC(float) _telemetryITerm;
    ELITE_ATOMIC(float) _telemetryDTerm;
    ELITE_ATOMIC(float) _telemetryFFTerm;
    ELITE_ATOMIC(float) _telemetryRawOutput;
    ELITE_ATOMIC(float) _telemetryClampedOutput;
    ELITE_ATOMIC(bool)  _telemetryGainsScheduled;
    ELITE_ATOMIC(bool)  _telemetrySensorFault;

    void Initialize();
};

/**
 * @class ElitePIDTuner
 * @brief Decoupled Asynchronous Auto-Tuning State Machine.
 */
template <size_t TuningCycles = 5>
class ElitePIDTuner {
public:
    enum class State : uint8_t {
        IDLE = 0,
        SWING_UP = 1,
        SWING_DOWN = 2,
        COMPLETED = 3,
        FAILED = 4
    };

    ElitePIDTuner(ElitePID& pid) 
        : _pid(pid), 
          _state(State::IDLE), 
          _relayAmplitude(10.0f), 
          _hysteresis(0.5f), 
          _timeoutUs(30000000), 
          _cycleCount(0), 
          _toggleCount(0), 
          _tuningRule(ElitePID::TuningRule::ZIEGLER_NICHOLS_PID),
          _tuningStartTime(0),
          _tuningBias(0.0f),
          _relayStateHigh(false),
          _lastToggleTime(0),
          _peakMax(-1e9f),
          _peakMin(1e9f),
          _tMinus2(0),
          _tMinus1(0) {
        
        for (size_t i = 0; i < TuningCycles; i++) {
            _periods[i] = 0.0f;
            _amplitudes[i] = 0.0f;
        }
    }

    void Start(float relayAmplitude, float hysteresis, uint32_t timeoutMs, ElitePID::TuningRule rule = ElitePID::TuningRule::ZIEGLER_NICHOLS_PID) {
        _relayAmplitude = relayAmplitude;
        _hysteresis = hysteresis;
        _timeoutUs = timeoutMs * 1000;
        _tuningRule = rule;
        
        _tuningStartTime = micros();
        _cycleCount = 0;
        _toggleCount = 0;
        _peakMax = -1e9f;
        _peakMin = 1e9f;
        
        _tuningBias = _pid.GetOutput();
        _pid.SetMode(ElitePID::Mode::MANUAL);
        
        float input = _pid.GetInput();
        float setpoint = _pid.GetSetpoint();
        
        bool initiallyBelow = (input < setpoint);
        if (_pid.GetControllerDirection() == ElitePID::Action::REVERSE) {
            _relayStateHigh = initiallyBelow;
        } else {
            _relayStateHigh = !initiallyBelow;
        }
        
        ApplyRelayOutput();
        
        _lastToggleTime = _tuningStartTime;
        _state = _relayStateHigh ? State::SWING_UP : State::SWING_DOWN;
    }

    void Cancel() {
        _state = State::FAILED;
        _pid.SetMode(ElitePID::Mode::MANUAL);
    }

    State GetState() const { return _state; }
    bool IsFinished() const { return _state == State::COMPLETED || _state == State::FAILED; }

    void Tick() {
        if (_state == State::IDLE || _state == State::COMPLETED || _state == State::FAILED) {
            return;
        }

        uint32_t now = micros();
        
        if (now - _tuningStartTime > _timeoutUs) {
            _state = State::FAILED;
            _pid.SetMode(ElitePID::Mode::MANUAL);
            return;
        }

        float input = _pid.GetInput();
        float setpoint = _pid.GetSetpoint();
        
        if (input > _peakMax) _peakMax = input;
        if (input < _peakMin) _peakMin = input;

        bool toggle = false;
        ElitePID::Action direction = _pid.GetControllerDirection();

        if (direction == ElitePID::Action::REVERSE) {
            if (_relayStateHigh && (input > setpoint + _hysteresis)) {
                _relayStateHigh = false;
                toggle = true;
                _state = State::SWING_DOWN;
            } else if (!_relayStateHigh && (input < setpoint - _hysteresis)) {
                _relayStateHigh = true;
                toggle = true;
                _state = State::SWING_UP;
            }
        } else {
            if (_relayStateHigh && (input < setpoint - _hysteresis)) {
                _relayStateHigh = false;
                toggle = true;
                _state = State::SWING_DOWN;
            } else if (!_relayStateHigh && (input > setpoint + _hysteresis)) {
                _relayStateHigh = true;
                toggle = true;
                _state = State::SWING_UP;
            }
        }

        if (toggle) {
            ApplyRelayOutput();
            
            if (_cycleCount == 0 && _toggleCount == 0) {
                _tMinus1 = now;
                _toggleCount = 1;
                _peakMax = input;
                _peakMin = input;
                return;
            }
            
            if (_toggleCount == 1) {
                _tMinus2 = _tMinus1;
                _tMinus1 = now;
                _toggleCount = 2;
                _peakMax = input;
                _peakMin = input;
                return;
            }
            
            float periodSec = (float)(now - _tMinus2) * 1e-6f;
            float amplitudeVal = (_peakMax - _peakMin) * 0.5f;

            if (_cycleCount < TuningCycles) {
                _periods[_cycleCount] = periodSec;
                _amplitudes[_cycleCount] = amplitudeVal;
                _cycleCount++;
            }

            _tMinus2 = _tMinus1;
            _tMinus1 = now;
            
            _peakMax = input;
            _peakMin = input;

            if (_cycleCount >= TuningCycles) {
                FinishTuning();
            }
        }
    }

private:
    ElitePID& _pid;
    State _state;
    float _relayAmplitude;
    float _hysteresis;
    uint32_t _timeoutUs;
    uint32_t _tuningStartTime;
    
    float _tuningBias;
    bool _relayStateHigh;
    size_t _cycleCount;
    int _toggleCount;
    uint32_t _lastToggleTime;
    float _peakMax;
    float _peakMin;
    
    uint32_t _tMinus2;
    uint32_t _tMinus1;

    ElitePID::TuningRule _tuningRule;

    float _periods[TuningCycles];
    float _amplitudes[TuningCycles];

    void ApplyRelayOutput() {
        float outputVal = _relayStateHigh ? (_tuningBias + _relayAmplitude) : (_tuningBias - _relayAmplitude);
        float outMin = _pid.GetOutputMin();
        float outMax = _pid.GetOutputMax();
        outputVal = fminf(fmaxf(outputVal, outMin), outMax);
        _pid.SetOutput(outputVal);
    }

    void FinishTuning() {
        float sumAmp = 0.0f;
        float sumPer = 0.0f;
        for (size_t i = 0; i < _cycleCount; i++) {
            sumAmp += _amplitudes[i];
            sumPer += _periods[i];
        }
        float avgAmp = sumAmp / (float)_cycleCount;
        float avgPer = sumPer / (float)_cycleCount;

        if (avgAmp < 1e-4f) avgAmp = 1e-4f;
        if (avgPer < 1e-4f) avgPer = 1e-4f;

        float Ku = (4.0f * _relayAmplitude) / (3.14159265f * avgAmp);
        float Tu = avgPer;

        float kp = 0.0f, ki = 0.0f, kd = 0.0f;

        if (_tuningRule == ElitePID::TuningRule::ZIEGLER_NICHOLS_PID) {
            kp = 0.6f * Ku;
            ki = 2.0f * kp / Tu;
            kd = 0.125f * kp * Tu;
        } else if (_tuningRule == ElitePID::TuningRule::ZIEGLER_NICHOLS_PI) {
            kp = 0.45f * Ku;
            ki = 1.2f * kp / Tu;
            kd = 0.0f;
        } else {
            kp = 0.5f * Ku;
            ki = 0.0f;
            kd = 0.0f;
        }

        _pid.SetTunings(kp, ki, kd);
        _pid.SetMode(ElitePID::Mode::AUTOMATIC);
        _state = State::COMPLETED;
    }
};

#endif // ELITE_PID_H
