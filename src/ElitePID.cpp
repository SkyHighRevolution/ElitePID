#include "ElitePID.h"

ElitePID::ElitePID(float kp, float ki, float kd, Action action)
    : _kp(kp),
      _ki(ki),
      _kd(kd),
      _beta(1.0f),
      _gamma(0.0f),
      _tauD(0.0f),
      _kt(1.0f),
      _ktAuto(true),
      _action(action),
      _mode(Mode::MANUAL),
      _sampleTimeUs(1000), // Default 1ms (1000us)
      _lastTime(0),
      _outMin(0.0f),
      _outMax(255.0f),
      _input(0.0f),
      _output(0.0f),
      _setpoint(0.0f),
      _integralSum(0.0f),
      _lastDerivState(0.0f),
      _dTerm(0.0f),
      _isFirstRun(true),
      _scheduler(nullptr),
      _telemetryEnabled(false),
      _telemetrySeq(0),
      _telemetryPTerm(0.0f),
      _telemetryITerm(0.0f),
      _telemetryDTerm(0.0f),
      _telemetryFFTerm(0.0f),
      _telemetryRawOutput(0.0f),
      _telemetryClampedOutput(0.0f),
      _telemetryGainsScheduled(false),
      _telemetrySensorFault(false) {
    
    Initialize();
}

float ElitePID::Compute(float input, float setpoint, float feedforward) {
    // 1. Sensor Failure Protection: NaN & Infinity Guard
    if (__builtin_expect(isnan(input) || isinf(input) || isnan(setpoint) || isinf(setpoint), 0)) {
        float outMin = ELITE_LOAD(_outMin);
        float outMax = ELITE_LOAD(_outMax);
        float safeOutput = (outMin <= 0.0f && outMax >= 0.0f) ? 0.0f : outMin;
        
        ELITE_STORE(_output, safeOutput);
        
        if (ELITE_LOAD(_telemetryEnabled)) {
            uint32_t seq = ELITE_LOAD(_telemetrySeq);
            ELITE_STORE(_telemetrySeq, seq + 1);
            
            ELITE_STORE(_telemetryPTerm, 0.0f);
            ELITE_STORE(_telemetryITerm, 0.0f);
            ELITE_STORE(_telemetryDTerm, 0.0f);
            ELITE_STORE(_telemetryFFTerm, 0.0f);
            ELITE_STORE(_telemetryRawOutput, safeOutput);
            ELITE_STORE(_telemetryClampedOutput, safeOutput);
            ELITE_STORE(_telemetryGainsScheduled, false);
            ELITE_STORE(_telemetrySensorFault, true);
            
            ELITE_STORE(_telemetrySeq, seq + 2);
        }
        return safeOutput;
    }

    uint32_t now = micros();
    uint32_t dt_us = now - ELITE_LOAD(_lastTime);
    uint32_t sampleTimeUs = ELITE_LOAD(_sampleTimeUs);
    
    // Store current state variables
    ELITE_STORE(_input, input);
    ELITE_STORE(_setpoint, setpoint);
    
    // Check if the configured sample time has elapsed
    if (__builtin_expect(dt_us < sampleTimeUs, 0)) {
        return ELITE_LOAD(_output);
    }
    
    // Check mode
    if (__builtin_expect(ELITE_LOAD(_mode) == Mode::MANUAL, 0)) {
        return ELITE_LOAD(_output);
    }
    
    // Dynamic parameter reads
    float kp = ELITE_LOAD(_kp);
    float ki = ELITE_LOAD(_ki);
    float kd = ELITE_LOAD(_kd);
    float beta = ELITE_LOAD(_beta);
    float gamma = ELITE_LOAD(_gamma);
    float tauD = ELITE_LOAD(_tauD);
    float kt = ELITE_LOAD(_kt);
    float outMin = ELITE_LOAD(_outMin);
    float outMax = ELITE_LOAD(_outMax);
    Action action = ELITE_LOAD(_action);
    
    float dt;
    if (__builtin_expect(dt_us == 0, 0)) {
        dt = 1e-6f;
    } else {
        dt = (float)dt_us * 1e-6f;
    }
    
    // Time Spike Protection: cap dt to prevent integral windup explosion during system lag
    float maxDt = 10.0f * (float)sampleTimeUs * 1e-6f;
    if (maxDt > 0.1f) {
        maxDt = 0.1f;
    }
    if (dt > maxDt) {
        dt = maxDt;
    }
    
    // 1. Dynamic Gain Scheduler Execution
    bool gainsScheduled = false;
    GainScheduler sched = ELITE_LOAD(_scheduler);
    if (__builtin_expect(sched != nullptr, 0)) {
        float error_now = setpoint - input;
        PIDGains scheduledGains = sched(error_now, input, kp, ki, kd);
        if (scheduledGains.kp != kp || scheduledGains.ki != ki || scheduledGains.kd != kd) {
            kp = scheduledGains.kp;
            ki = scheduledGains.ki;
            kd = scheduledGains.kd;
            gainsScheduled = true;
        }
    }
    
    // Pre-determine direction sign factor (branchless calculation)
    float sign = (action == Action::FORWARD) ? -1.0f : 1.0f;
    
    // 2. Proportional Term (2-DOF setpoint weighting)
    float error_p = sign * (beta * setpoint - input);
    float pTerm = kp * error_p;
    
    // 3. Derivative Term (2-DOF setpoint weighting + First-Order IIR Filtering)
    float dTermRaw = 0.0f;
    float currentDerivState = sign * (gamma * setpoint - input);
    
    if (__builtin_expect(!_isFirstRun, 1)) {
        dTermRaw = kd * (currentDerivState - _lastDerivState) / dt;
    }
    _lastDerivState = currentDerivState;
    
    // Filter coefficient alpha = dt / (dt + tauD).
    float alpha = dt / (dt + tauD);
    alpha = (tauD <= 0.0f) ? 1.0f : alpha;
    
    // Update filtered derivative term
    _dTerm += alpha * (dTermRaw - _dTerm);
    
    // 4. Integral Term Trial Update
    float error_i = sign * (setpoint - input);
    float trialIntegral = _integralSum + ki * error_i * dt;
    
    // Calculate raw PID output without feedforward
    float rawPIDOutput = pTerm + trialIntegral + _dTerm;
    
    // Calculate total raw output including dynamic feedforward injection
    float totalOutput = rawPIDOutput + feedforward;
    
    // FPU-accelerated branchless clamping
    float clampedOutput = fminf(fmaxf(totalOutput, outMin), outMax);
    
    // 5. Dynamic Back-Calculation Anti-Windup Integration
    // Decouple FF from the back-calculation by determining the available actuator headroom after FF is applied
    float clampedPIDOutput = clampedOutput - feedforward;
    float deltaPIDOutput = clampedPIDOutput - rawPIDOutput;
    _integralSum = trialIntegral + (kt * deltaPIDOutput * dt);
    
    // Clamp stored integral sum
    _integralSum = fminf(fmaxf(_integralSum, outMin), outMax);
    
    // Record telemetry values if enabled with a sequence lock pattern to prevent torn reads
    if (ELITE_LOAD(_telemetryEnabled)) {
        uint32_t seq = ELITE_LOAD(_telemetrySeq);
        ELITE_STORE(_telemetrySeq, seq + 1);
        
        ELITE_STORE(_telemetryPTerm, pTerm);
        ELITE_STORE(_telemetryITerm, _integralSum);
        ELITE_STORE(_telemetryDTerm, _dTerm);
        ELITE_STORE(_telemetryFFTerm, feedforward);
        ELITE_STORE(_telemetryRawOutput, totalOutput);
        ELITE_STORE(_telemetryClampedOutput, clampedOutput);
        ELITE_STORE(_telemetryGainsScheduled, gainsScheduled);
        ELITE_STORE(_telemetrySensorFault, false);
        
        ELITE_STORE(_telemetrySeq, seq + 2);
    }
    
    // Save state variables
    ELITE_STORE(_output, clampedOutput);
    ELITE_STORE(_lastTime, now);
    _isFirstRun = false;
    
    return clampedOutput;
}

void ElitePID::SetMode(Mode mode) {
    Mode prev = ELITE_EXCHANGE(_mode, mode);
    if (prev != mode && mode == Mode::AUTOMATIC) {
        Initialize();
    }
}

void ElitePID::SetControllerDirection(Action action) {
    ELITE_STORE(_action, action);
}

void ElitePID::SetTunings(float kp, float ki, float kd) {
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f) return;
    
    ELITE_STORE(_kp, kp);
    ELITE_STORE(_ki, ki);
    ELITE_STORE(_kd, kd);
    
    // Auto-calculate tracking gain for back-calculation: Kt = Ki / Kp
    // Safety guard: Gracefully default to 1.0f if Kp is too small (e.g. Pure Integral or PD loop)
    if (ELITE_LOAD(_ktAuto)) {
        if (kp > 1e-4f) {
            ELITE_STORE(_kt, ki / kp);
        } else {
            ELITE_STORE(_kt, 1.0f);
        }
    }
}

void ElitePID::SetSampleTimeUs(uint32_t sampleTimeUs) {
    if (sampleTimeUs > 0) {
        ELITE_STORE(_sampleTimeUs, sampleTimeUs);
    }
}

void ElitePID::SetOutputLimits(float minLimit, float maxLimit) {
    if (minLimit >= maxLimit) return;
    
    ELITE_STORE(_outMin, minLimit);
    ELITE_STORE(_outMax, maxLimit);
    
    // Clamp the integral sum
    _integralSum = fminf(fmaxf(_integralSum, minLimit), maxLimit);
}

void ElitePID::SetSetpointWeighting(float beta, float gamma) {
    ELITE_STORE(_beta, fminf(fmaxf(beta, 0.0f), 1.0f));
    ELITE_STORE(_gamma, fminf(fmaxf(gamma, 0.0f), 1.0f));
}

void ElitePID::SetDerivativeFilter(float tauD) {
    if (tauD >= 0.0f) {
        ELITE_STORE(_tauD, tauD);
    }
}

void ElitePID::SetTrackingGain(float kt) {
    if (kt >= 0.0f) {
        ELITE_STORE(_kt, kt);
        ELITE_STORE(_ktAuto, false);
    }
}

void ElitePID::SetTrackingGainAuto(bool autoCalc) {
    ELITE_STORE(_ktAuto, autoCalc);
    if (autoCalc) {
        float kp = ELITE_LOAD(_kp);
        float ki = ELITE_LOAD(_ki);
        if (kp > 1e-4f) {
            ELITE_STORE(_kt, ki / kp);
        } else {
            ELITE_STORE(_kt, 1.0f);
        }
    }
}

void ElitePID::SetGainScheduler(GainScheduler scheduler) {
    ELITE_STORE(_scheduler, scheduler);
}

ElitePID::PIDTelemetry ElitePID::GetTelemetry() const {
    PIDTelemetry tel;
    uint32_t seq1, seq2;
    do {
        seq1 = ELITE_LOAD(_telemetrySeq);
        tel.pTerm = ELITE_LOAD(_telemetryPTerm);
        tel.iTerm = ELITE_LOAD(_telemetryITerm);
        tel.dTerm = ELITE_LOAD(_telemetryDTerm);
        tel.ffTerm = ELITE_LOAD(_telemetryFFTerm);
        tel.rawOutput = ELITE_LOAD(_telemetryRawOutput);
        tel.clampedOutput = ELITE_LOAD(_telemetryClampedOutput);
        tel.gainsScheduled = ELITE_LOAD(_telemetryGainsScheduled);
        tel.sensorFault = ELITE_LOAD(_telemetrySensorFault);
        seq2 = ELITE_LOAD(_telemetrySeq);
    } while ((seq1 & 1) != 0 || seq1 != seq2);
    return tel;
}

void ElitePID::Reset() {
    _integralSum = 0.0f;
    _lastDerivState = 0.0f;
    _dTerm = 0.0f;
    _isFirstRun = true;
    ELITE_STORE(_lastTime, micros());
}

void ElitePID::Initialize() {
    float sign = (ELITE_LOAD(_action) == Action::FORWARD) ? -1.0f : 1.0f;
    
    // Correct scoping of the local sign variable to avoid redeclarations
    _lastDerivState = sign * (ELITE_LOAD(_gamma) * ELITE_LOAD(_setpoint) - ELITE_LOAD(_input));
    
    // Bumpless Transfer Initialization: set integral sum based on current output minus proportional contribution
    float kp = ELITE_LOAD(_kp);
    float beta = ELITE_LOAD(_beta);
    float input = ELITE_LOAD(_input);
    float setpoint = ELITE_LOAD(_setpoint);
    
    float error_p = sign * (beta * setpoint - input);
    _integralSum = ELITE_LOAD(_output) - (kp * error_p);
    
    float outMin = ELITE_LOAD(_outMin);
    float outMax = ELITE_LOAD(_outMax);
    _integralSum = fminf(fmaxf(_integralSum, outMin), outMax);
    
    _dTerm = 0.0f;
    _isFirstRun = true;
    ELITE_STORE(_lastTime, micros());
}
