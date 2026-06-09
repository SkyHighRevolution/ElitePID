#include <ElitePID.h>

// PID State variables
float motorInput = 0.0f;       // Process Variable (PV)
float motorOutput = 0.0f;      // Control Output (CO)
float motorSetpoint = 150.0f;  // Setpoint (SP)

// Create the elite PID controller with initial zero gains
// We use Action::REVERSE (Reverse acting motor speed/heating control: PV below SP -> Output increases)
ElitePID myPID(0.0f, 0.0f, 0.0f, ElitePID::Action::REVERSE);

// Instantiate the decoupled asynchronous auto-tuner template
// Templated with 5 oscillation cycles for averaging
ElitePIDTuner<5> myTuner(myPID);

// Simulated first-order motor parameters
float simulatedSpeed = 0.0f;
const float motorGain = 1.2f;       // K
const float motorTimeConst = 0.8f;  // Tau (seconds)
uint32_t lastSimTimeUs = 0;

// High-speed gain scheduling callback conforming to the new PIDGains signature
// Designed to return a PIDGains struct instead of altering references.
ElitePID::PIDGains advancedGainScheduler(float error, float input, float kp, float ki, float kd) {
  ElitePID::PIDGains scheduled = {kp, ki, kd};
  
  // If speed error is very large, dynamically increase Proportional gain by 50%
  // to push the motor back to track aggressively.
  if (fabsf(error) > 40.0f) {
    scheduled.kp = kp * 1.5f;
  }
  return scheduled;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Elite 32-bit hardware FPU optimizations setup:
  // 1. Output limits aligned to standard 8-bit PWM (0 to 255)
  myPID.SetOutputLimits(0.0f, 255.0f);
  
  // 2. High-speed loop execution rate: 1,000 microseconds (1 millisecond / 1000Hz)
  myPID.SetSampleTimeUs(1000);
  
  // 3. Set derivative filter time constant: TauD = 0.01 seconds (10ms IIR LPF)
  // This filters sensor noise while preserving phase margin for stability.
  myPID.SetDerivativeFilter(0.01f);
  
  // 4. Set 2-DOF Weights: Beta = 0.8f (reduced SP proportional kick), Gamma = 0.0f (no SP derivative kick)
  myPID.SetSetpointWeighting(0.8f, 0.0f);

  // 5. Enable the new PID Telemetry feature.
  // This allocates lightweight state trackers inside Compute() for easy serial plotting.
  myPID.EnableTelemetry(true);

  Serial.println("=== ElitePID Decoupled High-Speed Motor Control ===");
  Serial.println("Initiating Asynchronous Relay Feedback Tuning...");

  // Start tuning: Relay amplitude = 80, Hysteresis = 1.5, Timeout = 30 seconds
  myTuner.Start(80.0f, 1.5f, 30000, ElitePID::TuningRule::ZIEGLER_NICHOLS_PID);
  
  lastSimTimeUs = micros();
}

void loop() {
  uint32_t now = micros();
  
  // 1. Run physical motor simulation at a constant high-frequency (1000Hz / 1ms step)
  if (now - lastSimTimeUs >= 1000) {
    float dt = (float)(now - lastSimTimeUs) * 1e-6f;
    lastSimTimeUs = now;
    
    // First-order system physics: dy/dt = (K * u - y) / Tau
    float dSpeed = (motorGain * motorOutput - simulatedSpeed) / motorTimeConst;
    simulatedSpeed += dSpeed * dt;
    
    // Add realistic high-frequency noise typical of magnetic encoders
    motorInput = simulatedSpeed + ((random(-100, 100) / 100.0f) * 0.4f);
  }

  // 2. Asynchronous Auto-Tuning state machine handler
  if (!myTuner.IsFinished()) {
    // Tick the tuner asynchronously. This handles peak detection and state changes 
    // without introducing overhead in the active PID loop.
    myTuner.Tick();
    
    // Update active output to match relay commands set by the tuner
    motorOutput = myPID.GetOutput();
    
    // Log tuning oscillations to Serial (for plotter)
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 20) {
      lastLog = millis();
      Serial.print("TUNING_OSCILLATION | PV: "); Serial.print(motorInput);
      Serial.print(" | Setpoint: "); Serial.print(motorSetpoint);
      Serial.print(" | Actuator: "); Serial.println(motorOutput);
    }
  } else {
    // Tuning is finished. Execute active high-speed PID control loop.
    static bool setupCompleted = false;
    if (!setupCompleted) {
      setupCompleted = true;
      Serial.println("\n--- Tuning Phase Completed Successfully! ---");
      Serial.print("Tuned Parameters -> Kp: "); Serial.print(myPID.GetKp(), 4);
      Serial.print(" | Ki: "); Serial.print(myPID.GetKi(), 4);
      Serial.print(" | Kd: "); Serial.println(myPID.GetKd(), 4);
      
      // Register the dynamic gain scheduler callback
      myPID.SetGainScheduler(advancedGainScheduler);
      Serial.println("Gain scheduler registered (aggressive correction enabled for large errors).");
      Serial.println("Switching to active 2-DOF loop...\n");
      
      // Step input to test response
      motorSetpoint = 180.0f;
    }
    
    // Run the fast-path Compute loop (microsecond-precision)
    motorOutput = myPID.Compute(motorInput, motorSetpoint);
    
    // 3. Serial Plotter Logging using the new Telemetry structural API
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 50) {
      lastLog = millis();
      // Read telemetry structural data block
      ElitePID::PIDTelemetry telemetry = myPID.GetTelemetry();
      
      Serial.print("PV:"); Serial.print(motorInput);
      Serial.print(",Setpoint:"); Serial.print(motorSetpoint);
      Serial.print(",Actuator:"); Serial.print(motorOutput);
      Serial.print(",P_Term:"); Serial.print(telemetry.pTerm);
      Serial.print(",I_Term:"); Serial.print(telemetry.iTerm);
      Serial.print(",D_Term:"); Serial.print(telemetry.dTerm);
      Serial.print(",GainsScheduled:"); Serial.println(telemetry.gainsScheduled ? 1.0f : 0.0f);
    }
  }
}
