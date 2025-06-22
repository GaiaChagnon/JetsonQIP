#ifndef MOTORACC6_H
#define MOTORACC6_H

#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include <cmath>  // For M_PI

/* ----- Jetson pin mapping (DT names) ------------------------ */
constexpr const char* CHIPNAME = "gpiochip0";
constexpr unsigned STEP_PIN = 18;     // Use header pin number 18 directly
constexpr unsigned DIR_PIN = 15;      // Use header pin number 15 directly

/* ----- Mechanical constants (unchanged) -------------------- */
constexpr float   WHEEL_CIRC_M    = M_PI * 0.03145f;  // 31.45 mm diameter
constexpr uint32_t STEPS_PER_REV  = 1500;
constexpr float   METERS_PER_STEP = WHEEL_CIRC_M / STEPS_PER_REV;
constexpr float   RPM_TO_MS       = WHEEL_CIRC_M / 60.0f;
constexpr float   MS_TO_RPM       = 1.0f / RPM_TO_MS;

/* ----- Limits & thresholds --------------------------------- */
constexpr float MAX_POSITION     = 1.5f;   // m
constexpr float MIN_POSITION     = 0.0f;
constexpr float STOP_DIST_SAFETY_FACTOR = 0.20f;
constexpr float CART_WIDTH              = 0.01f;
constexpr float SAFETYUPPERBOUND = MAX_POSITION - STOP_DIST_SAFETY_FACTOR/2;
constexpr float SAFETYLOWERBOUND = MIN_POSITION + STOP_DIST_SAFETY_FACTOR/2;  

constexpr float MAX_SPEED_RPM    = 3000.0f;
constexpr float MAX_STEPS_PER_SEC= (MAX_SPEED_RPM/60)*STEPS_PER_REV; //75 kHz
constexpr float MAX_ACCEL        = 50.0f;   // m·s⁻²
constexpr float DIR_CHANGE_RPM   = 15.0f;
constexpr float INITIAL_POS      = MAX_POSITION / 2;

/* ===== Helper : convert m·s⁻² to steps·s⁻² ================= */
inline float accelToSteps(float acc_m_s2)
{
    return acc_m_s2 * (STEPS_PER_REV / WHEEL_CIRC_M);
}

class MotorController {
public:
    // Constructor: initializes MotorController with GPIO chip name and STEP/DIR pin numbers
    MotorController(const char* chipName, unsigned stepLineNum, unsigned dirLineNum);

    // Destructor: cleans up resources
    ~MotorController() noexcept;

    // Starts the motor control loop on an optional specific CPU core
    void start(int cpuId = -1);

    // Stops the motor control loop and releases GPIO resources
    void stop() noexcept;

    // Sets target acceleration in m/s²; used for ramping up/down motor speed
    void setTarget(float accel_m_s2);

    // Sets motor speed directly to a constant RPM, bypassing acceleration ramp
    void setConstantSpeed(float speed_rpm);

    // Returns current position in steps
    int64_t position() const;

    // Returns current position in meters
    float getPosition() const;

    // Immediately stops the motor safely by quickly decelerating to zero
    void panicStop() noexcept;

    // Checks if motor position is within safe boundaries
    bool boundaryCheck();

    // Moves motor from current position to specified target position
    void gotoPosition(float current_pos, float target_pos);

    // Returns current speed in steps per second
    float curSpeed(std::memory_order order = std::memory_order_relaxed) const;

    // Returns target speed in steps per second
    float targetSpeed() const;

    // Returns current speed in meters per second
    float curSpeed_m_s() const; 

private:
    // Opens GPIO lines if they haven't been opened already
    void openLinesIfNeeded();

    // Main control loop that generates motor pulses
    void loop() noexcept;

    // Enumeration to define different operational phases of motor control
    enum class Phase { RUN, PANIC_STOP, BRAKE_TO_ZERO, HOLD_ZERO };

    // GPIO chip name (platform-dependent)
    const char* chipName_;

    // GPIO line numbers for STEP and DIR signals
    unsigned stepLineNum_;
    unsigned dirLineNum_;

    // Tally of pulses generated (motor steps)
    std::atomic<int64_t> pulseTally{0};

    // Thread object managing the motor control loop
    std::thread worker;

    // Flag indicating if the control loop is active
    std::atomic<bool> running{false};

    // Target frequency (steps per second) and acceleration (steps per second squared)
    std::atomic<float> targetFreq{0}, targetAccel{0};

    // Current frequency (steps per second)
    std::atomic<float> curFreq{0};

    // Direction of motor rotation (true: forward, false: reverse)
    std::atomic<bool> dirForward{true};

    // Flag indicating an emergency stop condition
    std::atomic<bool> panicStopFlag{false};

    // Current operational phase of motor
    std::atomic<Phase> phase{Phase::RUN};

    // Pending acceleration value used when transitioning between states
    std::atomic<float> pendingAccel{0.0f};

    // Flag indicating if GPIO lines are initialized and ready
    bool linesReady_{false};

    // File stream used for logging motor data
    std::ofstream logFile;

    // Timestamp of the last log entry
    std::chrono::steady_clock::time_point lastLogTime;

    // Logging interval (500 milliseconds)
    static constexpr auto LOG_INTERVAL = std::chrono::milliseconds(500);

    // platform sanity check
    static_assert(std::atomic<float>::is_always_lock_free,
        "atomic<float> must be lock-free on this platform");
};

#endif // MOTORACC6_H
