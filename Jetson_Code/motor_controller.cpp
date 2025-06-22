#include "motor_controller.hpp"

#include <chrono>      // For timing and sleep functions
#include <cmath>       // For mathematical functions (abs, etc.)
#include <csignal>     // For signal handling
#include <iostream>    // For console I/O
#include <thread>      // For std::thread
#include <pthread.h>   // For pthread_setaffinity_np
#include <sched.h>     // For CPU_SET and scheduler functions
#include <atomic>      // For atomic variables
#include <time.h>      // For timespec and clock functions
#include <errno.h>     // For error codes (EINTR)
#include <cstdint>     // For int types like uint64_t
#include <algorithm>   // For std::clamp
#include <fstream>     // For file I/O
#include <iomanip>     // For std::setprecision, std::fixed

/* ----- fast GPIO & timer access --------------------------------------- */
#if defined(__linux__) && defined(__aarch64__)
#include <jetgpio.h>   // For GPIO control on Jetson
#else
#include "mock_gpio.h"   // Mock for non-Jetson systems
#endif

using clock_us = std::chrono::steady_clock;     

static inline struct timespec                                          
to_timespec(const clock_us::time_point& tp)                            
{                                                                      
    using namespace std::chrono;                                       
    int64_t ns = duration_cast<nanoseconds>(tp.time_since_epoch()).count(); 
    struct timespec ts;                                                
    ts.tv_sec  = ns / 1'000'000'000LL;                                 
    ts.tv_nsec = ns % 1'000'000'000LL;                                 
    return ts;                                                         
}                                                                      

static inline void sleepUntilAbs(const clock_us::time_point& tp)       
{                                                                      
#if defined(__linux__)
    struct timespec ts = to_timespec(tp);                              
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) 
           && errno == EINTR)                                          
        ; /* restart if interrupted */
#else
    // Fallback for non-Linux systems
    std::this_thread::sleep_until(tp);
#endif                                 
}                                                                      

/* ------------------------------------------------------------------------ */
/*  FAST TIMER helpers (borrowed from SPI example)                           */
/* ------------------------------------------------------------------------ */
/* ----- fast timer helpers -------------------------------------------- */
static inline uint64_t read_cycle()                               // unchanged
{ uint64_t v; asm volatile("mrs %0, cntvct_el0":"=r"(v)); return v; }

/* ----- Helper: calibrate timer frequency ---------------------------- */
static uint64_t timer_hz()
{
    static std::atomic<uint64_t> cached{0};
    uint64_t v = cached.load(std::memory_order_relaxed);
    if (v) return v;

    /* 100 ms calibration */
    using namespace std::chrono;
    auto t0 = high_resolution_clock::now();
    uint64_t c0 = read_cycle();
    std::this_thread::sleep_for(milliseconds(100));
    uint64_t c1 = read_cycle();
    auto t1 = high_resolution_clock::now();

    double secs = duration<double>(t1 - t0).count();
    v = static_cast<uint64_t>((c1 - c0) / secs + 0.5);
    cached.store(v, std::memory_order_relaxed);
    std::cout << "Generic timer calibrated: " << v << " Hz\n";
    return v;
}

static inline uint64_t us_to_cycles(float us)                     // MODIFIED
{ return static_cast<uint64_t>(us * timer_hz()/1'000'000.0f + 0.5f); }

MotorController::MotorController(const char* chipName,
        unsigned stepLineNum,
        unsigned dirLineNum)
    : chipName_(chipName),                        
    stepLineNum_(stepLineNum),
    dirLineNum_(dirLineNum),
    logFile("motor_data.csv"),
    lastLogTime(std::chrono::steady_clock::now()) {}                             

MotorController::~MotorController() noexcept      // <- MUST be noexcept
{
    stop();                     // releases GPIO lines inside
}

int64_t MotorController::position() const { 
    return pulseTally.load(std::memory_order_relaxed); 
}

float MotorController::getPosition() const {
    return INITIAL_POS + position() * METERS_PER_STEP;
}

void MotorController::openLinesIfNeeded()
{
    if (linesReady_) return;

    std::cout << "Initializing GPIO..." << std::endl;
    
    int result = gpioInitialise();

    if (result < 0) { 
        std::cerr << "gpioInitialise failed with error: " << result << std::endl;
        std::raise(SIGTERM);
    } 
    std::cout << "GPIO initialized" << std::endl;

    std::cout << "Setting up STEP pin " << stepLineNum_ << std::endl;
    result = gpioSetMode(stepLineNum_, JET_OUTPUT);
    if (result < 0) {
        std::cerr << "Failed to set STEP pin mode. Error: " << result << std::endl;
        std::raise(SIGTERM);
    }

    std::cout << "Setting up DIR pin " << dirLineNum_ << std::endl;
    result = gpioSetMode(dirLineNum_, JET_OUTPUT);
    if (result < 0) {
        std::cerr << "Failed to set DIR pin mode. Error: " << result << std::endl;
        std::raise(SIGTERM);
    }

    result = gpioWrite(stepLineNum_, 0);
    if (result < 0) {
        std::cerr << "Failed to write initial STEP pin state. Error: " << result << std::endl;
        std::raise(SIGTERM);
    }

    result = gpioWrite(dirLineNum_, 0);
    if (result < 0) {
        std::cerr << "Failed to write initial DIR pin state. Error: " << result << std::endl;
        std::raise(SIGTERM);
    }

    linesReady_ = true;
    std::cout << "GPIO setup complete" << std::endl;
}

void MotorController::start(int cpuId)               // ← optional core number
{
    openLinesIfNeeded();                 // GPIO request (RAII)
    if (running.exchange(true)) return;

    worker  = std::thread(&MotorController::loop, this);

    /* ----- set CPU affinity if the caller asked for it ------ */
#if defined(__linux__)
    if (cpuId >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpuId, &cpuset);
        pthread_setaffinity_np(worker.native_handle(),
                            sizeof(cpu_set_t), &cpuset);
    }
#else
    (void)cpuId; // Suppress unused parameter warning on non-Linux
#endif
}

void MotorController::stop() noexcept
{
    running=false;
    if(worker.joinable()) worker.join();
    if(linesReady_){ gpioTerminate(); linesReady_=false; }                         
}

void MotorController::setTarget(float accel_m_s2)
{
    /* 1 ── clamp to spec limits */
    accel_m_s2 = std::clamp(accel_m_s2, -MAX_ACCEL, MAX_ACCEL);

    /* 2 ── are we currently braking to zero? */
    Phase ph = phase.load(std::memory_order_acquire);
    if (ph == Phase::BRAKE_TO_ZERO)
    {
        /* read the instantaneous speed once */
        float f = curSpeed();

        /* --- case A : flip no longer required ------------------- */
        if (accel_m_s2 * f >= 0.0f)
        {
            targetAccel.store(accelToSteps(std::fabs(accel_m_s2)),
                            std::memory_order_relaxed);
            targetFreq .store(accel_m_s2 > 0 ?  MAX_STEPS_PER_SEC
                                            : -MAX_STEPS_PER_SEC,
                            std::memory_order_relaxed);

            phase.store(Phase::RUN, std::memory_order_release);
            pendingAccel.store(0.0f,  std::memory_order_relaxed);

            return;
        }

        /* --- case B : still braking, but update the pending acceleration -- */
        pendingAccel.store(accel_m_s2, std::memory_order_release);
        /* braking continues with the new magnitude */
        return;
    }

    /* 3 ── normal path (we are in RUN or FLIP_ACCEL) ------------- */
    float curF = curFreq.load(std::memory_order_relaxed);      
    if (accel_m_s2 * curF < 0 &&
        std::fabs(curF) > DIR_CHANGE_RPM * STEPS_PER_REV / 60.0f) {
        targetAccel.store(accelToSteps(std::fabs(accel_m_s2)),
                    std::memory_order_relaxed);
        pendingAccel.store(accel_m_s2, std::memory_order_release);
        phase.store(Phase::BRAKE_TO_ZERO, std::memory_order_release);
        return;
    }

    /* straight run – same direction */
    targetAccel.store(accelToSteps(std::fabs(accel_m_s2)),
                    std::memory_order_relaxed);
    targetFreq .store(accel_m_s2 > 0 ?  MAX_STEPS_PER_SEC
                                    : -MAX_STEPS_PER_SEC,
                    std::memory_order_relaxed);
    phase.store(Phase::RUN, std::memory_order_release);
}

/* safer version */
float MotorController::curSpeed(std::memory_order order) const
{
    return curFreq.load(order);
}

float MotorController::targetSpeed() const
{
    return targetFreq.load(std::memory_order_relaxed);
}

float MotorController::curSpeed_m_s() const
{
    return curSpeed() * METERS_PER_STEP;
}

// Fix for the frequency issue in MotorController::loop()
void MotorController::loop() noexcept
{
    using namespace std::chrono;

    /* ---------- constants ------------------------------------- */
    constexpr float  FLIP_THRESH   =
        DIR_CHANGE_RPM * STEPS_PER_REV / 60.0f;               // steps s‑¹
    constexpr auto   BC_PERIOD     = microseconds(492);      // check every 2.03ms (max 10cm travel between checks)

    /* ---------- timers ---------------------------------------- */
    auto nextWake     = clock_us::now();
    auto nextBndCheck = clock_us::now() + BC_PERIOD;

    // Add overhead measurement variables
    static uint64_t lastLoopEndCycle = 0;

    /* helper: first‑order ramp toward a target frequency -------- */
    auto rampTowards = [this](float tgt, float accelStepsPerSec2)
    {
        static auto lastUpdate = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastUpdate).count();
        lastUpdate = now;

        float df = tgt - curFreq.load(std::memory_order_relaxed);   
        if (df == 0) return;
        
        // If acceleration is zero or very close to zero, maintain current frequency
        if (std::fabs(accelStepsPerSec2) < 0.0001f) return;
        
        // Calculate maximum frequency change based on actual elapsed time
        float max_df = accelStepsPerSec2 * dt;
        if (std::fabs(df) > max_df)
            df = std::copysign(max_df, df);

        float newFreq = curFreq.load(std::memory_order_relaxed) + df;
        
        if (panicStopFlag.load(std::memory_order_relaxed)) {
            if ((curFreq > 0 && newFreq < 0) || (curFreq < 0 && newFreq > 0)) {
                newFreq = 0;
            }
        }
        
        curFreq.store(newFreq, std::memory_order_relaxed);

        if (panicStopFlag.load(std::memory_order_relaxed)) {
                std::cout << "Panic stop was engaged - newFreq = " << newFreq << " steps/s, "
                          << "decel = " << accelStepsPerSec2 << " steps/s², "
                          << "position = " << (INITIAL_POS + position() * METERS_PER_STEP) << " meters\n";  
            }   
    };
    
    while (running.load(std::memory_order_acquire))
    {
        /* ---------- state‑machine ----------------------------- */
        switch (phase.load(std::memory_order_acquire))
        {
        case Phase::RUN:
            if (panicStopFlag.load(std::memory_order_relaxed)) {
                panicStop();
                break;
            }   

            rampTowards(targetFreq.load(std::memory_order_relaxed),
                            targetAccel.load(std::memory_order_relaxed));
            break;

        case Phase::PANIC_STOP:
            {
                panicStopFlag.store(true, std::memory_order_relaxed);
                
                /* magnitude the user asked for (always ≥ 0) */
                const float effAccel_steps_s2 =
                std::fabs(targetAccel.load(std::memory_order_relaxed));

                /* when slow enough, freeze at zero                        */
                const float flipThresh =
                    DIR_CHANGE_RPM * STEPS_PER_REV / 60.0f;   // steps s-¹
                
                if (std::fabs(curFreq.load(std::memory_order_relaxed)) <= flipThresh)
                {
                    targetAccel.store(0.0f, std::memory_order_relaxed);
                    targetFreq .store(0.0f, std::memory_order_relaxed);
                    phase.store(Phase::HOLD_ZERO, std::memory_order_release);
                    // Send a SIGINT signal to the process (equivalent to Ctrl+C)
                    std::cout << "Emergency stop complete. Current frequency: " << curFreq.load(std::memory_order_relaxed) << " steps/s. Sending SIGINT to terminate program." << std::endl;
                    std::raise(SIGINT);
                }

                /* decelerate toward 0 Hz                                  */
                rampTowards(0.0f, effAccel_steps_s2);

                break;
            }
            
        case Phase::BRAKE_TO_ZERO:
            {
                if (panicStopFlag.load(std::memory_order_relaxed)) {
                    panicStop();
                    break;
                }   
                
                /* Decelerate toward 0 Hz at the magnitude requested by the user.
                If that magnitude is 0 the motor simply coasts forever and the
                state machine never advances, exactly as you specified. */
                const float effAccel_m_s2 =
                    std::fabs(pendingAccel.load(std::memory_order_relaxed));
            
                rampTowards(0.0f, accelToSteps(effAccel_m_s2));
            
                if (effAccel_m_s2 > 0.0f &&
                    std::fabs(curFreq.load(std::memory_order_relaxed)) <= FLIP_THRESH)
                {                    
                    // Execute FLIP_ACCEL logic immediately
                    float acc = pendingAccel.load(std::memory_order_relaxed);
                    targetAccel.store(accelToSteps(std::fabs(acc)),
                                    std::memory_order_relaxed);
                    targetFreq.store(acc > 0 ?  MAX_STEPS_PER_SEC
                                              : -MAX_STEPS_PER_SEC,
                                    std::memory_order_relaxed);
                    phase.store(Phase::RUN, std::memory_order_release);
                }

                break;
            }
        /* -------------------------------------------------------
        * HOLD_ZERO: stay idle at 0 Hz; minimal CPU load.
        * A new setTarget() call can overwrite phase at any time.
        * ------------------------------------------------------*/
        case Phase::HOLD_ZERO:
            {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));    
            }
            
        break;
            
        }

        bool fwd = curFreq.load(std::memory_order_relaxed) >= 0.0f;
        if (fwd != dirForward.load(std::memory_order_relaxed)) {
            dirForward.store(fwd, std::memory_order_relaxed);
            gpioWrite(dirLineNum_, fwd ? 0 : 1);  
            std::cout << "Direction changed to " 
                      << (fwd ? "forward" : "reverse")
                      << " (DIR_PIN state: " << gpioRead(dirLineNum_) << ")"
                      << std::endl;
        }

        float absF = std::fabs(curFreq.load(std::memory_order_relaxed));
        if (absF > 1.0f) {
            // Calculate timing values first
            float period_us = 1'000'000.0f / absF;
            uint64_t full_cycle = us_to_cycles(period_us);
            uint64_t half = full_cycle / 2;
            
            // Log data every 500ms (120 times per minute)
            auto currentTime = std::chrono::steady_clock::now();
            if (currentTime - lastLogTime >= LOG_INTERVAL) {
                // Calculate elapsed time in seconds
                auto elapsed = std::chrono::duration<double>(currentTime.time_since_epoch()).count();
                
                // Get current acceleration
                float currentAccel = 0.0f;
                currentAccel = targetAccel.load(std::memory_order_relaxed);
                if (!dirForward.load(std::memory_order_relaxed)) {
                    currentAccel = -currentAccel;
                }    
                // Calculate current position
                float pos = INITIAL_POS + position() * METERS_PER_STEP;
                
                // Write to file: timestamp, frequency, acceleration, position
                logFile << std::fixed << std::setprecision(3)
                        << elapsed << ","
                        << absF << ","
                        << currentAccel << ","
                        << pos << std::endl;
                
                lastLogTime = currentTime;
            }
            /* ----- High half-cycle ---------------------------------- */
            // uint64_t t0 = read_cycle();
            uint64_t t0 = lastLoopEndCycle ? lastLoopEndCycle : read_cycle();
            
            gpioWrite(stepLineNum_, 1);
            
            // Simple busy wait without overhead compensation
            uint64_t end_high = t0 + half;
            while (read_cycle() < end_high) { /* busy wait */ }

            /* ----- Low half-cycle ----------------------------------- */
            // t0 = read_cycle();
            gpioWrite(stepLineNum_, 0);
            
            // Simple busy wait without overhead compensation
            // uint64_t end_low = t0 + half;
            uint64_t end_low = end_high + half;
            while (read_cycle() < end_low) { /* busy wait */ }

            pulseTally.fetch_add(fwd ? +1 : -1, std::memory_order_relaxed);

            lastLoopEndCycle = end_low;
        } else {
            std::this_thread::sleep_for(milliseconds(1));
            nextWake = clock_us::now() + microseconds(1);
        }

        auto now = clock_us::now();
        if (now >= nextBndCheck && !panicStopFlag.load(std::memory_order_relaxed)) {
            if (!boundaryCheck()) {
                logFile << "boundaryCheck failed" << std::endl;
            }
            nextBndCheck = now + BC_PERIOD;
            int dirPinState = gpioRead(dirLineNum_);
            std::cout << "position is " << std::fixed << std::setprecision(4) 
                      << (INITIAL_POS + position() * METERS_PER_STEP) << " meters, DIR_PIN=" 
                      << dirPinState << ", time=" << now.time_since_epoch().count() << std::endl;
        }
        
        // FIX: Only sleep if we're not at high frequency
        if (absF <= 1.0f) {
            sleepUntilAbs(nextWake);
        }
    }
}

/* ---------------------------------------------------------------
 * Panic-stop: bring the motor to 0 Hz in minimum time
 * --------------------------------------------------------------*/
void MotorController::panicStop() noexcept          // ← new helper
{
    /* Snapshot current signed frequency (steps · s⁻¹) */
    float f = curFreq.load(std::memory_order_acquire);
    if (std::fabs(f) < 1.0f)          // already (almost) stopped
        return;

    /* Choose the deceleration sign opposite to motion */
    float stopAccel_m_s2 = -std::copysign(MAX_ACCEL, f);     // ±MAX_ACCEL

    /* — Drive the state machine directly —
       • targetFreq  = 0 Hz ⇒ rampTowards() knows we want to stop
       • targetAccel = |MAX_ACCEL|  (positive magnitude)
       • phase       = RUN so no direction flip will be triggered
       • pendingAccel cleared so nothing revives after the stop   */
    targetAccel.store(accelToSteps(stopAccel_m_s2), std::memory_order_relaxed);
    targetFreq .store(0.0f,                 std::memory_order_relaxed);
    phase.store(Phase::PANIC_STOP,       std::memory_order_release);

    std::cout << "Panic stop engaged at " << f << " steps/s, "
              << "decel = " << stopAccel_m_s2 << " m·s⁻², "
              << "targetAccel = " << accelToSteps(stopAccel_m_s2) << " steps/s², "
              << "position = " << (INITIAL_POS + position() * METERS_PER_STEP) << " meters\n";
}

/* ------------------------------------------------------------------
 *  MotorController::setConstantSpeed()
 *  -----------------------------------------------------------------
 *  Immediately forces the generator to the requested signed speed
 *  ( +RPM = forward, –RPM = reverse ).  Acceleration is set to zero,
 *  so the state machine no longer ramps; the loop() thread will
 *  simply start ticking at the new frequency on its very next cycle.
 * ----------------------------------------------------------------*/
void MotorController::setConstantSpeed(float speed_rpm)
{
    /* 1 ── convert RPM → signed step frequency (steps·s-¹) */
    float f_steps = speed_rpm * (STEPS_PER_REV / 60.0f);  // This is giving us too high frequency

    /* 2 ── clip to the electrical limits of the drive */
    f_steps = std::clamp(f_steps, -MAX_STEPS_PER_SEC, +MAX_STEPS_PER_SEC);

    /* 3 ── shove the numbers straight into the atomics */
    curFreq   .store(f_steps,              std::memory_order_relaxed);
    targetFreq.store(f_steps,              std::memory_order_relaxed);
    targetAccel.store(0.0f,                std::memory_order_relaxed);
    pendingAccel.store(0.0f,               std::memory_order_relaxed);

    /* 4 ── be sure the state-machine is in the plain RUN phase          */
    phase.store(Phase::RUN, std::memory_order_release);
}

bool MotorController::boundaryCheck()                 
{
    if (panicStopFlag.load(std::memory_order_relaxed)) {
        return true;
    }

    float v = curSpeed() * METERS_PER_STEP;
    float absV = std::fabs(v);
    float stopDist = (absV * absV) / (2.0f * MAX_ACCEL);

    float pos = INITIAL_POS + position() * METERS_PER_STEP;   // you would replace by encoder

    if ((v < 0.0f && pos < MIN_POSITION + STOP_DIST_SAFETY_FACTOR + stopDist) || 
        (v > 0.0f && pos > MAX_POSITION - CART_WIDTH - STOP_DIST_SAFETY_FACTOR - stopDist)) {
        std::cout << "Boundary check failed at " << pos << " meters, v=" << v << " m/s, stopDist=" << stopDist << " meters" << std::endl;
        panicStop();
        return false;
    }

    return true;
}

void MotorController::gotoPosition(float current_pos, float target_pos)
{
    float delta = target_pos - current_pos;
    if (delta == 0.0f) return; // Already at target

    // Set direction
    if (delta > 0) {
        gpioWrite(dirLineNum_, 0); // Forward
    } else {
        gpioWrite(dirLineNum_, 1); // Reverse
        delta = -delta; // Make positive for pulse count
    }

    // Calculate number of steps needed
    int steps = static_cast<int>(delta / METERS_PER_STEP + 0.5f); // Round to nearest

    // Send pulses
    for (int i = 0; i < steps; ++i) {
        gpioWrite(stepLineNum_, 1);
        // Insert a small delay here for your hardware, e.g. busy-wait or sleep
        std::this_thread::sleep_for(std::chrono::microseconds(1)); // 1 microseconds delay
        gpioWrite(stepLineNum_, 0);
        // Insert a small delay here for your hardware, e.g. busy-wait or sleep
        std::this_thread::sleep_for(std::chrono::microseconds(1)); // 1 microseconds delay
    }

    gpioWrite(dirLineNum_, 0); // Forward
}

