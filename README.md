Pendulum Balance Controller - Advanced Real-Time Control System

PRIMARY PURPOSE
---------------
An advanced real-time pendulum balance controller—an inverted-pendulum robot that keeps one or more pendulum arms upright while moving a cart along a linear track.

SYSTEM ARCHITECTURE
===================

1. Hardware Setup
   • Cart on linear track: 1.5 m travel range  
   • Stepper motor: 1500 steps/rev, 31.45 mm wheel diameter  
   • Pendulum arms: 1–4 arms (configurable) mounted on cart  
   • Wireless sensors: STM32 + encoder + nRF24L01+ on each arm  
   • Main controller: NVIDIA Jetson Nano

2. Control-System Components
   • motor_controller.cpp / .hpp  
     – 75 kHz step pulses (3000 RPM max)  
     – ±50 m/s² acceleration limits  
     – Boundary safety and emergency stop  
     – Cycle-accurate timing (ARM cycle counter)  
     – Runs on dedicated CPU core 2  

   • communication_controller.cpp / .hpp  
     – Bit-banged SPI at 10 MHz to nRF24L01+ radios  
     – Supports up to 4 pendulum arms  
     – Reads encoder data at 150 Hz  
     – CRC-4 error checking and recovery  
     – Custom wireless packet format  

   • inference_engine.cpp / .hpp  
     – Loads TensorRT model file pendulum.trt  
     – GPU-accelerated inference on Jetson Nano  
     – Builds state vector (cart pos/vel + arm angles/vels)  
     – Returns cart-acceleration command  

   • pendulum_balance_controller.cpp  
     – 150 Hz control loop (6.67 ms period)  
     – Multi-threaded: motor control on its own thread  
     – Rust-style Result<T> error handling  
     – Real-time metrics and logging  
     – Fallback control when AI inference fails

CONTROL-LOOP OPERATION (150 Hz)
-------------------------------
1. Sensor collection  
   – Cart position & velocity from motor controller  
   – Pendulum angles & velocities via nRF24L01+ radios  
2. AI inference  
   – Current state → TensorRT → optimal cart acceleration  
   – Validate output, handle errors  
3. Control application  
   – Send acceleration to motor controller (step pulses)  
   – Apply safety limits and boundary checks  
4. Performance monitoring  
   – Measure timing, log events, keep real-time deadlines

WHAT THE SYSTEM DOES
--------------------
• Balances one to four pendulum arms upright in real time  
• Moves the cart to target positions while balancing  
• Adapts via neural-network inference  
• Performs emergency stop and PID fallback on failure

REAL-WORLD USES
---------------
• Control-systems research  
• Robotics education demos  
• AI/ML controller validation  
• Precision mechatronics showcases

ADVANCED FEATURES
-----------------
Performance
  – SCHED_FIFO real-time threads  
  – CPU affinity (dedicated core)  
  – RAII and smart pointers for memory safety  
  – Lock-free atomics where possible

Robustness
  – Graceful error recovery  
  – Physical boundary protection  
  – Wireless packet-loss handling  
  – PID fallback controller

Monitoring & Debug
  – Thread-safe logging with timestamps  
  – Live timing metrics  
  – Configurable verbosity levels  
  – CSV data logging for analysis

EXPECTED RUN-TIME BEHAVIOUR
---------------------------
1. Initialise hardware (motor, SPI, GPIO, radios)  
2. Calibrate timers and load TensorRT model  
3. Start 150 Hz control loop  
4. Continuously balance pendulum(s) and move cart  
5. Log performance and accept user commands  
6. Graceful shutdown on Ctrl+C

# Build Instructions:
    
    g++ -std=c++17 -O3 -Wall -Wextra -pthread \
    pendulum_balance_controller.cpp \
    motor_controller.cpp \
    inference_engine.cpp \
    communication_controller.cpp \
    -ljetgpio -lrt \
    -L/usr/local/cuda-10.2/lib64 \
    -L/usr/local/cuda-10.2/targets/aarch64-linux/lib \
    -lcuda -lcudart -lnvinfer -lnvinfer_plugin \
    -o pendulum_balance_controller
            
# Run (as root on Jetson for GPIO access):
    sudo ./pendulum_balance_controller
