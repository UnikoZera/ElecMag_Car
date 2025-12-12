ElecMag_Car
===========

Overview
--------
- STM32F103C8T6 based electromagnetic car project (dual DC motors, encoders, ultrasonic, IMU, line tracker).
- Motor control: TIM1 PWM, TIM2/TIM3 encoders, PID speed loop in `Core/Src/motor.c`.
- Sensors: ADC line sensors, MPU6050 gyro, HC-SR04 ultrasonic, UART VOFA debug.

Hardware Pin Highlights
-----------------------
- Left motor IN1/IN2: PB4/PB5 (PB4 defaults to JTAG NJTRST, requires SWJ remap and GPIO init).
- Right motor IN1/IN2: PB0/PB1.
- Encoders: TIM2 (left), TIM3 (right).
- PWM: TIM1 CH2 (left), CH1 (right).
- Ultrasonic: PC14 trigger, PC15 echo interrupt.

Build & Flash
-------------
- Toolchain: STM32CubeIDE/Make (see Debug/makefile).
- Typical steps: `clean -> build -> flash` (via ST-Link SWD).
- If using CLI: run make in `Debug/` or IDE build target.

Runtime Notes
-------------
- Call `Sensor_Init()` then `Motor_Init()` before enabling motion control.
- Periodic tasks: `Sensor_Updater()`, `Get_Motor_Info()`, and `PID_Motor_Controllers_Speed_Updater(left, right)`.
- VOFA debug: send `debug_data` via UART1 when enabled in main loop.

PB4 Checklist (common issue)
----------------------------
- Enable AFIO clock and remap: `__HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_SWJ_NOJTAG();`
- Configure PB4 as output push-pull in GPIO init.
- Ensure no external short or burnt pin; test with simple toggle.

Tuning
------
- PID gains (`MOTOR_KP/KI/KD/DT`) in `motor.c`.
- Speed limits: `MOTOR_MAX_OUTPUT/MIN_OUTPUT` and PWM clamp `MOTOR_MAX_PWM/MIN_PWM` (negative min required for reverse).

Folder Map
----------
- Core/Inc: headers (motor, sensor, pid, tracker, etc.).
- Core/Src: application code (motor control, sensors, peripherals).
- Core/Startup: startup assembly.
- Debug/: build artifacts, makefiles.
- Drivers/: HAL and CMSIS sources.

Future TODO
-----------
- Add line-tracker fusion with gyro for smoother path keeping.
- Add position PID (currently commented) for precise stop.
- Document wiring diagram and power budget.

NOW STATE
-------------
- I should cancel this project by some reason
- Saddy.
- feel free to check my code.