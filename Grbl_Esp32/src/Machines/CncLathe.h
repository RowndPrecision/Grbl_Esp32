#pragma once
// clang-format off

#define MACHINE_NAME "CncLathe"

#define N_AXIS 4

// #define SERVO_MODE // Distinguishes between regular machines and those upgraded with a servo  

#define CUSTOM_CODE_FILENAME    "../Custom/CncLathe.cpp"

#define USE_I2S_OUT

#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_23
#define I2S_OUT_DATA            GPIO_NUM_21
#define I2S_OUT_EN              GPIO_NUM_19

#define RS485_RX                GPIO_NUM_33
#define RS485_TX                GPIO_NUM_32
#define RS485_CTRL              GPIO_NUM_12

#define DEFAULT_PLANE   Plane::ZX  // ZX for lathe

#define DEFAULT_SWAP_Y  C_AXIS
#define DEFAULT_SWAP_C  Y_AXIS

#define REMOVABLE_AXIS_LIMIT A_AXIS
// #define BLOCK_AXIS_ON_PWM_MODE  A_AXIS

#define COOLANT_MIST_PIN GPIO_NUM_13

#define POSITIONABLE_SPINDLE_AXIS DEFAULT_SWAP_C
#define POSITIONABLE_AXIS_CONVERT 360.0f // 1.0f
// #define POSITIONABLE_AXIS_CONVERT 1.0f // 360.0f

#define X_STEP_PIN              GPIO_NUM_27
#define X_DIRECTION_PIN         GPIO_NUM_14

#define Z_STEP_PIN              GPIO_NUM_25
#define Z_DIRECTION_PIN         GPIO_NUM_26

#define STEPPERS_DISABLE_PIN    I2SO(0)

#define Y_STEP_PIN          GPIO_NUM_2
#define Y_DIRECTION_PIN     GPIO_NUM_15
#define Y_DISABLE_PIN       I2SO(2)

#define DEFAULT_INVERT_ST_ENABLE 0 //bit(Y_AXIS)

#define A_STEP_PIN          GPIO_NUM_5
#define A_DIRECTION_PIN     I2SO(5)
#define A_DISABLE_PIN       I2SO(6)

#define X_LIMIT_PIN                 GPIO_NUM_36
#define Z_LIMIT_PIN                 GPIO_NUM_39
#define A_LIMIT_PIN                 GPIO_NUM_18
#define PROBE_PIN                   GPIO_NUM_34
#define CONTROL_SAFETY_DOOR_PIN     GPIO_NUM_35

#define DEFAULT_HOMING_CYCLE_0  bit(X_AXIS)
#define DEFAULT_HOMING_CYCLE_1  bit(Z_AXIS)
#define DEFAULT_HOMING_CYCLE_2  0
#define DEFAULT_HOMING_CYCLE_3  0
#define DEFAULT_HOMING_CYCLE_4  0
#define DEFAULT_HOMING_CYCLE_5  0

#define SPINDLE_TYPE            SpindleType::PWM
#define DEFAULT_SPINDLE_FREQ    5000.0f

#define SPINDLE_OUTPUT_PIN      GPIO_NUM_17
#define SPINDLE_DIR_PIN         I2SO(5)
#define SPINDLE_ENABLE_PIN      I2SO(7)

#define ASDA_CN1_OUTPUT_PIN     GPIO_NUM_16
#define ASDA_CN1_S_P_PIN        I2SO(1)
#define ASDA_CN1_ENABLE_PIN     I2SO(2)
#define ASDA_CN1_DIR_PIN        I2SO(3)

#define DEFAULT_ROWND_VERBOSE 0
#define DEFAULT_ROWND_G76_IGNORE_OFFSET false
#define DEFAULT_ROWND_IGNORE_DOOR_SWITCH false

#define LED_PIN     I2SO(4)
#define DEFAULT_LED_STATE 1
#define DEFAULT_LED_INVERSE 0


#define DEFAULT_INVERT_SPINDLE_OUTPUT_PIN       false
#define DEFAULT_INVERT_SPINDLE_ENABLE_PIN       true
#define DEFAULT_INVERT_SPINDLE_DIRECTION_PIN    false

#define DEFAULT_INVERT_CHUCK_OUTPUT_PIN       false
#define DEFAULT_INVERT_CHUCK_ENABLE_PIN       true
#define DEFAULT_INVERT_CHUCK_DIRECTION_PIN    false

#define DEFAULT_INVERT_LASER_OUTPUT_PIN       false
#define DEFAULT_INVERT_LASER_ENABLE_PIN       false
#define DEFAULT_INVERT_LASER_DIRECTION_PIN    false


#define DEFAULT_MONITOR_FILTER_RANGE 3
#define DEFAULT_MONITOR_FILTER_MIDDLE 1950
#define DEFAULT_MONITOR_FILTER_MIN 1150
#define DEFAULT_MONITOR_FILTER_MAX  2750
#define DEFAULT_MONITOR_MAP_MAX 1500

// #define USING_AXIS_AS_SPINDLE   Y_AXIS // uncomment to use this feature

#ifdef USING_AXIS_AS_SPINDLE
    #define Y_STEP_PIN          SPINDLE_OUTPUT_PIN
    #define Y_DIRECTION_PIN     SPINDLE_DIR_PIN
    #define Y_ENABLE_PIN        SPINDLE_ENABLE_PIN
#endif

// defaults
#define DEFAULT_STEP_PULSE_MICROSECONDS 3
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // 0xFF = stay on

#define DEFAULT_STEPPING_INVERT_MASK 0 // uint8_t
#define DEFAULT_DIRECTION_INVERT_MASK (bit(Z_AXIS) | bit(A_AXIS)) // uint8_t
#define DEFAULT_INVERT_LIMIT_PINS bit(A_AXIS) // uint8_t
#define DEFAULT_INVERT_PROBE_PIN 1 // boolean

#define DEFAULT_STATUS_REPORT_MASK 1

#define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
#define DEFAULT_ARC_TOLERANCE 0.002 // mm
#define DEFAULT_REPORT_INCHES 0 // false

#define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
#define DEFAULT_HARD_LIMIT_ENABLE 1  // false

#define DEFAULT_HOMING_ENABLE 1
#define DEFAULT_HOMING_DIR_MASK 0 // move positive dir Z, negative X,Y
#define DEFAULT_HOMING_FEED_RATE 200.0 // mm/min
#define DEFAULT_HOMING_SEEK_RATE 1000.0 // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF 3.0 // mm

#define DEFAULT_X_HOMING_MPOS 3.0 // mm
#define DEFAULT_Z_HOMING_MPOS 3.0 // mm

#define DEFAULT_CHUCK_RPM_MAX 3000.0 // rpm
#define DEFAULT_SPINDLE_RPM_MAX 12000.0 // rpm
#define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm

#define DEFAULT_LASER_MODE 0 // false

#define DEFAULT_X_STEPS_PER_MM 1600.0   // steps per mm
#define DEFAULT_Y_STEPS_PER_MM 10.0 * POSITIONABLE_AXIS_CONVERT    // steps per rev
#define DEFAULT_Z_STEPS_PER_MM 1600.0   // steps per mm
#define DEFAULT_A_STEPS_PER_MM 24000.0   // steps per mm

#define DEFAULT_X_MAX_RATE 1200.0   // mm/min
#define DEFAULT_Y_MAX_RATE 360000.0 / POSITIONABLE_AXIS_CONVERT  // rpm
#define DEFAULT_Z_MAX_RATE 1200.0   // mm/min
#define DEFAULT_A_MAX_RATE 70   // mm/min

#define DEFAULT_X_ACCELERATION 200.0 // mm/sec^2. 50 mm/sec^2 = 180000 mm/min^2
#define DEFAULT_Y_ACCELERATION 3600.0 / POSITIONABLE_AXIS_CONVERT // mm/sec^2
#define DEFAULT_Z_ACCELERATION 200.0 // mm/sec^2
#define DEFAULT_A_ACCELERATION 20.0 // mm/sec^2

#define DEFAULT_X_MAX_TRAVEL 100.0  // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 0.0    // 0 = inf
#define DEFAULT_Z_MAX_TRAVEL 300.0  // This is percent in servo mode...used for calibration
#define DEFAULT_A_MAX_TRAVEL 40.0   // This is percent in servo mode...used for calibration

#define DEFAULT_ATC_STATE false

#define DEFAULT_ATC_SPEED 57.14

#define DEFAULT_ATC_DISTANCE 5

#define DEFAULT_ATC_OFFSET 2

#define DEFAULT_SPINDLE_DELAY_SPINUP 0.75

#define DEFAULT_COOLANT_DELAY_TURNON 0.5


#ifdef SERVO_MODE
#undef DEFAULT_X_MAX_RATE
#undef DEFAULT_Z_MAX_RATE
#define DEFAULT_X_MAX_RATE 2000.0   // mm/min
#define DEFAULT_Z_MAX_RATE 2000.0   // mm/min
#endif
