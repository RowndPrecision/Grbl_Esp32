
#include "../src/Grbl.h"

/*
This function is used as a one time setup for your machine.
*/
void machine_init() {
    // print a startup message to show the kinematics are enable
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Rownd 5 Axis kinematics Init");
}

/*
This is used to initialize a display.
*/
void display_init() {}

/*
  limitsCheckTravel() is called to check soft limits
  It returns true if the motion is outside the limit values
*/
bool limitsCheckTravel() {
    return false;
}

/*
  user_defined_homing(uint8_t cycle_mask) is called at the begining of the normal Grbl_ESP32 homing
  sequence.  If user_defined_homing(uint8_t cycle_mask) returns false, the rest of normal Grbl_ESP32
  homing is skipped if it returns false, other normal homing continues.  For
  example, if you need to manually prep the machine for homing, you could implement
  user_defined_homing(uint8_t cycle_mask) to wait for some button to be pressed, then return true.
*/
// Cycle mask is 0 unless the user sends a single axis command like $HZ
// This will always return true to prevent the normal Grbl homing cycle
bool user_defined_homing(uint8_t cycle_mask) {
    return false;
}

/*
  kinematics_pre_homing() is called before normal homing
  You can use it to do special homing or just to set stuff up

  cycle_mask is a bit mask of the axes being homed this time.
*/
bool kinematics_pre_homing(uint8_t cycle_mask) {
    return false;  // finish normal homing cycle
}

/*
 Apply inverse kinematics for a lathe spindle
 
 float target: 					The desired target location in machine space
 plan_line_data_t *pl_data:		Plan information like feed rate, etc
 float *position:				The previous "from" location of the move
*/
bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
    float delta_in[MAX_N_AXIS];
    float delta_out[MAX_N_AXIS];

    for (size_t idx = 0; idx < MAX_N_AXIS; ++idx) {
        delta_in[idx] = target[idx] - position[idx];
    }

    if (rownd_param_experimental_axis_feed->get() && !gc_state.Rownd_thread) {
        float d_mm  = 0;
        float d_deg = 0;
        float f_mm;
        float f_deg;
        float cosA;
        float sinA;
        float y_projected;
        float radius_a;
        float radius_a0;
        float radius_a1;
        float radius_c;
        float radius_c0;
        float radius_c1;

        for (size_t idx = 0; idx < MAX_N_AXIS; ++idx) {
            // there are no B-axis and A-axis is fixed along the X-axis but C-axis is not fixed and can rotate as A-axis changes
            if (delta_in[idx] != 0 && idx == A_AXIS) {
                radius_a0 = sqrtf(position[Y_AXIS] * position[Y_AXIS] + position[Z_AXIS] * position[Z_AXIS]);

                radius_a1 = sqrtf(target[Y_AXIS] * target[Y_AXIS] + target[Z_AXIS] * target[Z_AXIS]);

                delta_out[idx] = DEG_TO_RAD * delta_in[idx] * axis_convert_multiplier->get() * ((radius_a0 + radius_a1) / 2);
            } else if (delta_in[idx] != 0 && idx == C_AXIS) {
                cosA           = cosf(DEG_TO_RAD * position[A_AXIS]);
                sinA           = sinf(DEG_TO_RAD * position[A_AXIS]);
                y_projected    = position[Y_AXIS] * cosA + position[Z_AXIS] * sinA;
                radius_c0      = sqrtf(position[X_AXIS] * position[X_AXIS] + y_projected * y_projected);
                cosA           = cosf(DEG_TO_RAD * target[A_AXIS]);
                sinA           = sinf(DEG_TO_RAD * target[A_AXIS]);
                y_projected    = target[Y_AXIS] * cosA + target[Z_AXIS] * sinA;
                radius_c1      = sqrtf(target[X_AXIS] * target[X_AXIS] + y_projected * y_projected);
                radius_c       = (radius_c0 + radius_c1) / 2;
                delta_out[idx] = DEG_TO_RAD * delta_in[idx] * axis_convert_multiplier->get() * radius_c;
            } else {
                delta_out[idx] = delta_in[idx];
            }

            d_deg += delta_in[idx] * delta_in[idx];
            d_mm += delta_out[idx] * delta_out[idx];
        }

        d_deg = sqrtf(d_deg);
        d_mm  = sqrtf(d_mm);

        f_mm  = pl_data->feed_rate;
        f_deg = (d_deg / d_mm) * f_mm;

        pl_data->feed_rate = f_deg;

        if (rownd_verbose_enable->get()) {
            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "f_in: %.2f, f_out: %.2f", f_mm, f_deg);
        }
    }
#ifdef DEFAULT_ROWND_TCP
    if (tcp_active->get()) {
        target[X_AXIS] = delta_in[X_AXIS];
    }
#endif

    // mc_line() returns false if a jog is cancelled.
    // In that case we stop sending segments to the planner.
    return mc_line(target, pl_data);
}

/*
  user_tool_change() is called when tool change gcode is received,
  to perform appropriate actions for your machine.
*/
Error user_tool_change(uint8_t new_tool) {
    Error oPut = Error::Ok;

#ifdef ROWND_REPORT
    return Error::Ok;
#else
    return oPut;
#endif
}

/*
  options.  user_defined_macro() is called with the button number to
  perform whatever actions you choose.
*/
void user_defined_macro(uint8_t index) {}

/*
  user_m30() is called when an M30 gcode signals the end of a gcode file.
*/
void user_m30() {}

// If you add any additional functions specific to your machine that
// require calls from common code, guard their calls in the common code with
// #ifdef USE_WHATEVER and add function prototypes (also guarded) to grbl.h
