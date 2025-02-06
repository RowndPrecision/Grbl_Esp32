
#include "../src/Grbl.h"

/*
This function is used as a one time setup for your machine.
*/
void machine_init() {
    // print a startup message to show the kinematics are enable
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "CNC Lathe kinematics Init");
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
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "home usr def mask: %d", cycle_mask);
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "b1: %d, b2: %d, b3: %d", bitnum_istrue(cycle_mask, X_AXIS), bitnum_istrue(cycle_mask, Z_AXIS), cycle_mask == 0);
    if (bitnum_istrue(cycle_mask, X_AXIS) || bitnum_istrue(cycle_mask, Z_AXIS) || cycle_mask == 0) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "b1: %d, b2: %d, b3: %d", bitnum_istrue(cycle_mask, X_AXIS), bitnum_istrue(cycle_mask, Z_AXIS), cycle_mask == 0);
        return false;
    }

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "TODO home: %d", cycle_mask);
    return true;
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
  user_tool_change() is called when tool change gcode is received,
  to perform appropriate actions for your machine.
*/
Error user_tool_change(uint8_t new_tool) {
    if (!atc_connected->get()) {
        return Error::AtcNotConnected;
    }

    bool is_absolute = gc_state.modal.distance == Distance::Absolute;

    char tc_line[20];

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Active Tool No: %d | New Tool No: %d", tool_active->get(), new_tool);

    tool_active->setValue(new_tool);

    snprintf(tc_line, sizeof(tc_line), "G1G90F%.2fA%.2f\r\n", atc_speed->get(), atc_distance->get() * tool_active->get(-1) + atc_distance->get() * atc_offset->get());

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "atc line: %s", tc_line);

    protocol_buffer_synchronize();

    // WebUI::inputBuffer.push(tc_line);  // It's more efficient to add to the buffer instead of executing immediately.

    Error oPut = execute_line(tc_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

#ifdef ROWND_REPORT
    report_status_message(oPut, CLIENT_SERIAL);
#endif

    protocol_buffer_synchronize();

    if (is_absolute) {
        gc_state.modal.distance = Distance::Absolute;
    } else {
        gc_state.modal.distance = Distance::Incremental;
    }

#ifdef ROWND_REPORT
    return Error::Ok;
#else
    return oPut;
#endif
}

Error rownd_G33(parser_block_t* gc_block, float* position) {
    float pos_diff[MAX_N_AXIS];
    char  g33_line[50];
    bool  is_lathe    = static_cast<SpindleType>(spindle_type->get()) == SpindleType::ASDA_CN1;
    bool  is_absolute = gc_block->modal.distance == Distance::Absolute;
    float feed_in     = 0;
    float feed_out    = 0;
    float revolution  = 0;
    float total_dist  = 0;

    if (is_lathe) {
        gc_state.Rownd_special = true;
        spindle_type->setEnumValue((int8_t)SpindleType::PWM);
        gc_state.Rownd_special = false;
    } else {
        return Error::AsdaMode;
    }

    for (size_t idx = 0; idx < MAX_N_AXIS; ++idx) {
        pos_diff[idx] = gc_block->values.xyz[idx] - position[idx];
        total_dist += pos_diff[idx];
    }

    protocol_buffer_synchronize();

    if (gc_block->values.f > 0) {
        feed_in = gc_block->values.f;
    } else if (gc_block->values.ijk[Z_AXIS] > 0) {
        feed_in = gc_block->values.ijk[Z_AXIS];
    } else {
        return Error::GcodeValueWordMissing;
    }

    revolution = total_dist / feed_in;

    pos_diff[DEFAULT_SWAP_C] = revolution * 360.0f;

    feed_out = (gc_block->values.s * 360.0) * ((pos_diff[DEFAULT_SWAP_C] + total_dist) / pos_diff[DEFAULT_SWAP_C]);

    if (is_absolute) {
        snprintf(g33_line, sizeof(g33_line), "G1G90F%.2fX%.2fZ%.2fC%.2f\r\n", feed_out, gc_block->values.xyz[X_AXIS], gc_block->values.xyz[Z_AXIS], position[DEFAULT_SWAP_C] + pos_diff[DEFAULT_SWAP_C]);
    } else {
        snprintf(g33_line, sizeof(g33_line), "G1G91F%.2fX%.2fZ%.2fC%.2f\r\n", feed_out, pos_diff[X_AXIS], pos_diff[Z_AXIS], pos_diff[DEFAULT_SWAP_C]);
    }

    // grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g33 line: %s", g33_line);

    Error oPut = execute_line(g33_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

    // grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g33 out: %i", oPut);

    if (is_lathe) {
        gc_state.Rownd_special = true;
        spindle_type->setEnumValue((int8_t)SpindleType::ASDA_CN1);
        gc_state.Rownd_special = false;
    }

#ifdef ROWND_REPORT
    report_status_message(oPut, CLIENT_SERIAL);
#endif

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
