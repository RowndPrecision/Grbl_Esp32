
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
