
#include "../src/Grbl.h"

#define RADIUS_AXIS X_AXIS
#define POLAR_AXIS POSITIONABLE_SPINDLE_AXIS

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
    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "home usr def mask: %d", cycle_mask);
    if (bitnum_istrue(cycle_mask, X_AXIS) || bitnum_istrue(cycle_mask, Z_AXIS) || cycle_mask == 0) {
        return false;
    }

    if (bitnum_istrue(cycle_mask, POSITIONABLE_SPINDLE_AXIS)) {
        sys_position[POSITIONABLE_SPINDLE_AXIS] = 0;
        return true;
    }
    if (rownd_verbose_enable->get())
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
 Apply inverse kinematics for a lathe spindle
 
 float target: 					The desired target location in machine space
 plan_line_data_t *pl_data:		Plan information like feed rate, etc
 float *position:				The previous "from" location of the move
*/
bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
    if (rownd_param_experimental_axis_feed->get()) {
        float dx;
        float dz;
        float dc_mm;
        float dc_deg;
        float d_mm;
        float d_deg;
        float f_mm;
        float f_deg;

        dx     = target[X_AXIS] - position[X_AXIS];
        dz     = target[Z_AXIS] - position[Z_AXIS];
        dc_deg = target[POLAR_AXIS] - position[POLAR_AXIS];
        // dc_deg *= axis_convert_multiplier->get();
        if (dc_deg != 0) {
            // if (position[RADIUS_AXIS] <= 0) {
            //     grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "radius should be positive:", position[RADIUS_AXIS]);
            //     return false;
            // }
            dc_mm = DEG_TO_RAD * dc_deg * position[RADIUS_AXIS] * axis_convert_multiplier->get();

            d_deg = sqrtf((dx * dx) + (dz * dz) + (dc_deg * dc_deg));
            d_mm  = sqrtf((dx * dx) + (dz * dz) + (dc_mm * dc_mm));

            f_mm  = pl_data->feed_rate;
            f_deg = (d_deg / d_mm) * f_mm;

            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "f_in: %.2f, f_out: %.2f", f_mm, f_deg);
            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "dx: %.2f, dz: %.2f", dx, dz);
            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "dc_deg: %.2f, dc_mm: %.2f", dc_deg, dc_mm);
        }
    }

    // mc_line() returns false if a jog is cancelled.
    // In that case we stop sending segments to the planner.
    return mc_line(target, pl_data);
}

/*
  user_tool_change() is called when tool change gcode is received,
  to perform appropriate actions for your machine.
*/
Error user_tool_change(uint8_t new_tool) {
    if (!atc_connected->get()) {
        return Error::AtcNotConnected;
    }

    protocol_buffer_synchronize();

    float* curr_mpos = system_get_mpos();  // returns MAX_N_AXIS lenght array

    auto n_axis = number_axis->get();  // is N_AXIS and can be <= MAX_N_AXIS
    for (uint8_t idx = 0; idx < n_axis; idx++) {
        if (idx != REMOVABLE_AXIS_LIMIT && idx != POSITIONABLE_SPINDLE_AXIS) {
            if (curr_mpos[idx] != coords[CoordIndex::G28]->get()[idx]) {
                return Error::AtcNeedsG28;
            }
            if (curr_mpos[idx] != coords[CoordIndex::G28]->get()[idx]) {  // WIP
                return Error::AtcInsufficientRadius;
            }
        }
    }

    Error oPut = Error::Ok;

    bool  is_absolute          = gc_state.modal.distance == Distance::Absolute;
    bool  is_inverseTime       = gc_state.modal.feed_rate == FeedRate::InverseTime;
    bool  is_imperial          = gc_state.modal.units == Units::Inches;
    int   tool_diff            = new_tool - tool_active->get();
    int   tool_move            = (tool_diff < 0) ? tool_diff + tool_count->get() : tool_diff;
    float lock_test_percentage = 0.1;

    // float mult_conv            = homing_pulloff->get() / (atc_distance->get() + atc_offset->get());

    char tc_line[20];

    if (tool_diff == 0) {  // no tool change cycle needed since we're already at the selected tool
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Active Tool No: %d | New Tool No: %d", tool_active->get(), new_tool);
        return Error::Ok;
    } else {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Active Tool No: %d | New Tool No: %d", tool_active->get(), new_tool);
    }

    if (is_inverseTime) {
        gc_state.modal.feed_rate = FeedRate::UnitsPerMin;
    }
    if (is_imperial) {
        gc_state.modal.units = Units::Mm;
    }

    gc_state.Rownd_isAtc = true;

    // old atc
    // snprintf(tc_line, sizeof(tc_line), "G1G90F%.2fA%.2f\r\n", atc_speed->get(), atc_distance->get() * tool_active->get(-1) + atc_distance->get() * atc_offset->get());
    // WebUI::inputBuffer.push(tc_line);  // It's more efficient to add to the buffer instead of executing immediately.

    // ATC Lock Check
    snprintf(tc_line, sizeof(tc_line), "G1G91F%.2fA%.2f", atc_speed->get(), (-atc_offset->get()) * lock_test_percentage);

    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Lock Check:  %s", tc_line);

    oPut = execute_line(tc_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

    if (oPut != Error::Ok) {
        return oPut;
    }

    // ATC Rise Up / Unlock
    snprintf(tc_line, sizeof(tc_line), "G1G91F%.2fA%.2f", atc_speed->get(), atc_distance->get());

    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Rise Up:     %s", tc_line);

    oPut = execute_line(tc_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

    if (oPut != Error::Ok) {
        return oPut;
    }

    // ATC goto Target
    snprintf(tc_line, sizeof(tc_line), "G1G91F%.2fA%.2f", atc_speed->get(), tool_move * atc_offset->get());

    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC goto Target: %s", tc_line);

    oPut = execute_line(tc_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

    if (oPut != Error::Ok) {
        return oPut;
    }

    // ATC Lock Back
    snprintf(tc_line, sizeof(tc_line), "G1G91F%.2fA%.2f", atc_speed->get(), -(atc_distance->get() + atc_offset->get()));

    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "ATC Lock Back:   %s", tc_line);

    oPut = execute_line(tc_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

    if (oPut != Error::Ok) {
        return oPut;
    }

    oPut = tool_active->setValue(new_tool);

    if (oPut != Error::Ok) {
        return oPut;
    }

#ifdef ROWND_REPORT
    report_status_message(oPut, CLIENT_SERIAL);
#endif

    protocol_buffer_synchronize();

    gc_state.Rownd_isAtc = false;

    if (is_absolute) {
        gc_state.modal.distance = Distance::Absolute;
    } else {
        gc_state.modal.distance = Distance::Incremental;
    }
    if (is_inverseTime) {
        gc_state.modal.feed_rate = FeedRate::InverseTime;
    } else {
        gc_state.modal.feed_rate = FeedRate::UnitsPerMin;
    }
    if (is_imperial) {
        gc_state.modal.units = Units::Inches;
    } else {
        gc_state.modal.units = Units::Mm;
    }

#ifdef ROWND_REPORT
    return Error::Ok;
#else
    return oPut;
#endif
}

// This code works, but not as well as we hoped, so we're disabling it for now. We might revisit and improve it in the distant future, but for now, it's on hold.
/*
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
*/

float calculate_G76_feed(float s, float rev, float dz, float dx) {
#ifdef POSITIONABLE_AXIS_CONVERT
    float feed_out = 0;

    if (rev != 0) {
        float duration = rev / s;
        float feed_c   = (s * 360.0 / axis_convert_multiplier->get());
        float feed_z   = dz / duration;
        float feed_x   = dx / duration;
        feed_out       = sqrtf((feed_c * feed_c) + (feed_z * feed_z) + (feed_x * feed_x));
    } else {
        feed_out = axis_settings[X_AXIS]->max_rate->get();
    }

    if (feed_out < 0)
        feed_out *= -1;

    return feed_out;
#else
    return -1;
#endif
}

Error rownd_G76(parser_block_t* gc_block, g76_params_t* g76_params, parser_state_t* gc_state) {
    float pos_start[MAX_N_AXIS];
    char  g76_line[50];
    bool  is_lathe       = static_cast<SpindleType>(spindle_type->get()) == SpindleType::ASDA_CN1;
    bool  is_absolute    = gc_block->modal.distance == Distance::Absolute;
    bool  is_inverseTime = gc_block->modal.feed_rate == FeedRate::InverseTime;
    bool  is_inches      = gc_block->modal.units == Units::Inches;
    float dirMultiplier  = (gc_block->modal.spindle == SpindleState::Ccw) ? -1.0f : 1.0f;
    float feed_in        = gc_block->values.s;
    float feed_out       = 0;
    float magnitude      = 0;
    float rev_total      = 0;
    float rev_enter      = 0;
    float rev_exit       = 0;
    float rev_thread     = 0;
    float rev_smooth     = 1;
    float rev_offset     = 0;
    float mult_smooth    = 1;
    float total_dist     = 0;
    float depth_line     = 0;
    int   pass_count     = 0;
    int   current_pass   = 0;
    float depth_last     = 0;
    float depth_current  = 0;
    Error oPut           = Error::Ok;

    if (is_lathe) {
        gc_state->Rownd_special = true;
        spindle_type->setEnumValue((int8_t)SpindleType::PWM);
        gc_state->Rownd_special = false;
    } else {
        return Error::AsdaMode;
    }

    gc_state->Rownd_special = false;
    gc_state->Rownd_thread  = true;

    protocol_buffer_synchronize();

    if (is_inverseTime) {
        gc_state->modal.feed_rate = FeedRate::UnitsPerMin;
    }

    if (is_inches) {
        gc_state->modal.units = Units::Mm;
    }

    // calculate variables 1 (for loop)

    // No tapered threading only works on Z axis
    for (size_t idx = 0; idx < MAX_N_AXIS; ++idx) {
        pos_start[idx] = gc_state->position[idx] - gc_state->coord_system[idx] - gc_state->coord_offset[idx] - gc_state->tool_length_offset[idx];

#ifdef POSITIONABLE_AXIS_CONVERT
        if (idx == POSITIONABLE_SPINDLE_AXIS) {
            pos_start[idx] *= axis_convert_multiplier->get();
        }
#else
        return Error::AnotherInterfaceBusy;
#endif
        if (rownd_verbose_enable->get())
            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "sta_pos[%c]: %f", "XYZABC"[idx], pos_start[idx]);
    }

    if (g76_params->degression < 1)
        g76_params->degression = 1;

    g76_params->depth_minimum_cut = 0.01;

    g76_params->depth_first_cut = abs(g76_params->depth_first_cut);
    if (g76_params->depth_first_cut <= g76_params->depth_minimum_cut) {
        g76_params->depth_first_cut = g76_params->depth_minimum_cut;
    }

    g76_params->offset_peak = abs(g76_params->offset_peak);

    if (g76_params->depth_thread > 0) {
        // g76_params->depth_minimum_cut *= 1;
        // g76_params->depth_first_cut *= 1;
        g76_params->offset_peak *= -1;
    }
    if (g76_params->depth_thread < 0) {  // new
        g76_params->depth_minimum_cut *= -1;
        g76_params->depth_first_cut *= -1;
        // g76_params->offset_peak *= 1;
    }

    total_dist = gc_block->values.xyz[Z_AXIS] - gc_state->position[Z_AXIS];

    if (total_dist == 0) {
        oPut = Error::GcodeAxisCommandConflict;
    }

    // rev_total = total_dist / (g76_params->pitch * g76_params->start_count);  // tepeden tepeye
    rev_total = total_dist / g76_params->pitch;  // hatve
    if (rev_total < 0)
        rev_total *= -1;
    rev_thread = rev_total;

    if (bit_istrue(g76_params->chamfer_mode, G76_taperModes::Entry)) {
        // rev_enter = g76_params->chamfer_angle / (g76_params->pitch * g76_params->start_count);  // tepeden tepeye
        rev_enter = g76_params->chamfer_angle / g76_params->pitch;  // hatve
        if (rev_total < 0)
            rev_enter *= -1;
        rev_thread -= rev_enter;
    }
    if (bit_istrue(g76_params->chamfer_mode, G76_taperModes::Exit)) {
        // rev_exit = g76_params->chamfer_angle / (g76_params->pitch * g76_params->start_count);  // tepeden tepeye
        rev_exit = g76_params->chamfer_angle / g76_params->pitch;  // hatve
        if (rev_total < 0)
            rev_exit *= -1;
        rev_thread -= rev_exit;
    }

    if (rev_enter <= 0)
        rev_enter = 0;
    if (rev_exit <= 0)
        rev_exit = 0;

    rev_total = rev_enter + rev_thread + rev_exit;

    rev_offset = 360.0f / g76_params->start_count;

    depth_line = g76_params->depth_thread - g76_params->offset_peak;

    pass_count    = 0;
    depth_current = 0;
    depth_last    = g76_params->depth_first_cut;

    if (g76_params->depth_thread == 0) {
        oPut = Error::BadNumberFormat;
    }

    if (rownd_param_G76_ignore_offset->get()) {
        pass_count = ceilf(powf((depth_line + g76_params->offset_peak) / g76_params->depth_first_cut, g76_params->degression));
    } else {
        pass_count = ceilf(powf(depth_line / g76_params->depth_first_cut, g76_params->degression));
    }

    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 Pass Counter: %i (offset ignore: %s)", pass_count, rownd_param_G76_ignore_offset->getStringValue());

    if (rownd_param_G76_ignore_offset->get()) {
        depth_current = -g76_params->offset_peak;
    } else {
        depth_current = 0;
    }

    depth_last = g76_params->depth_first_cut;

    for (int pass = 1; pass <= pass_count + g76_params->spring_pass; pass++) {
        if (oPut != Error::Ok) {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 init error: %i", oPut);
            break;
        }

        if (pass > pass_count) {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 spring pass: %i", pass - pass_count);
        } else {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 pass: %i", pass);
        }

        // calculations
        if ((g76_params->depth_thread > 0 && depth_last < g76_params->depth_minimum_cut) || (g76_params->depth_thread < 0 && depth_last > g76_params->depth_minimum_cut))
            depth_last = g76_params->depth_minimum_cut;

        depth_current += depth_last;

        if ((g76_params->depth_thread > 0 && depth_current > depth_line) || (g76_params->depth_thread < 0 && depth_current < depth_line))
            depth_current = depth_line;

        for (int current_start = 0; current_start < g76_params->start_count; current_start++) {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 start no: %i", current_start + 1);
            // Threading
#pragma region entering smooth
            feed_out = calculate_G76_feed(feed_in, rev_enter + rev_smooth, ((total_dist * rev_enter) / rev_total), depth_current);

            mult_smooth = (rev_smooth / (rev_enter + rev_smooth));

            if (is_absolute) {
                snprintf(g76_line,
                         sizeof(g76_line),
                         "G1G90F%.2fX%.3fZ%.3fC%.2f",
                         feed_out,
                         pos_start[X_AXIS] - (depth_current * mult_smooth),
                         pos_start[Z_AXIS] + (((total_dist * rev_enter) / rev_total) * mult_smooth),
                         pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset) + (dirMultiplier * ((rev_enter + rev_smooth) * 360.0) * mult_smooth));
            } else {
                snprintf(
                    g76_line, sizeof(g76_line), "G1G91F%.2fX%.3fZ%.3fC%.2f", feed_out, -(depth_current * mult_smooth), (((total_dist * rev_enter) / rev_total) * mult_smooth), (dirMultiplier * ((rev_enter + rev_smooth) * 360.0) * mult_smooth));
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 enter 1:       %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 enter error: %i", oPut);
                break;
            }
#pragma endregion

#pragma region entering taper
            if (rev_enter > 0) {
                if (is_absolute) {
                    snprintf(g76_line,
                             sizeof(g76_line),
                             "G1G90F%.2fX%.3fZ%.3fC%.2f",
                             feed_out,
                             pos_start[X_AXIS] - depth_current,
                             pos_start[Z_AXIS] + ((total_dist * rev_enter) / rev_total),
                             pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset) + dirMultiplier * ((rev_enter + rev_smooth) * 360.0));
                } else {
                    snprintf(g76_line,
                             sizeof(g76_line),
                             "G1G91F%.2fX%.3fZ%.3fC%.2f",
                             feed_out,
                             -(depth_current * (1 - mult_smooth)),
                             (((total_dist * rev_enter) / rev_total) * (1 - mult_smooth)),
                             (dirMultiplier * ((rev_enter + rev_smooth) * 360.0) * (1 - mult_smooth)));
                }

                if (rownd_verbose_enable->get())
                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 enter 2:       %s", g76_line);

                oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

                if (oPut != Error::Ok) {
                    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 enter error: %i", oPut);
                    break;
                }
            }
#pragma endregion

#pragma region threading
            feed_out = calculate_G76_feed(feed_in, rev_thread, ((total_dist * rev_thread) / rev_total), 0.0f);

            if (is_absolute) {
                snprintf(g76_line,
                         sizeof(g76_line),
                         "G1G90F%.2fX%.3fZ%.3fC%.2f",
                         feed_out,
                         pos_start[X_AXIS] - depth_current,
                         pos_start[Z_AXIS] + ((total_dist * (rev_thread + rev_enter)) / rev_total),
                         pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset) + dirMultiplier * ((rev_thread + rev_enter + rev_smooth) * 360.0));
            } else {
                snprintf(g76_line, sizeof(g76_line), "G1G91F%.2fX%.3fZ%.3fC%.2f", feed_out, 0.0f, ((total_dist * rev_thread) / rev_total), dirMultiplier * (rev_thread * 360.0));
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 thread:        %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 thread error: %i", oPut);
                break;
            }
#pragma endregion

#pragma region exiting
            feed_out = calculate_G76_feed(feed_in, rev_exit, ((total_dist * rev_exit) / rev_total), depth_current);

            if (is_absolute) {
                snprintf(g76_line,
                         sizeof(g76_line),
                         "G1G90F%.2fX%.3fZ%.3fC%.2f",
                         feed_out,
                         pos_start[X_AXIS],
                         pos_start[Z_AXIS] + total_dist,
                         pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset) + dirMultiplier * ((rev_total + rev_smooth) * 360.0));
            } else {
                snprintf(g76_line, sizeof(g76_line), "G1G91F%.2fX%.3fZ%.3fC%.2f", feed_out, depth_current, ((total_dist * rev_exit) / rev_total), dirMultiplier * (rev_exit * 360.0));
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 exit:          %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 exit error: %i", oPut);
                break;
            }
#pragma endregion

            // returning (safety exit + return + safety enter)
#pragma region safety exit
            if (is_absolute) {
                snprintf(g76_line,
                         sizeof(g76_line),
                         "G0G90X%.3fZ%.3fC%.2f",
                         pos_start[X_AXIS] + g76_params->depth_first_cut,
                         pos_start[Z_AXIS] + total_dist,
                         pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset) + dirMultiplier * ((rev_total + rev_smooth) * 360.0));
            } else {
                snprintf(g76_line, sizeof(g76_line), "G0G91X%.3fZ%.3fC%.2f", g76_params->depth_first_cut, 0.0f, 0.0f);
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/exit:   %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/exit error: %i", oPut);
                break;
            }
#pragma endregion

#pragma region returning
            if (is_absolute) {
                snprintf(g76_line, sizeof(g76_line), "G0G90X%.3fZ%.3fC%.2f", pos_start[X_AXIS] + g76_params->depth_first_cut, pos_start[Z_AXIS], pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset));
            } else {
                snprintf(g76_line, sizeof(g76_line), "G0G91X%.3fZ%.3fC%.2f", 0.0f, -total_dist, dirMultiplier * -((rev_total + rev_smooth) * 360.0));
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/thread: %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/thread error: %i", oPut);
                break;
            }
#pragma endregion

#pragma region safety enter
            if (is_absolute) {
                snprintf(g76_line, sizeof(g76_line), "G0G90X%.3fZ%.3fC%.2f", pos_start[X_AXIS], pos_start[Z_AXIS], pos_start[DEFAULT_SWAP_C] + (dirMultiplier * current_start * rev_offset));
            } else {
                snprintf(g76_line, sizeof(g76_line), "G0G91X%.3fZ%.3fC%.2f", -g76_params->depth_first_cut, 0.0f, 0.0f);
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/enter:  %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/enter error: %i", oPut);
                break;
            }
#pragma endregion

// make sure you are at the correct start position
#pragma region goto next start
            if (is_absolute) {
                snprintf(g76_line, sizeof(g76_line), "G0G90X%.3fZ%.3fC%.2f", pos_start[X_AXIS], pos_start[Z_AXIS], pos_start[DEFAULT_SWAP_C] + (dirMultiplier * (current_start + 1) * rev_offset));
            } else {
                snprintf(g76_line, sizeof(g76_line), "G0G91X%.3fZ%.3fC%.2f", 0.0f, 0.0f, (dirMultiplier * rev_offset));
            }

            if (rownd_verbose_enable->get())
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 goto start:    %s", g76_line);

            oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

            if (oPut != Error::Ok) {
                grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/enter error: %i", oPut);
                break;
            }
#pragma endregion

            protocol_buffer_synchronize();
        }
        // make sure you are at the correct start position
#pragma region return to start
        if (is_absolute) {
            snprintf(g76_line, sizeof(g76_line), "G0G90X%.3fZ%.3fC%.2f", pos_start[X_AXIS], pos_start[Z_AXIS], pos_start[DEFAULT_SWAP_C]);
        } else {
            // snprintf(g76_line, sizeof(g76_line), "G0G91X%.3fZ%.3fC%.2f", 0.0f, 0.0f, -(dirMultiplier * g76_params->start_count * rev_offset));
            snprintf(g76_line, sizeof(g76_line), "G0G91X%.3fZ%.3fC%.2f", 0.0f, 0.0f, -360.0f);
        }

        if (rownd_verbose_enable->get())
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return start:  %s", g76_line);

        oPut = execute_line(g76_line, CLIENT_SERIAL, WebUI::AuthenticationLevel::LEVEL_GUEST);

        if (oPut != Error::Ok) {
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "g76 return/enter error: %i", oPut);
            break;
        }
#pragma endregion

        protocol_buffer_synchronize();

        depth_last = g76_params->depth_first_cut * (powf((pass + 1), (1 / g76_params->degression)) - powf(pass, (1 / g76_params->degression)));
    }

    if (is_lathe) {
        gc_state->Rownd_special = true;
        spindle_type->setEnumValue((int8_t)SpindleType::ASDA_CN1);
        gc_state->Rownd_special = false;
    }

    if (is_inverseTime) {
        gc_state->modal.feed_rate = FeedRate::InverseTime;
    }

    if (is_inches) {
        gc_state->modal.units = Units::Inches;
    }

    gc_state->Rownd_thread = false;

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
