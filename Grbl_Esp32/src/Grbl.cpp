/*
  Grbl.cpp - Initialization and main loop for Grbl
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modifed for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Grbl.h"
#include <WiFi.h>

void grbl_init() {
#ifdef USE_I2S_OUT
    i2s_out_init();  // The I2S out must be initialized before it can access the expanded GPIO port
#endif
    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.enableSTA(false);
    WiFi.enableAP(false);
    WiFi.mode(WIFI_OFF);
    client_init();  // Setup serial baud rate and interrupts
    display_init();

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "CNCLathe Ver %s Date %s", CNC_VERSION, CNC_VERSION_BUILD);      // print grbl_esp32 verion info
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Grbl_ESP32 Ver %s Date %s", GRBL_VERSION, GRBL_VERSION_BUILD);  // print grbl_esp32 verion info
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Compiled with ESP32 SDK:%s", ESP.getSdkVersion());              // print the SDK version
// show the map name at startup
#ifdef MACHINE_NAME
    report_machine_type(CLIENT_SERIAL);
#endif
    settings_init();  // Load Grbl settings from non-volatile storage
    stepper_init();   // Configure stepper pins and interrupt timers
    system_ini();     // Configure pinout pins and pin-change interrupt (Renamed due to conflict with esp32 files)
    init_motors();
    memset(sys_position, 0, sizeof(sys_position));  // Clear machine position.
    machine_init();                                 // weak definition in Grbl.cpp does nothing

    led_state->setBoolValue(true);

    if (rownd_verbose_enable->get())
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Led test");

    // Initialize system state.
#ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = State::Alarm;
#else
    sys.state = State::Idle;
#endif
    // Check for power-up and set system alarm if homing is enabled to force homing cycle
    // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
    // startup scripts, but allows access to settings and internal commands. Only a homing
    // cycle '$H' or kill alarm locks '$X' will disable the alarm.
    // NOTE: The startup script will run after successful completion of the homing cycle, but
    // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
    // things uncontrollably. Very bad.
#ifdef HOMING_INIT_LOCK
    if (homing_enable->get()) {
        sys.state = State::Alarm;
    }
#endif
    Spindles::Spindle::select();
#ifdef ENABLE_WIFI
    WebUI::wifi_config.begin();
#endif
#ifdef ENABLE_BLUETOOTH
    WebUI::bt_config.begin();
#endif
    WebUI::inputBuffer.begin();
}

static void reset_variables() {
    // Reset system variables.
    State prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t));  // Clear system struct variable.
    sys.state             = prior_state;
    sys.f_override        = FeedOverride::Default;              // Set to 100%
    sys.r_override        = RapidOverride::Default;             // Set to 100%
    sys.spindle_speed_ovr = SpindleSpeedOverride::Default;      // Set to 100%
    memset(sys_probe_position, 0, sizeof(sys_probe_position));  // Clear probe position.

    sys_probe_state                      = Probe::Off;
    sys_rt_exec_state.value              = 0;
    sys_rt_exec_accessory_override.value = 0;
    sys_rt_exec_alarm                    = ExecAlarm::None;
    cycle_stop                           = false;
    sys_rt_f_override                    = FeedOverride::Default;
    sys_rt_r_override                    = RapidOverride::Default;
    sys_rt_s_override                    = SpindleSpeedOverride::Default;

    // Reset Grbl primary systems.
    client_reset_read_buffer(CLIENT_ALL);
    gc_init();  // Set g-code parser to default state
    spindle->stop();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset();  // Clear block buffer and planner variables
    st_reset();    // Clear stepper subsystem variables
    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();
    report_init_message(CLIENT_ALL);

    // used to keep track of a jog command sent to mc_line() so we can cancel it.
    // this is needed if a jogCancel comes along after we have already parsed a jog and it is in-flight.
    sys_pl_data_inflight = NULL;
}

void run_once() {
    reset_variables();
    // Start Grbl main loop. Processes program inputs and executes them.
    // This can exit on a system abort condition, in which case run_once()
    // is re-executed by an enclosing loop.
    protocol_main_loop();
}

void __attribute__((weak)) machine_init() {}

void __attribute__((weak)) display_init() {}

void __attribute__((weak)) user_m30() {}

// Error __attribute__((weak)) user_tool_change(uint8_t new_tool) {
//     return Error::Ok;
// }

Error __attribute__((weak)) rownd_G33(parser_block_t* gc_block, float* position) {
    return Error::Ok;
}

// Error __attribute__((weak)) rownd_G76(parser_block_t* gc_block,  parser_state_t* gc_state) {
//     return Error::Ok;
// }

bool user_defined_homing(uint8_t cycle_mask) {
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "home usr def mask: %d", cycle_mask);
    if (bitnum_istrue(cycle_mask, X_AXIS) || bitnum_istrue(cycle_mask, Z_AXIS) || cycle_mask == 0) {
        return false;
    }

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "TODO home: %d", cycle_mask);
    return false;
}

Error user_tool_change(uint8_t new_tool) {
    if (!atc_connected->get()) {
        return Error::AtcNotConnected;
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

    protocol_buffer_synchronize();

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

float calculate_G76_feed(float s, float rev, float dz, float dx) {
    float feed_out = 0;

    if (rev != 0) {
        float duration = rev / s;
        float feed_c   = (s * 360.0 / axis_convet_multiplier->get());
        float feed_z   = dz / duration;
        float feed_x   = dx / duration;
        feed_out       = sqrtf((feed_c * feed_c) + (feed_z * feed_z) + (feed_x * feed_x));
    } else {
        feed_out = axis_settings[X_AXIS]->max_rate->get();
    }

    if (feed_out < 0)
        feed_out *= -1;

    return feed_out;
}

Error rownd_G76(parser_block_t* gc_block, g76_params_t* g76_params, parser_state_t* gc_state) {
    // float pos_diff[MAX_N_AXIS];
    float pos_start[MAX_N_AXIS];
    char  g76_line[50];
    bool  is_lathe       = static_cast<SpindleType>(spindle_type->get()) == SpindleType::ASDA_CN1;
    bool  is_absolute    = gc_block->modal.distance == Distance::Absolute;
    bool  is_inverseTime = gc_block->modal.feed_rate == FeedRate::InverseTime;
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
        gc_block->modal.feed_rate = FeedRate::UnitsPerMin;
    }

    // calculate variables 1 (for loop)

    // No tapered threading only works on Z axis
    for (size_t idx = 0; idx < MAX_N_AXIS; ++idx) {
        pos_start[idx] = gc_state->position[idx] - gc_state->coord_system[idx] - gc_state->coord_offset[idx] - gc_state->tool_length_offset[idx];
        // pos_diff[idx]  = gc_block->values.xyz[idx] - pos_start[idx];
        // total_dist += pos_diff[idx];
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
        gc_block->modal.feed_rate = FeedRate::InverseTime;
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
  setup() and loop() in the Arduino .ino implements this control flow:

  void main() {
     init();          // setup()
     while (1) {      // loop()
         run_once();
     }
  }
*/
