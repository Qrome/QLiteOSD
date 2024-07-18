/**
 * Contains the data for any dynamic configuration options
*/
#include "config.h"
#include "MSP_OSD.h"

const configValue_t configValues[CONFIG_VALUE_COUNT] = {
    {"CRAFT_NAME",&CRAFT_NAME,CONFIG_VALUE_STRING,14}, // 14 character limit
    {"USE_IMPERIAL_UNITS",&USE_IMPERIAL_UNITS,CONFIG_VALUE_BOOL},
    {"USE_PWM_ARM",&USE_PWM_ARM,CONFIG_VALUE_BOOL},

    {"BOARD_VCC", &BOARD_VCC, CONFIG_VALUE_FLOAT},

#ifdef USE_LEDS
    {"RGB_MODE", &RGB_MODE, CONFIG_VALUE_STRING, 16},
    {"RED_VALUE", &RED_VALUE, CONFIG_VALUE_UINT8},
    {"GREEN_VALUE", &GREEN_VALUE, CONFIG_VALUE_UINT8},
    {"BLUE_VALUE", &BLUE_VALUE, CONFIG_VALUE_UINT8},
#endif

    {"OSD_ALTITUDE_POS", &msp_osd_config.osd_altitude_pos, CONFIG_VALUE_UINT16},
    {"OSD_AVG_CELL_VOLTAGE_POS", &msp_osd_config.osd_avg_cell_voltage_pos, CONFIG_VALUE_UINT16},
    {"OSD_MAIN_BATT_VOLTAGE_POS", &msp_osd_config.osd_main_batt_voltage_pos, CONFIG_VALUE_UINT16},
    {"OSD_CRAFTNAME_POS", &msp_osd_config.osd_craft_name_pos, CONFIG_VALUE_UINT16},
    {"OSD_GPS_SATS_POS", &OSD_GPS_SATS_POS, CONFIG_VALUE_UINT16},
    {"OSD_HOME_DIR_POS", &msp_osd_config.osd_home_dir_pos, CONFIG_VALUE_UINT16},
    {"OSD_HOME_DIST_POS", &msp_osd_config.osd_home_dist_pos, CONFIG_VALUE_UINT16},
    {"OSD_GPS_SPEED_POS", &msp_osd_config.osd_gps_speed_pos, CONFIG_VALUE_UINT16},
    {"OSD_GPS_LAT_POS", &msp_osd_config.osd_gps_lat_pos, CONFIG_VALUE_UINT16},
    {"OSD_GPS_LON_POS", &msp_osd_config.osd_gps_lon_pos, CONFIG_VALUE_UINT16},
    {"OSD_CROSSHAIRS_POS", &msp_osd_config.osd_crosshairs_pos, CONFIG_VALUE_UINT16}
};

char CRAFT_NAME[15] = CRAFT_NAME_DEFAULT; // Do not make larger than 14 characters

#ifdef USE_LEDS

char RGB_MODE[16] = RGB_MODE_DEFAULT;

uint8_t RED_VALUE = RED_VALUE_DEFAULT;
uint8_t GREEN_VALUE = GREEN_VALUE_DEFAULT;
uint8_t BLUE_VALUE = BLUE_VALUE_DEFAULT;

#endif

bool USE_IMPERIAL_UNITS = USE_IMPERIAL_UNITS_DEFAULT;  // Set to false to see units in Metric

bool USE_PWM_ARM = USE_PWM_ARM_DEFAULT;

float BOARD_VCC = BOARD_VCC_DEFAULT;

uint16_t OSD_GPS_SATS_POS = GPS_SATS_POS_DEFAULT;

/**
 * Default values for the OSD Values are given here, some OSD elements
 * here may not be implemented, so using the web interface for config is reccomended.
 **/
msp_osd_config_t msp_osd_config = {
    0,                   // osd_flags
    0,                   // video_system
    0,                   // units (0 for imperial, 1 for metric)
    0,                   // rssi_alarm
    0,                   // cap_alarm
    0,                   // old_timer_alarm
    56,                  // osd_item_count
    0,                   // alt_alarm
    OSD_HIDDEN,          // osd_rssi_value_pos
    MAIN_BATT_VOLT_POS_DEFAULT,  // osd_main_batt_voltage_pos
    CROSSHAIRS_POS_DEFAULT,      // osd_crosshairs_pos
    OSD_HIDDEN,          // osd_artificial_horizon_pos
    OSD_HIDDEN,          // osd_horizon_sidebars_pos
    OSD_HIDDEN,          // osd_item_timer_1_pos
    OSD_HIDDEN,          // osd_item_timer_2_pos
    OSD_HIDDEN,          // osd_flymode_pos
    CRAFTNAME_POS_DEFAULT,       // osd_craft_name_pos
    OSD_HIDDEN,          // osd_throttle_pos_pos
    OSD_HIDDEN,          // osd_vtx_channel_pos
    OSD_HIDDEN,          // osd_current_draw_pos
    OSD_HIDDEN,          // osd_mah_drawn_pos
    GPS_SPEED_POS_DEFAULT,       // osd_gps_speed_pos
    GPS_SATS_POS_DEFAULT,        // osd_gps_sats_pos
    ALTITUDE_POS_DEFAULT,        // osd_altitude_pos
    OSD_HIDDEN,          // osd_roll_pids_pos
    OSD_HIDDEN,          // osd_pitch_pids_pos
    OSD_HIDDEN,          // osd_yaw_pids_pos
    OSD_HIDDEN,          // osd_power_pos
    OSD_HIDDEN,          // osd_pidrate_profile_pos
    OSD_HIDDEN,          // osd_warnings_pos
    AVG_CELL_VOLT_POS_DEFAULT,   // osd_avg_cell_voltage_pos
    GPS_LON_POS_DEFAULT,         // osd_gps_lon_pos
    GPS_LAT_POS_DEFAULT,         // osd_gps_lat_pos
    OSD_HIDDEN,          // osd_debug_pos
    OSD_HIDDEN,          // osd_pitch_angle_pos
    OSD_HIDDEN,          // osd_roll_angle_pos
    OSD_HIDDEN,          // osd_main_batt_usage_pos
    OSD_HIDDEN,          // osd_disarmed_pos
    GPS_HOME_DIR_POS_DEFAULT,    // osd_home_dir_pos
    GPS_HOME_DIST_POS_DEFAULT,   // osd_home_dist_pos
    OSD_HIDDEN,          // osd_numerical_heading_pos
    NUM_VARIO_POS_DEFAULT,       // osd_numerical_vario_pos
    OSD_HIDDEN,          // osd_compass_bar_pos
    OSD_HIDDEN,          // osd_esc_tmp_pos
    OSD_HIDDEN,          // osd_esc_rpm_pos
    OSD_HIDDEN,          // osd_remaining_time_estimate_pos
    OSD_HIDDEN,          // osd_rtc_datetime_pos
    OSD_HIDDEN,          // osd_adjustment_range_pos
    OSD_HIDDEN,          // osd_core_temperature_pos
    OSD_HIDDEN,          // osd_anti_gravity_pos
    OSD_HIDDEN,          // osd_g_force_pos
    OSD_HIDDEN,          // osd_motor_diag_pos
    OSD_HIDDEN,          // osd_log_status_pos
    OSD_HIDDEN,          // osd_flip_arrow_pos
    OSD_HIDDEN,          // osd_link_quality_pos
    OSD_HIDDEN,          // osd_flight_dist_pos
    OSD_HIDDEN,          // osd_stick_overlay_left_pos
    OSD_HIDDEN,          // osd_stick_overlay_right_pos
    OSD_HIDDEN,          // osd_display_name_pos
    OSD_HIDDEN,          // osd_esc_rpm_freq_pos
    OSD_HIDDEN,          // osd_rate_profile_name_pos
    OSD_HIDDEN,          // osd_pid_profile_name_pos
    OSD_HIDDEN,          // osd_profile_name_pos
    OSD_HIDDEN,          // osd_rssi_dbm_value_pos
    OSD_HIDDEN,          // osd_rc_channels_pos
    24,                  // osd_stat_count
    0,                   // osd_stat_rtc_date_time
    0,                   // osd_stat_timer_1
    0,                   // osd_stat_timer_2
    0,                   // osd_stat_max_speed
    0,                   // osd_stat_max_distance
    0,                   // osd_stat_min_battery
    0,                   // osd_stat_end_battery
    0,                   // osd_stat_battery
    0,                   // osd_stat_min_rssi
    0,                   // osd_stat_max_current
    0,                   // osd_stat_used_mah
    0,                   // osd_stat_max_altitude
    0,                   // osd_stat_blackbox
    0,                   // osd_stat_blackbox_number
    0,                   // osd_stat_max_g_force
    0,                   // osd_stat_max_esc_temp
    0,                   // osd_stat_max_esc_rpm
    0,                   // osd_stat_min_link_quality
    0,                   // osd_stat_flight_distance
    0,                   // osd_stat_max_fft
    0,                   // osd_stat_total_flights
    0,                   // osd_stat_total_time
    0,                   // osd_stat_total_dist
    0,                   // osd_stat_min_rssi_dbm
    2,                   // osd_timer_count
    0,                   // osd_timer_1
    0,                   // osd_timer_2
    0,                   // enabledwarnings
    16,                  // osd_warning_count
    0,                   // enabledwarnings_1_41_plus
    1,                   // osd_profile_count
    1,                   // osdprofileindex
    0,                   // overlay_radio_mode
};
