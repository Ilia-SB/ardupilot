/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define FISHFINDER_DEBUG

#ifdef FISHFINDER_DEBUG
    #define DEBUG_PRINTF(...)         hal.console->printf(__VA_ARGS__)
# else
    #define DEBUG_PRINTF(...)
#endif

class AP_RangeFinder_FishFinder : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_FishFinder(RangeFinder::RangeFinder_State &_state,
                                AP_RangeFinder_Params &_params,
                                uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;

    struct PACKED FishFinderData
    {
        bool wifi_active;
        bool device_active;
        float depth;
        float temperature;
        float large_fish_depth;
        float medium_fish_depth;
        float small_fish_depth;
        uint8_t battery_level;
    };

    bool has_unreported_data(void) {return _has_unreported_data;}
    void set_data_reported(void) {_has_unreported_data = false;}
    bool wifi_active(void) {return _data.wifi_active;}
    bool device_active(void) {return _data.device_active;}
    bool fish_detected(void);
    float depth_cm(void) {return _data.depth;}
    float temperature_deg(void) {return _data.temperature;}
    float large_fish_depth_cm(void) {return _data.large_fish_depth;}
    float medium_fish_depth_cm(void) {return _data.medium_fish_depth;}
    float small_fish_depth_cm(void) {return _data.small_fish_depth;}
    uint8_t battery_level(void) {return _data.battery_level;}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
#ifdef FISHFINDER_DEBUG
    bool _data_requested = false;
#endif
    FishFinderData _data;
    bool _has_unreported_data = false;

    // get a reading
    bool get_reading(FishFinderData&);

    AP_HAL::UARTDriver *uart = nullptr; // pointer to serial uart
    char rx_buffer[64]; // serial buffer
    uint8_t rx_buffer_pos = 0; // position in serial buffer
    AP_Int16 poll_interval; //how often to poll fishfinder
};