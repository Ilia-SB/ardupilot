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

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_FishFinder.h"

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_RangeFinder_FishFinder::var_info[] = {
    // @Param: FF_POL
    // @DisplayName: FishFinder polling interval
    // @Description: How often to request data from fish finder (ms)
    // @Range: 200 5000
    // @User: Advanced
    AP_GROUPINFO("FF_POL", 1, AP_RangeFinder_FishFinder, poll_interval, 1000),

    AP_GROUPEND
};

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_FishFinder::AP_RangeFinder_FishFinder(RangeFinder::RangeFinder_State &_state,
                                                        AP_RangeFinder_Params &_params,
                                                        uint8_t serial_instance) : 
    AP_RangeFinder_Backend(_state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);

    const AP_SerialManager &serial_manager = AP::serialmanager(); //get serial manager instance
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance); //get uart
    if (uart != nullptr) {
        uint32_t baud_rate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
        DEBUG_PRINTF("FishFinder: Starting UART @ %" PRIu32 "\r", baud_rate);
        uart->begin(baud_rate);
        state.var_info = var_info;
    }
}

/* 
   detect if a FishFinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_FishFinder::detect(uint8_t serial_instance)
{
    hal.console -> printf("!!!FishFinder detect!!!\r");
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

bool AP_RangeFinder_FishFinder::get_reading(FishFinderData &ff_data)
{
    if (uart == nullptr)
    {
        return false;
    }

    if ((signed)(AP_HAL::millis() - state.last_reading_ms) > poll_interval)
    {
        DEBUG_PRINTF("FishFinder::poll_interval: %d\r", (int16_t)poll_interval);
        DEBUG_PRINTF("FishFinder::request data\r");
        uart->write('d');
        state.last_reading_ms = AP_HAL::millis();
        DEBUG_PRINTF("FishFinder::and return\r");
#ifdef FISHFINDER_DEBUG
        _data_requested = true;
#endif
        return false;
    }
    ff_data.device_active = false;
    int16_t bytes_available = uart->available();
    while (bytes_available-- > 0)
    {
        char c = uart->read();
        if (c != '\r') // then store the byte in buffer
        {
            rx_buffer[rx_buffer_pos++] = c;
            if (rx_buffer_pos > sizeof(rx_buffer)) // overflow protection
            {
                DEBUG_PRINTF("FishFinder: Rx buffer overflow while reading data!");
                rx_buffer_pos = 0;
                return false;
            }
        }
        else
        {
            rx_buffer_pos = 0; // end of transmission
        }

        if (rx_buffer_pos == 0)
        {
            memcpy(&ff_data, rx_buffer, sizeof(ff_data)); //copy data to struct
            _has_unreported_data = true; //for use in mavlink message
            return true;
        }
        else
        {
            return false;
        }
    }
#ifdef FISHFINDER_DEBUG
    if (_data_requested) {
        _data.device_active = true;
        _data.battery_level = 2;
        _data.depth = abs(rand_float())*100;
        _data.large_fish_depth = abs(rand_float())*100;
        _data.medium_fish_depth = abs(rand_float())*100;
        _data.small_fish_depth = abs(rand_float())*100;
        _data.temperature = abs(rand_float())*100;
        _data.wifi_active = true;

        _has_unreported_data = true;
        _data_requested = false;
        return true;
    }
    else
    {
        return false;
    }
#endif
    return false;
}

void AP_RangeFinder_FishFinder::update(void)
{
    if (get_reading(_data))
    {
        state.last_reading_ms = AP_HAL::millis();
        state.distance_cm = _data.depth;
        update_status();
    }
    else if ((signed)(AP_HAL::millis() - state.last_reading_ms) > poll_interval)
    {
        DEBUG_PRINTF("%" PRIu32 " <?> %d\r", (signed)(AP_HAL::millis() - state.last_reading_ms), (int16_t)poll_interval);
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

bool AP_RangeFinder_FishFinder::fish_detected(void)
{
    if (large_fish_depth_cm() > 0.0f || medium_fish_depth_cm() > 0.0f || small_fish_depth_cm() > 0.0f)
    {
        return true;
    }
    else
    {
        return false;
    }
}