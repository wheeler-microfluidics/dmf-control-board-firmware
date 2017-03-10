/*
Copyright 2011 Ryan Fobel

This file is part of dmf_control_board.

dmf_control_board is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

dmf_control_board is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with dmf_control_board.  If not, see <http://www.gnu.org/licenses/>.
*/

#if !( defined(AVR) || defined(__SAM3X8E__) ) // this file is not compiled by the Arduino IDE

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "DMFControlBoard.h"

using namespace boost::python;

const uint16_t RemoteObject::PERSISTENT_PIN_MODE_ADDRESS;
const uint16_t RemoteObject::PERSISTENT_PIN_STATE_ADDRESS;
const uint16_t RemoteObject::PERSISTENT_BAUD_RATE_ADDRESS;
const uint16_t RemoteObject::PERSISTENT_SERIAL_NUMBER_ADDRESS;
const uint8_t RemoteObject::RETURN_OK;
const uint8_t RemoteObject::RETURN_GENERAL_ERROR;
const uint8_t RemoteObject::RETURN_UNKNOWN_COMMAND;
const uint8_t RemoteObject::RETURN_TIMEOUT;
const uint8_t RemoteObject::RETURN_NOT_CONNECTED;
const uint8_t RemoteObject::RETURN_BAD_INDEX;
const uint8_t RemoteObject::RETURN_BAD_PACKET_SIZE;
const uint8_t RemoteObject::RETURN_BAD_CRC;
const uint8_t RemoteObject::RETURN_BAD_VALUE;
const uint8_t RemoteObject::RETURN_MAX_PAYLOAD_EXCEEDED;
const uint16_t RemoteObject::MAX_PAYLOAD_LENGTH;
const uint8_t DMFControlBoard::SINE;
const uint8_t DMFControlBoard::SQUARE;
const uint16_t DMFControlBoard::PERSISTENT_CONFIG_SETTINGS;
const uint8_t DMFControlBoard::RETURN_MAX_CURRENT_EXCEEDED;

/* # Remote object command codes # */
const uint8_t RemoteObject::CMD_ANALOG_READ;
const uint8_t RemoteObject::CMD_ANALOG_WRITE;
const uint8_t RemoteObject::CMD_DIGITAL_READ;
const uint8_t RemoteObject::CMD_DIGITAL_WRITE;
const uint8_t RemoteObject::CMD_GET_ADC_PRESCALER;
const uint8_t RemoteObject::CMD_GET_AREF;
const uint8_t RemoteObject::CMD_GET_DEBUG_BUFFER;
const uint8_t RemoteObject::CMD_GET_DEVICE_NAME;
const uint8_t RemoteObject::CMD_GET_HARDWARE_VERSION;
const uint8_t RemoteObject::CMD_GET_MANUFACTURER;
const uint8_t RemoteObject::CMD_GET_MCU_TYPE;
const uint8_t RemoteObject::CMD_GET_PROTOCOL_NAME;
const uint8_t RemoteObject::CMD_GET_PROTOCOL_VERSION;
const uint8_t RemoteObject::CMD_GET_SAMPLING_RATE;
const uint8_t RemoteObject::CMD_GET_SOFTWARE_VERSION;
const uint8_t RemoteObject::CMD_GET_URL;
const uint8_t RemoteObject::CMD_I2C_READ;
const uint8_t RemoteObject::CMD_I2C_SCAN;
const uint8_t RemoteObject::CMD_I2C_WRITE;
const uint8_t RemoteObject::CMD_ONEWIRE_GET_ADDRESS;
const uint8_t RemoteObject::CMD_ONEWIRE_READ;
const uint8_t RemoteObject::CMD_ONEWIRE_WRITE;
const uint8_t RemoteObject::CMD_PERSISTENT_READ;
const uint8_t RemoteObject::CMD_PERSISTENT_WRITE;
const uint8_t RemoteObject::CMD_SET_ADC_PRESCALER;
const uint8_t RemoteObject::CMD_SET_PIN_MODE;
const uint8_t RemoteObject::CMD_SET_SAMPLING_RATE;
const uint8_t RemoteObject::CMD_SPI_SET_BIT_ORDER;
const uint8_t RemoteObject::CMD_SPI_SET_CLOCK_DIVIDER;
const uint8_t RemoteObject::CMD_SPI_SET_DATA_MODE;
const uint8_t RemoteObject::CMD_SPI_TRANSFER;

/* # Control board command codes # */
const uint8_t DMFControlBoard::CMD_DEBUG_MESSAGE;
const uint8_t DMFControlBoard::CMD_DEBUG_ON;
const uint8_t DMFControlBoard::CMD_GET_AMPLIFIER_GAIN;
const uint8_t DMFControlBoard::CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN;
const uint8_t DMFControlBoard::CMD_GET_NUMBER_OF_CHANNELS;
const uint8_t DMFControlBoard::CMD_GET_SERIES_CAPACITANCE;
const uint8_t DMFControlBoard::CMD_GET_SERIES_RESISTANCE;
const uint8_t DMFControlBoard::CMD_GET_SERIES_RESISTOR_INDEX;
const uint8_t DMFControlBoard::CMD_GET_STATE_OF_ALL_CHANNELS;
const uint8_t DMFControlBoard::CMD_GET_STATE_OF_CHANNEL;
const uint8_t DMFControlBoard::CMD_GET_WATCHDOG_ENABLED;
const uint8_t DMFControlBoard::CMD_GET_WATCHDOG_STATE;
const uint8_t DMFControlBoard::CMD_GET_WAVEFORM;
const uint8_t DMFControlBoard::CMD_GET_WAVEFORM_FREQUENCY;
const uint8_t DMFControlBoard::CMD_GET_WAVEFORM_VOLTAGE;
const uint8_t DMFControlBoard::CMD_LOAD_CONFIG;
const uint8_t DMFControlBoard::CMD_MEASURE_IMPEDANCE;
const uint8_t DMFControlBoard::CMD_SWEEP_CHANNELS;
const uint8_t DMFControlBoard::CMD_SET_AMPLIFIER_GAIN;
const uint8_t DMFControlBoard::CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN;
const uint8_t DMFControlBoard::CMD_SET_SERIES_CAPACITANCE;
const uint8_t DMFControlBoard::CMD_SET_SERIES_RESISTANCE;
const uint8_t DMFControlBoard::CMD_SET_SERIES_RESISTOR_INDEX;
const uint8_t DMFControlBoard::CMD_SET_STATE_OF_ALL_CHANNELS;
const uint8_t DMFControlBoard::CMD_SET_STATE_OF_CHANNEL;
const uint8_t DMFControlBoard::CMD_SET_WATCHDOG_ENABLED;
const uint8_t DMFControlBoard::CMD_SET_WATCHDOG_STATE;
const uint8_t DMFControlBoard::CMD_SET_WAVEFORM;
const uint8_t DMFControlBoard::CMD_SET_WAVEFORM_FREQUENCY;
const uint8_t DMFControlBoard::CMD_SET_WAVEFORM_VOLTAGE;
const uint8_t DMFControlBoard::CMD_SYSTEM_RESET;
#if ___ATX_POWER_CONTROL___
const uint8_t DMFControlBoard::CMD_GET_POWER_SUPPLY_PIN;
const uint8_t DMFControlBoard::CMD_GET_ATX_POWER_STATE;
const uint8_t DMFControlBoard::CMD_SET_ATX_POWER_STATE;
#endif

BOOST_PYTHON_MODULE(dmf_control_board_base)
{
  scope().attr("INPUT") = 0;
  scope().attr("OUTPUT") = 1;
  scope().attr("HIGH") = 1;
  scope().attr("LOW") = 0;
  scope().attr("SINE") = DMFControlBoard::SINE;
  scope().attr("SQUARE") = DMFControlBoard::SQUARE;

  class_<std::vector<uint8_t> >("uint8_tVector")
    .def(vector_indexing_suite<std::vector<uint8_t> >())
  ;

  class_<std::vector<int8_t> >("int8_tVector")
    .def(vector_indexing_suite<std::vector<int8_t> >())
  ;

  class_<std::vector<uint16_t> >("uint16_tVector")
    .def(vector_indexing_suite<std::vector<uint16_t> >())
  ;

  class_<std::vector<int16_t> >("int16_tVector")
    .def(vector_indexing_suite<std::vector<int16_t> >())
  ;

  class_<std::vector<float> >("floatVector")
    .def(vector_indexing_suite<std::vector<float> >())
  ;

object DMFControlBoard_class
  = class_<DMFControlBoard,boost::noncopyable>("DMFControlBoard")
    .def("connect",&DMFControlBoard::connect)
    .def("disconnect",&DMFControlBoard::disconnect)
    .def("connected",&DMFControlBoard::connected)
    .def("return_code",&DMFControlBoard::return_code)
    .def("set_debug",&DMFControlBoard::set_debug)
    .def("protocol_name",&DMFControlBoard::protocol_name)
    .def("protocol_version",&DMFControlBoard::protocol_version)
    .def("name",&DMFControlBoard::name)
    .def("manufacturer",&DMFControlBoard::manufacturer)
    .def("software_version",&DMFControlBoard::software_version)
    .def("hardware_version",&DMFControlBoard::hardware_version)
    .def("url",&DMFControlBoard::url)
    .def("mcu_type",&DMFControlBoard::mcu_type)
    .def("set_pin_mode",&DMFControlBoard::set_pin_mode)
    .def("digital_read",&DMFControlBoard::digital_read)
    .def("digital_write",&DMFControlBoard::digital_write)
    .def("analog_read",&DMFControlBoard::analog_read)
    .def("analog_reads",&DMFControlBoard::analog_reads)
    .def("analog_write",&DMFControlBoard::analog_write)
    .def("persistent_read",&DMFControlBoard::persistent_read)
    .def("_persistent_write",&DMFControlBoard::persistent_write)
    .def("serial_data_available",&DMFControlBoard::serial_data_available)
    .def("onewire_address",&DMFControlBoard::onewire_address)
    .def("onewire_read",&DMFControlBoard::onewire_read)
    .def("onewire_write",&DMFControlBoard::onewire_write)
    .def("i2c_read",&DMFControlBoard::i2c_read)
    .def("i2c_write",&DMFControlBoard::i2c_write)
    .def("i2c_send_command",&DMFControlBoard::i2c_send_command)
    .def("i2c_scan",&DMFControlBoard::i2c_scan)
    .def("spi_set_bit_order",&DMFControlBoard::spi_set_bit_order)
    .def("spi_set_clock_divider",&DMFControlBoard::spi_set_clock_divider)
    .def("spi_set_data_mode",&DMFControlBoard::spi_set_data_mode)
    .def("spi_transfer",&DMFControlBoard::spi_transfer)
    .def("debug_buffer",&DMFControlBoard::debug_buffer)
    .def("number_of_channels",&DMFControlBoard::number_of_channels)
    .def("state_of_all_channels",&DMFControlBoard::state_of_all_channels)
    .def("state_of_channel",&DMFControlBoard::state_of_channel)
    .def("sampling_rate",&DMFControlBoard::sampling_rate)
    .def("adc_prescaler",&DMFControlBoard::adc_prescaler)
    .def("_aref",&DMFControlBoard::aref)
    .def("series_resistor_index",&DMFControlBoard::series_resistor_index)
    .def("_series_resistance",&DMFControlBoard::series_resistance)
    .def("_series_capacitance",&DMFControlBoard::series_capacitance)
    .def("_amplifier_gain",&DMFControlBoard::amplifier_gain)
    .def("_auto_adjust_amplifier_gain",
         &DMFControlBoard::auto_adjust_amplifier_gain)
    .def("waveform",&DMFControlBoard::waveform)
    .def("waveform_voltage",&DMFControlBoard::waveform_voltage)
    .def("waveform_frequency",&DMFControlBoard::waveform_frequency)
    .def("set_state_of_channel",&DMFControlBoard::set_state_of_channel)
    .def("set_state_of_all_channels",
         &DMFControlBoard::set_state_of_all_channels)
    .def("set_waveform",&DMFControlBoard::set_waveform)
    .def("set_waveform_voltage",&DMFControlBoard::set_waveform_voltage)
    .def("set_waveform_frequency",&DMFControlBoard::set_waveform_frequency)
    .def("set_sampling_rate",&DMFControlBoard::set_sampling_rate)
    .def("set_adc_prescaler",&DMFControlBoard::set_adc_prescaler)
    .def("set_series_resistor_index",
         &DMFControlBoard::set_series_resistor_index)
    .def("_set_series_resistance",&DMFControlBoard::set_series_resistance)
    .def("_set_series_capacitance",&DMFControlBoard::set_series_capacitance)
    .def("_set_amplifier_gain",&DMFControlBoard::set_amplifier_gain)
    .def("_set_auto_adjust_amplifier_gain",
         &DMFControlBoard::set_auto_adjust_amplifier_gain)
    .def("measure_impedance",&DMFControlBoard::measure_impedance)
    .def("measure_impedance_non_blocking",
         &DMFControlBoard::measure_impedance_non_blocking)
    .def("sweep_channels_non_blocking",
         &DMFControlBoard::sweep_channels_non_blocking)
    .def("sweep_channels",
         &DMFControlBoard::sweep_channels)
    .def("send_interrupt",&DMFControlBoard::send_interrupt)
    .def("get_measure_impedance_data",&DMFControlBoard::get_measure_impedance_data)
    .def("get_sweep_channels_data",&DMFControlBoard::get_sweep_channels_data)
    .def("waiting_for_reply",&DMFControlBoard::waiting_for_reply)
    .def("_reset_config_to_defaults",&DMFControlBoard::reset_config_to_defaults)
    .def("load_config",&DMFControlBoard::load_config)
    .def("flush",&DMFControlBoard::flush)
    .def("host_name",&DMFControlBoard::host_name)
    .def("host_manufacturer",&DMFControlBoard::host_manufacturer)
    .def("host_software_version",&DMFControlBoard::host_software_version)
    .def("host_url",&DMFControlBoard::host_url)
    .add_property("power_supply_pin", &DMFControlBoard::power_supply_pin)
    .add_property("watchdog_enabled", &DMFControlBoard::watchdog_enabled,
                  &DMFControlBoard::set_watchdog_enabled)
    .add_property("watchdog_state", &DMFControlBoard::watchdog_state,
                  &DMFControlBoard::set_watchdog_state)
    .add_property("atx_power_state",
                  &DMFControlBoard::atx_power_state,
                  &DMFControlBoard::set_atx_power_state)
  ;
DMFControlBoard_class.attr("PERSISTENT_PIN_MODE_ADDRESS") = \
    DMFControlBoard::PERSISTENT_PIN_MODE_ADDRESS;
DMFControlBoard_class.attr("PERSISTENT_PIN_STATE_ADDRESS") = \
    DMFControlBoard::PERSISTENT_PIN_STATE_ADDRESS;
DMFControlBoard_class.attr("PERSISTENT_BAUD_RATE_ADDRESS") = \
    DMFControlBoard::PERSISTENT_BAUD_RATE_ADDRESS;
DMFControlBoard_class.attr("PERSISTENT_SERIAL_NUMBER_ADDRESS") = \
    DMFControlBoard::PERSISTENT_SERIAL_NUMBER_ADDRESS;
DMFControlBoard_class.attr("PERSISTENT_CONFIG_SETTINGS") = \
    DMFControlBoard::PERSISTENT_CONFIG_SETTINGS;
DMFControlBoard_class.attr("MAX_PAYLOAD_LENGTH") = \
    DMFControlBoard::MAX_PAYLOAD_LENGTH;

/* # Remote return codes # */
DMFControlBoard_class.attr("RETURN_OK") = DMFControlBoard::RETURN_OK;
DMFControlBoard_class.attr("RETURN_GENERAL_ERROR") = \
    DMFControlBoard::RETURN_GENERAL_ERROR;
DMFControlBoard_class.attr("RETURN_UNKNOWN_COMMAND") = \
    DMFControlBoard::RETURN_UNKNOWN_COMMAND;
DMFControlBoard_class.attr("RETURN_TIMEOUT") = DMFControlBoard::RETURN_TIMEOUT;
DMFControlBoard_class.attr("RETURN_NOT_CONNECTED") = \
    DMFControlBoard::RETURN_NOT_CONNECTED;
DMFControlBoard_class.attr("RETURN_BAD_INDEX") = \
    DMFControlBoard::RETURN_BAD_INDEX;
DMFControlBoard_class.attr("RETURN_BAD_PACKET_SIZE") = \
    DMFControlBoard::RETURN_BAD_PACKET_SIZE;
DMFControlBoard_class.attr("RETURN_BAD_CRC") = DMFControlBoard::RETURN_BAD_CRC;
DMFControlBoard_class.attr("RETURN_BAD_VALUE") = \
    DMFControlBoard::RETURN_BAD_VALUE;
DMFControlBoard_class.attr("RETURN_MAX_CURRENT_EXCEEDED") = \
    DMFControlBoard::RETURN_MAX_CURRENT_EXCEEDED;

/* # Remote object command codes # */
DMFControlBoard_class.attr("CMD_ANALOG_READ") = RemoteObject::CMD_ANALOG_READ;
DMFControlBoard_class.attr("CMD_ANALOG_WRITE") = RemoteObject::CMD_ANALOG_WRITE;
DMFControlBoard_class.attr("CMD_DIGITAL_READ") = RemoteObject::CMD_DIGITAL_READ;
DMFControlBoard_class.attr("CMD_DIGITAL_WRITE") = RemoteObject::CMD_DIGITAL_WRITE;
DMFControlBoard_class.attr("CMD_GET_ADC_PRESCALER") = RemoteObject::CMD_GET_ADC_PRESCALER;
DMFControlBoard_class.attr("CMD_GET_AREF") = RemoteObject::CMD_GET_AREF;
DMFControlBoard_class.attr("CMD_GET_DEBUG_BUFFER") = RemoteObject::CMD_GET_DEBUG_BUFFER;
DMFControlBoard_class.attr("CMD_GET_DEVICE_NAME") = RemoteObject::CMD_GET_DEVICE_NAME;
DMFControlBoard_class.attr("CMD_GET_HARDWARE_VERSION") = RemoteObject::CMD_GET_HARDWARE_VERSION;
DMFControlBoard_class.attr("CMD_GET_MANUFACTURER") = RemoteObject::CMD_GET_MANUFACTURER;
DMFControlBoard_class.attr("CMD_GET_MCU_TYPE") = RemoteObject::CMD_GET_MCU_TYPE;
DMFControlBoard_class.attr("CMD_GET_PROTOCOL_NAME") = RemoteObject::CMD_GET_PROTOCOL_NAME;
DMFControlBoard_class.attr("CMD_GET_PROTOCOL_VERSION") = RemoteObject::CMD_GET_PROTOCOL_VERSION;
DMFControlBoard_class.attr("CMD_GET_SAMPLING_RATE") = RemoteObject::CMD_GET_SAMPLING_RATE;
DMFControlBoard_class.attr("CMD_GET_SOFTWARE_VERSION") = RemoteObject::CMD_GET_SOFTWARE_VERSION;
DMFControlBoard_class.attr("CMD_GET_URL") = RemoteObject::CMD_GET_URL;
DMFControlBoard_class.attr("CMD_I2C_READ") = RemoteObject::CMD_I2C_READ;
DMFControlBoard_class.attr("CMD_I2C_SCAN") = RemoteObject::CMD_I2C_SCAN;
DMFControlBoard_class.attr("CMD_I2C_WRITE") = RemoteObject::CMD_I2C_WRITE;
DMFControlBoard_class.attr("CMD_ONEWIRE_GET_ADDRESS") = RemoteObject::CMD_ONEWIRE_GET_ADDRESS;
DMFControlBoard_class.attr("CMD_ONEWIRE_READ") = RemoteObject::CMD_ONEWIRE_READ;
DMFControlBoard_class.attr("CMD_ONEWIRE_WRITE") = RemoteObject::CMD_ONEWIRE_WRITE;
DMFControlBoard_class.attr("CMD_PERSISTENT_READ") = RemoteObject::CMD_PERSISTENT_READ;
DMFControlBoard_class.attr("CMD_PERSISTENT_WRITE") = RemoteObject::CMD_PERSISTENT_WRITE;
DMFControlBoard_class.attr("CMD_SET_ADC_PRESCALER") = RemoteObject::CMD_SET_ADC_PRESCALER;
DMFControlBoard_class.attr("CMD_SET_PIN_MODE") = RemoteObject::CMD_SET_PIN_MODE;
DMFControlBoard_class.attr("CMD_SET_SAMPLING_RATE") = RemoteObject::CMD_SET_SAMPLING_RATE;
DMFControlBoard_class.attr("CMD_SPI_SET_BIT_ORDER") = RemoteObject::CMD_SPI_SET_BIT_ORDER;
DMFControlBoard_class.attr("CMD_SPI_SET_CLOCK_DIVIDER") = RemoteObject::CMD_SPI_SET_CLOCK_DIVIDER;
DMFControlBoard_class.attr("CMD_SPI_SET_DATA_MODE") = RemoteObject::CMD_SPI_SET_DATA_MODE;
DMFControlBoard_class.attr("CMD_SPI_TRANSFER") = RemoteObject::CMD_SPI_TRANSFER;

/* # Control board command codes # */
DMFControlBoard_class.attr("CMD_DEBUG_MESSAGE") = DMFControlBoard::CMD_DEBUG_MESSAGE;
DMFControlBoard_class.attr("CMD_DEBUG_ON") = DMFControlBoard::CMD_DEBUG_ON;
DMFControlBoard_class.attr("CMD_GET_AMPLIFIER_GAIN") = DMFControlBoard::CMD_GET_AMPLIFIER_GAIN;
DMFControlBoard_class.attr("CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN") = DMFControlBoard::CMD_GET_AUTO_ADJUST_AMPLIFIER_GAIN;
DMFControlBoard_class.attr("CMD_GET_NUMBER_OF_CHANNELS") = DMFControlBoard::CMD_GET_NUMBER_OF_CHANNELS;
DMFControlBoard_class.attr("CMD_GET_SERIES_CAPACITANCE") = DMFControlBoard::CMD_GET_SERIES_CAPACITANCE;
DMFControlBoard_class.attr("CMD_GET_SERIES_RESISTANCE") = DMFControlBoard::CMD_GET_SERIES_RESISTANCE;
DMFControlBoard_class.attr("CMD_GET_SERIES_RESISTOR_INDEX") = DMFControlBoard::CMD_GET_SERIES_RESISTOR_INDEX;
DMFControlBoard_class.attr("CMD_GET_STATE_OF_ALL_CHANNELS") = DMFControlBoard::CMD_GET_STATE_OF_ALL_CHANNELS;
DMFControlBoard_class.attr("CMD_GET_STATE_OF_CHANNEL") = DMFControlBoard::CMD_GET_STATE_OF_CHANNEL;
DMFControlBoard_class.attr("CMD_GET_WATCHDOG_ENABLED") = DMFControlBoard::CMD_GET_WATCHDOG_ENABLED;
DMFControlBoard_class.attr("CMD_GET_WATCHDOG_STATE") = DMFControlBoard::CMD_GET_WATCHDOG_STATE;
DMFControlBoard_class.attr("CMD_GET_WAVEFORM") = DMFControlBoard::CMD_GET_WAVEFORM;
DMFControlBoard_class.attr("CMD_GET_WAVEFORM_FREQUENCY") = DMFControlBoard::CMD_GET_WAVEFORM_FREQUENCY;
DMFControlBoard_class.attr("CMD_GET_WAVEFORM_VOLTAGE") = DMFControlBoard::CMD_GET_WAVEFORM_VOLTAGE;
DMFControlBoard_class.attr("CMD_LOAD_CONFIG") = DMFControlBoard::CMD_LOAD_CONFIG;
DMFControlBoard_class.attr("CMD_MEASURE_IMPEDANCE") = DMFControlBoard::CMD_MEASURE_IMPEDANCE;
DMFControlBoard_class.attr("CMD_SWEEP_CHANNELS") = DMFControlBoard::CMD_SWEEP_CHANNELS;
DMFControlBoard_class.attr("CMD_SET_AMPLIFIER_GAIN") = DMFControlBoard::CMD_SET_AMPLIFIER_GAIN;
DMFControlBoard_class.attr("CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN") = DMFControlBoard::CMD_SET_AUTO_ADJUST_AMPLIFIER_GAIN;
DMFControlBoard_class.attr("CMD_SET_SERIES_CAPACITANCE") = DMFControlBoard::CMD_SET_SERIES_CAPACITANCE;
DMFControlBoard_class.attr("CMD_SET_SERIES_RESISTANCE") = DMFControlBoard::CMD_SET_SERIES_RESISTANCE;
DMFControlBoard_class.attr("CMD_SET_SERIES_RESISTOR_INDEX") = DMFControlBoard::CMD_SET_SERIES_RESISTOR_INDEX;
DMFControlBoard_class.attr("CMD_SET_STATE_OF_ALL_CHANNELS") = DMFControlBoard::CMD_SET_STATE_OF_ALL_CHANNELS;
DMFControlBoard_class.attr("CMD_SET_STATE_OF_CHANNEL") = DMFControlBoard::CMD_SET_STATE_OF_CHANNEL;
DMFControlBoard_class.attr("CMD_SET_WATCHDOG_ENABLED") = DMFControlBoard::CMD_SET_WATCHDOG_ENABLED;
DMFControlBoard_class.attr("CMD_SET_WATCHDOG_STATE") = DMFControlBoard::CMD_SET_WATCHDOG_STATE;
DMFControlBoard_class.attr("CMD_SET_WAVEFORM") = DMFControlBoard::CMD_SET_WAVEFORM;
DMFControlBoard_class.attr("CMD_SET_WAVEFORM_FREQUENCY") = DMFControlBoard::CMD_SET_WAVEFORM_FREQUENCY;
DMFControlBoard_class.attr("CMD_SET_WAVEFORM_VOLTAGE") = DMFControlBoard::CMD_SET_WAVEFORM_VOLTAGE;
DMFControlBoard_class.attr("CMD_SYSTEM_RESET") = DMFControlBoard::CMD_SYSTEM_RESET;
#if ___ATX_POWER_CONTROL___
DMFControlBoard_class.attr("CMD_GET_POWER_SUPPLY_PIN") = DMFControlBoard::CMD_GET_POWER_SUPPLY_PIN;
DMFControlBoard_class.attr("CMD_GET_ATX_POWER_STATE") = DMFControlBoard::CMD_GET_ATX_POWER_STATE;
DMFControlBoard_class.attr("CMD_SET_ATX_POWER_STATE") = DMFControlBoard::CMD_SET_ATX_POWER_STATE;
#endif
}

#endif //!( defined(AVR) || defined(__SAM3X8E__) )
