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

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "dmf_control_board.h"

using namespace boost::python;

const uint16_t RemoteObject::EEPROM_PIN_MODE_ADDRESS;
const uint16_t RemoteObject::EEPROM_PIN_STATE_ADDRESS;
const uint8_t RemoteObject::RETURN_OK;
const uint8_t RemoteObject::RETURN_GENERAL_ERROR;
const uint8_t RemoteObject::RETURN_UNKNOWN_COMMAND;
const uint8_t RemoteObject::RETURN_TIMEOUT;
const uint8_t RemoteObject::RETURN_NOT_CONNECTED;
const uint8_t RemoteObject::RETURN_BAD_INDEX;
const uint8_t RemoteObject::RETURN_BAD_PACKET_SIZE;
const uint8_t RemoteObject::RETURN_BAD_CRC;
const uint8_t DmfControlBoard::SINE;
const uint8_t DmfControlBoard::SQUARE;

BOOST_PYTHON_MODULE(dmf_control_board_base)
{
  scope().attr("INPUT") = 0;
  scope().attr("OUTPUT") = 1;
  scope().attr("HIGH") = 1;
  scope().attr("LOW") = 0;
  scope().attr("SINE") = DmfControlBoard::SINE;
  scope().attr("SQUARE") = DmfControlBoard::SQUARE;
  
  class_<std::vector<uint8_t> >("uint8_tVector")
    .def(vector_indexing_suite<std::vector<uint8_t> >())
  ;

  class_<std::vector<uint16_t> >("uint16_tVector")
    .def(vector_indexing_suite<std::vector<uint16_t> >())
  ;

  class_<std::vector<float> >("floatVector")
    .def(vector_indexing_suite<std::vector<float> >())
  ;

object DmfControlBoard_class
  = class_<DmfControlBoard,boost::noncopyable>("DmfControlBoard")
    .def("connect",&DmfControlBoard::Connect)
    .def("connected",&DmfControlBoard::connected)
    .def("return_code",&DmfControlBoard::return_code)
    .def("set_debug",&DmfControlBoard::set_debug)
    .def("protocol_name",&DmfControlBoard::protocol_name)
    .def("protocol_version",&DmfControlBoard::protocol_version)
    .def("name",&DmfControlBoard::name)
    .def("manufacturer",&DmfControlBoard::manufacturer)
    .def("software_version",&DmfControlBoard::software_version)
    .def("hardware_version",&DmfControlBoard::hardware_version)
    .def("url",&DmfControlBoard::url)
    .def("set_pin_mode",&DmfControlBoard::set_pin_mode)
    .def("digital_read",&DmfControlBoard::digital_read)
    .def("digital_write",&DmfControlBoard::digital_write)
    .def("analog_read",&DmfControlBoard::analog_read)
    .def("analog_write",&DmfControlBoard::analog_write)
    .def("eeprom_read",&DmfControlBoard::eeprom_read)
    .def("eeprom_write",&DmfControlBoard::eeprom_write)
    .def("onewire_address",&DmfControlBoard::onewire_address)
    .def("onewire_read",&DmfControlBoard::onewire_read)
    .def("onewire_write",&DmfControlBoard::onewire_write)
    .def("i2c_read",&DmfControlBoard::i2c_read)
    .def("i2c_write",&DmfControlBoard::i2c_write)
    .def("spi_set_bit_order",&DmfControlBoard::spi_set_bit_order)
    .def("spi_set_clock_divider",&DmfControlBoard::spi_set_clock_divider)
    .def("spi_set_data_mode",&DmfControlBoard::spi_set_data_mode)
    .def("spi_transfer",&DmfControlBoard::spi_transfer)
    .def("number_of_channels",&DmfControlBoard::number_of_channels)
    .def("state_of_all_channels",&DmfControlBoard::state_of_all_channels)
    .def("state_of_channel",&DmfControlBoard::state_of_channel)
    .def("sampling_rate",&DmfControlBoard::sampling_rate)
    .def("series_resistor",&DmfControlBoard::series_resistor)
    .def("waveform",&DmfControlBoard::waveform)
    .def("set_state_of_channel",&DmfControlBoard::set_state_of_channel)
    .def("set_state_of_all_channels",&DmfControlBoard::set_state_of_all_channels)
    .def("set_waveform",&DmfControlBoard::set_waveform)
    .def("set_waveform_voltage",&DmfControlBoard::set_waveform_voltage)
    .def("set_waveform_frequency",&DmfControlBoard::set_waveform_frequency)
    .def("set_sampling_rate",&DmfControlBoard::set_sampling_rate)
    .def("set_series_resistor",&DmfControlBoard::set_series_resistor)
    .def("sample_voltage",&DmfControlBoard::SampleVoltage)
    .def("set_experiment_log_file",&DmfControlBoard::SetExperimentLogFile)
    .def("log_experiment",&DmfControlBoard::LogExperiment)
    .def("measure_impedance",&DmfControlBoard::MeasureImpedance)
    .def("flush",&DmfControlBoard::flush)
    .def("host_name",&DmfControlBoard::host_name)
    .def("host_hardware_version",&DmfControlBoard::host_hardware_version)
    .def("host_software_version",&DmfControlBoard::host_software_version)
  ;
DmfControlBoard_class.attr("EEPROM_PIN_MODE_ADDRESS") = DmfControlBoard::EEPROM_PIN_MODE_ADDRESS;
DmfControlBoard_class.attr("EEPROM_PIN_STATE_ADDRESS") = DmfControlBoard::EEPROM_PIN_STATE_ADDRESS;
DmfControlBoard_class.attr("RETURN_OK") = DmfControlBoard::RETURN_OK;
DmfControlBoard_class.attr("RETURN_GENERAL_ERROR") = DmfControlBoard::RETURN_GENERAL_ERROR;
DmfControlBoard_class.attr("RETURN_UNKNOWN_COMMAND") = DmfControlBoard::RETURN_UNKNOWN_COMMAND;
DmfControlBoard_class.attr("RETURN_TIMEOUT") = DmfControlBoard::RETURN_TIMEOUT;
DmfControlBoard_class.attr("RETURN_NOT_CONNECTED") = DmfControlBoard::RETURN_NOT_CONNECTED;
DmfControlBoard_class.attr("RETURN_BAD_INDEX") = DmfControlBoard::RETURN_BAD_INDEX;
DmfControlBoard_class.attr("RETURN_BAD_PACKET_SIZE") = DmfControlBoard::RETURN_BAD_PACKET_SIZE;
DmfControlBoard_class.attr("RETURN_BAD_CRC") = DmfControlBoard::RETURN_BAD_CRC;
}
