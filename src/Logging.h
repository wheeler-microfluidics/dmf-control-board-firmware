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

#ifndef _LOGGING_H
#define _LOGGING_H

#include <stdint.h>


class Logging {
public:
  static const uint8_t DEBUG=9;
  static const uint8_t INFO=6;
  static const uint8_t WARN=3;
  static const uint8_t ERROR=0;

  // display any log messages<=level (default is 9)
  static void set_log_level(uint8_t level) { log_level_ = level; }
  static void log_debug(const char* message,
                        const char* class_name = 0,
                        const char* function_name = 0) {
                        log_message(Logging::DEBUG,
                                    message,class_name,function_name); }
  static void log_info(const char* message,
                       const char* class_name = 0,
                       const char* function_name = 0) {
                       log_message(Logging::INFO,
                                   message,class_name,function_name); }
  static void log_warning(const char* message,
                          const char* class_name = 0,
                          const char* function_name = 0) {
                          log_message(Logging::WARN,
                                      message,class_name,function_name); }
  static void log_error(const char* message,
                        const char* class_name = 0,
                        const char* function_name = 0) {
                        log_message(Logging::ERROR,
                                    message,class_name,function_name); }
  static void log_message(uint8_t log_level,
                         const char* message,
                         const char* class_name = 0,
                         const char* function_name = 0);
  // log messages with formatting and a variable argument list
  static void log_message_f(uint8_t log_level,
                            const char* message,
                            const char* class_name = 0,
                            const char* function_name = 0,
                            ... );
  static void log_separator(char s='=');

private:
  static void print_class_and_function(const char* class_name,
                                       const char* function_name);
  static void print(const char* str);
  static uint8_t log_level_;
};

#endif // _LOGGING_H
