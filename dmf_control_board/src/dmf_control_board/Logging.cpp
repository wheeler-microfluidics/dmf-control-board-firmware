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

#include <string.h>
#include "Logging.h"

#include <iostream>
#include <stdio.h>
#include <cstdarg>
using namespace std;

uint8_t Logging::log_level_ = Logging::DEBUG;

void Logging::log_message(uint8_t log_level,
                          const char* message,
                          const char* class_name,
                          const char* function_name) {
  if(log_level <= log_level_) {
    print_class_and_function(class_name, function_name);
    print(message);
    print("\r\n");
  }
}

/**
 * Log a message with formatting and a variable argument list.
 * \param log_level message priority
 * \param message the format of the message (same as printf)
 */
void Logging::log_message_f(uint8_t log_level,
                            const char* message,
                            const char* class_name,
                            const char* function_name,
                            ... ) {
  if(log_level <= log_level_) {
    print_class_and_function(class_name, function_name);
    va_list argList;
    va_start(argList, message);
    vprintf(message, argList);
    va_end(argList);
    print("\r\n");
  }
}

void Logging::log_separator(char s) {
  char separator[81];
  memset(separator,s,80);
  separator[80] = 0;
  print(separator);
  print("\r\n");
}

void Logging::print_class_and_function(const char* class_name,
                                       const char* function_name) {
  if(class_name) {
    print(class_name);
  }
  if(class_name && function_name) {
    print("::");
  }
  if(function_name) {
    print(function_name);
  }
  if(class_name || function_name) {
    print(": ");
  }
}

void Logging::print(const char* str) {
  cout << str << flush;
}

#endif // !( defined(AVR) || defined(__SAM3X8E__) )
