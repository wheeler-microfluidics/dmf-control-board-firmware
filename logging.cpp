/*
Copyright 2011 Ryan Fobel

This file is part of dmf_control_board.

Microdrop is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Microdrop is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Microdrop.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include "logging.h"

#include <iostream>
#include <stdio.h>
#include <cstdarg>
using namespace std;

uint8_t Logging::log_level_ = Logging::DEBUG;

void Logging::LogMessage(uint8_t log_level,
                         const char* message,
                         const char* class_name,
                         const char* function_name) {
  if(log_level<=log_level_) {
    PrintClassAndFunction(class_name,function_name);
    Print(message);
    Print("\r\n");
  }
}

/**
 * Log a message with formatting and a variable argument list.
 * \param log_level message priority
 * \param message the format of the message (same as printf)
 */
void Logging::LogMessageF(uint8_t log_level,
			  const char* message,
			  const char* class_name,
			  const char* function_name,
			  ... ) {
  if(log_level<=log_level_) {
    PrintClassAndFunction(class_name,function_name);
    va_list argList;
    va_start(argList, message);
    vprintf(message, argList);
    va_end(argList);
    Print("\r\n");
  }
}

void Logging::LogSeparator(char s) {
  char separator[81];
  memset(separator,s,80);
  separator[80] = 0;
  Print(separator);
  Print("\r\n");
}

void Logging::PrintClassAndFunction(const char* class_name,
                                  const char* function_name) {
  if(class_name) {
    Print(class_name);
  }
  if(class_name&&function_name) {
    Print("::");
  }
  if(function_name) {
    Print(function_name);
  }
  if(class_name||function_name) {
    Print(": ");
  }
}

void Logging::Print(const char* str) {
  cout<<str<<flush;
}
