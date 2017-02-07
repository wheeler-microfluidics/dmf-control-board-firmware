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

#include <Wire.h>
#include <SPI.h>
#if defined(AVR)
  #include "Memory.h"
  #include <EEPROM.h>
  #include <TimerThree.h>
  #include <AdvancedADC.h>
#elif defined(__SAM3X8E__)
  #include <DueFlashStorage.h>
  DueFlashStorage EEPROM;
#endif
#include <OneWire.h>
#include "RemoteObject.h"
#include "DMFControlBoard.h"
#include "FeedbackController.h"

DMFControlBoard dmf_control_board;
volatile unsigned int timer_count = 1;

void callback();

void setup() {
  /* Trigger timer to callback every 5 seconds _(5000000 microseconds)_. */
  Timer3.initialize(5000000L);
  Timer3.attachInterrupt(callback);
  dmf_control_board.begin();
#ifdef AVR
  Serial.print("ram="); Serial.println(ram_size(), DEC);
  Serial.print(".data="); Serial.println(data_size(), DEC);
  Serial.print(".bss="); Serial.println(bss_size(), DEC);
  Serial.print("heap="); Serial.println(heap_size(), DEC);
  Serial.print("stack="); Serial.println(stack_size(), DEC);
  Serial.print("free memory="); Serial.println(free_memory(), DEC);
#endif
}

void callback() { timer_count += 1; }

void loop() {
  dmf_control_board.listen();
  if (timer_count % 25 == 0) {
    /* Check the watchdog-state every 24 timer periods, _i.e., every 120
     * seconds_. */
    dmf_control_board.watchdog_timeout();
    timer_count = 1;
  }
}

