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
#include <EEPROM.h>
#include <OneWire.h>
#include "Memory.h"
#include "RemoteObject.h"
#include "dmf_control_board.h"

DmfControlBoard dmf_control_board;

void setup() {
  dmf_control_board.begin();
  Serial.print("ram="); Serial.println(ram_size(), DEC);
  Serial.print(".data="); Serial.println(data_size(), DEC);
  Serial.print(".bss="); Serial.println(bss_size(), DEC);
  Serial.print("heap="); Serial.println(heap_size(), DEC);
  Serial.print("stack="); Serial.println(stack_size(), DEC);
  Serial.print("free memory="); Serial.println(free_memory(), DEC);
}

void loop() {
  dmf_control_board.Listen();
}
