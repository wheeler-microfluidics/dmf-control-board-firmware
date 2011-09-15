#ifndef _MEMORY_H
#define _MEMORY_H

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h> //RAMEND, SP

/*
 * Based on code from several sources:
 * http://www.arduino.cc/playground/Code/AvailableMemory
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1213583720
 * http://forum.pololu.com/viewtopic.php?f=10&t=989&view=unread#p4218
 *
 * Reference figure of AVR memory areas .data, .bss, heap (all growing upwards),
 * then stack growing downward:
 * http://www.nongnu.org/avr-libc/user-manual/malloc.html
 */

extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

inline int16_t ram_size() {
  return (int16_t) RAMEND - (int16_t) &__data_start;
}
inline int16_t data_size() {
  return (int16_t) &__data_end - (int16_t) &__data_start;
}
inline int16_t bss_size() {
  return (int16_t) &__bss_end - (int16_t) &__bss_start;
}
inline int16_t heap_size() {
  if((int16_t)__brkval==0) { // no heap
    return 0;
  }
  return (int16_t) __brkval - (int16_t) &__heap_start;
}
inline int16_t stack_size() {
  return (int16_t) RAMEND - (int16_t) SP;
}
inline int16_t free_memory() {
  if((int16_t)__brkval==0) { // no heap
    return (int16_t) SP - (int16_t) &__bss_end;
  }
  return (int16_t) SP - (int16_t) __brkval;
}
/*inline int16_t free_memory2()
{
  int16_t free_memory;

  if((int16_t)__brkval == 0)
     free_memory = ((int16_t)&free_memory) - ((int16_t)&__bss_end);
  else
    free_memory = ((int16_t)&free_memory) - ((int16_t)__brkval);

  return free_memory;
}
*/
#endif
