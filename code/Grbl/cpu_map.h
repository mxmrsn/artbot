
#ifndef cpu_map_h
#define cpu_map_h


#ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.
  #include "cpu_map/cpu_map_atmega328p.h"
#endif

#ifdef CPU_MAP_ATMEGA2560 // (Arduino Mega 2560) Working @EliteEng
  #include "cpu_map/cpu_map_atmega2560.h"
#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
