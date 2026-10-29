#pragma once

// The fault-injection engine is built into every vehicle (it is listed in each
// vehicle's wscript) because the sensor drivers hook into it. It can be
// disabled for a whole build with -DAP_FAULT_INJECTION_ENABLED=0.
//
// Note: this flag must resolve identically in every translation unit of a
// build. ArduPilot shares a single "vehicle independent" object archive across
// vehicles, so it must not depend on the vehicle build type (see
// Tools/ardupilotwaf/ap_library.py: ap_library_check_headers).
#ifndef AP_FAULT_INJECTION_ENABLED
#define AP_FAULT_INJECTION_ENABLED 1
#endif
