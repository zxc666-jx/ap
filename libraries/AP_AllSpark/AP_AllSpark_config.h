#pragma once

// The AllSpark binary protocol is built into every vehicle (it is listed in
// each vehicle's wscript) so that AP_Vehicle can own the object. Only ArduPlane
// implements the vehicle-side hooks, so it is a no-op elsewhere.
// It can be disabled for a whole build with -DAP_ALLSPARK_ENABLED=0.
//
// Note: this flag must resolve identically in every translation unit of a
// build. ArduPilot shares a single "vehicle independent" object archive across
// vehicles, so it must not depend on the vehicle build type (see
// Tools/ardupilotwaf/ap_library.py: ap_library_check_headers).
#ifndef AP_ALLSPARK_ENABLED
#define AP_ALLSPARK_ENABLED 1
#endif
