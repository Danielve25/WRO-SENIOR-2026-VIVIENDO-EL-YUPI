#pragma once

#ifndef WRO_DEBUG_MODE
#define WRO_DEBUG_MODE 1
#endif

#if WRO_DEBUG_MODE
constexpr bool DEBUG_MODE_ENABLED = true;
#else
constexpr bool DEBUG_MODE_ENABLED = false;
#endif
