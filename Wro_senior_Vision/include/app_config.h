#pragma once

#ifndef WRO_DEBUG_MODE
#define WRO_DEBUG_MODE 1
#endif

#ifndef WRO_COMPETITION_DEBUG
#define WRO_COMPETITION_DEBUG 0
#endif

#if WRO_DEBUG_MODE
constexpr bool DEBUG_MODE_ENABLED = true;
#else
constexpr bool DEBUG_MODE_ENABLED = false;
#endif
