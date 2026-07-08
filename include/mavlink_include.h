#pragma once

#if defined(MAVLINK_DEVELOPMENT) || __has_include(<development/mavlink.h>)
#include <development/mavlink.h>
#else
#include <common/mavlink.h>
#endif
