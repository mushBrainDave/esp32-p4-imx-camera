// SPDX-License-Identifier: Apache-2.0
#pragma once
/* Low-rate quality modes need a complete frame-period allowance in addition
 * to scheduling jitter. At5fps,250ms alone leaves only50ms beyond one tick.
 * Capacity remains3 AUs; faster modes keep their existing250ms age ceiling. */
static inline unsigned stream_queue_age_limit_ms(unsigned fps)
{
    return fps > 0 && fps <= 5 ? 250U + 1000U / fps : 250U;
}
