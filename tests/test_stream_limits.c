// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <stdio.h>
#include "stream_limits.h"
int main(void)
{
    assert(stream_queue_age_limit_ms(0)==250);
    assert(stream_queue_age_limit_ms(1)==1250);
    assert(stream_queue_age_limit_ms(3)==583);
    assert(stream_queue_age_limit_ms(5)==450);
    assert(stream_queue_age_limit_ms(15)==250);
    assert(stream_queue_age_limit_ms(30)==250);
    // Observed5fps startup ages cross the old ceiling but stay within one
    // nominal frame plus the existing jitter budget. Excess age still expires.
    assert(254399U > stream_queue_age_limit_ms(15)*1000);
    assert(254399U < stream_queue_age_limit_ms(5)*1000);
    assert(254566U < stream_queue_age_limit_ms(5)*1000);
    assert(450001U > stream_queue_age_limit_ms(5)*1000);
    puts("Low-rate frame budget, observed startup regressions and fast-mode limits passed");
}
