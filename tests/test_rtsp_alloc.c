// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include "rtsp_alloc.h"
static unsigned real_calls;
void *__real_media_lib_malloc(size_t size) { real_calls++; errno=EAGAIN; return malloc(size); }
void *__real_media_lib_calloc(size_t n,size_t size) { real_calls++; return calloc(n,size); }
void __real_media_lib_free(void *p) { real_calls++; free(p); }
void *__wrap_media_lib_malloc(size_t);
void *__wrap_media_lib_calloc(size_t,size_t);
void __wrap_media_lib_free(void *);
static void *other_task(void *arg) {
 (void)arg;
 void *p=__wrap_media_lib_malloc(900);assert(p);__wrap_media_lib_free(p);
 return NULL;
}
int main(void) {
 void *p=__wrap_media_lib_malloc(10);assert(p && errno==EAGAIN);__wrap_media_lib_free(p);
 assert(!rtsp_alloc_stats_get().alloc_calls);
 rtsp_alloc_scope_begin();
 p=__wrap_media_lib_calloc(2,40);assert(p);
 for(unsigned i=0;i<80;i++)assert(((unsigned char *)p)[i]==0);
 __wrap_media_lib_free(p);
 p=__wrap_media_lib_malloc(1216);assert(p && errno==EAGAIN);__wrap_media_lib_free(p);
 p=__wrap_media_lib_malloc(4096);assert(p);__wrap_media_lib_free(p);
 pthread_t thread;assert(!pthread_create(&thread,NULL,other_task,NULL));pthread_join(thread,NULL);
 rtsp_alloc_scope_end();
 rtsp_alloc_stats_t s=rtsp_alloc_stats_get();
 assert(s.alloc_calls==3 && s.free_calls==3 && s.alloc_bytes==80+1216+4096);
 assert(s.small_calls==1 && s.packet_calls==1 && s.large_calls==1 && !s.failures);
 assert(real_calls==10);
 rtsp_alloc_stats_reset();assert(!rtsp_alloc_stats_get().alloc_calls);
 puts("RTSP scoped allocator, task isolation, calloc zeroing, errno and reset passed");
}
