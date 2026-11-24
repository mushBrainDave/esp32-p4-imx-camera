// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <stdio.h>
#include <pthread.h>
#include "esp_timer.h"
#include "rtsp_timer.h"
static esp_timer_create_args_t saved;
static bool fail_delete;
static int token;
esp_err_t __real_esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *h) { saved=*a; *h=&token; return ESP_OK; }
esp_err_t __real_esp_timer_delete(esp_timer_handle_t h) { assert(h==&token); return fail_delete ? ESP_ERR_INVALID_STATE : ESP_OK; }
esp_err_t __wrap_esp_timer_create(const esp_timer_create_args_t *,esp_timer_handle_t *);
esp_err_t __wrap_esp_timer_delete(esp_timer_handle_t);
static esp_timer_handle_t timer;
static pthread_mutex_t lock=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t changed=PTHREAD_COND_INITIALIZER;
static bool deleted, finish;
static bool alloc_scope;
void rtsp_alloc_scope_begin(void) { assert(!alloc_scope); alloc_scope=true; }
void rtsp_alloc_scope_end(void) { assert(alloc_scope); alloc_scope=false; }
static void callback(void *arg) {
 assert(alloc_scope);
 assert(arg==&token);
 assert(__wrap_esp_timer_delete(timer)==ESP_OK);
 pthread_mutex_lock(&lock); deleted=true; pthread_cond_broadcast(&changed);
 while(!finish) pthread_cond_wait(&changed,&lock);
 pthread_mutex_unlock(&lock);
 // Original callback still uses its context after deleting the timer.
 assert(arg==&token);
}
static void *invoke(void *unused) { (void)unused; saved.callback(saved.arg); return NULL; }
int main(void) {
 rtsp_timer_init();
 esp_timer_create_args_t a={.name="other_timer",.callback=callback,.arg=&token};
 assert(__wrap_esp_timer_create(&a,&timer)==ESP_OK && saved.callback==callback);
 assert(rtsp_timer_wait_idle(0));
 a.name="rtp_video_timer";
 assert(__wrap_esp_timer_create(&a,&timer)==ESP_OK && saved.callback!=callback);
 assert(!rtsp_timer_wait_idle(0));
 fail_delete=true; assert(__wrap_esp_timer_delete(timer)==ESP_ERR_INVALID_STATE);
 assert(!rtsp_timer_wait_idle(0)); fail_delete=false;
 pthread_t thread; assert(!pthread_create(&thread,NULL,invoke,NULL));
 pthread_mutex_lock(&lock); while(!deleted) pthread_cond_wait(&changed,&lock); pthread_mutex_unlock(&lock);
 assert(!rtsp_timer_wait_idle(10)); // delete alone is not completion
 pthread_mutex_lock(&lock); finish=true; pthread_cond_broadcast(&changed); pthread_mutex_unlock(&lock);
 pthread_join(thread,NULL); assert(rtsp_timer_wait_idle(0));
 assert(!alloc_scope);
 rtsp_timer_stats_t stats=rtsp_timer_stats_get();
 assert(stats.calls==1 && stats.total_us>=10000 && stats.max_us==stats.total_us);
 rtsp_timer_stats_reset();assert(rtsp_timer_stats_get().calls==0);
 puts("RTSP callback-tail barrier, failed deletion and unrelated timer passthrough passed");
}
