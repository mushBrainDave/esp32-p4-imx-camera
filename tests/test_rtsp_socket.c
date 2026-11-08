// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include "rtsp_socket.h"
#include "h264_sdp.h"
static h264_params_t cached;
bool stream_h264_params(h264_params_t *params) {
    if (!h264_params_ready(&cached)) return false;
    *params=cached;return true;
}
static int retire_count;
void stream_retire_connection(void) { retire_count++; }
ssize_t __real_media_lib_socket_send(int s, const void *p, size_t n, int flags) { return send(s,p,n,flags); }
ssize_t __real_media_lib_socket_write(int s, const void *p, size_t n) { return write(s,p,n); }
int __real_media_lib_socket_accept(int s, struct sockaddr *a, socklen_t *n) { return accept(s,a,n); }
int __real_media_lib_socket_close(int s) { return close(s); }
int __real_media_lib_socket_listen(int s,int backlog) { return listen(s,backlog); }
int __wrap_media_lib_socket_accept(int s, struct sockaddr *a, socklen_t *n);
int __wrap_media_lib_socket_close(int s);
int __wrap_media_lib_socket_listen(int s,int backlog);
ssize_t __wrap_media_lib_socket_send(int s,const void *p,size_t n,int flags);
static int connect_client(int listener, const struct sockaddr_un *address, int *peer)
{
    *peer = socket(AF_UNIX, SOCK_STREAM, 0); assert(*peer >= 0);
    assert(connect(*peer,(const struct sockaddr *)address,sizeof(*address)) == 0);
    return __wrap_media_lib_socket_accept(listener,NULL,NULL);
}
int main(void)
{
    signal(SIGPIPE,SIG_IGN);
    rtsp_socket_init();
    struct sockaddr_un address = {.sun_family=AF_UNIX};
    snprintf(address.sun_path,sizeof(address.sun_path),"/tmp/astra-rtsp-test-%ld",(long)getpid());
    int listener=socket(AF_UNIX,SOCK_STREAM,0);assert(listener>=0);
    assert(bind(listener,(struct sockaddr *)&address,sizeof(address))==0 && listen(listener,8)==0);
    int peers[5],fds[4];
    fds[0]=connect_client(listener,&address,&peers[0]);assert(fds[0]>=0);
    const char response[]="RTSP/1.0 200 OK\r\nContent-Length: 12 \r\n\r\n";
    assert(__wrap_media_lib_socket_send(fds[0],response,sizeof(response)-1,0)==sizeof(response)-1);
    char received[sizeof(response)]={0};
    assert(read(peers[0],received,sizeof(received))==sizeof(response)-1);
    assert(strstr(received,"Content-Length: 012\r\n"));
    rtsp_socket_stats_t metrics=rtsp_socket_stats_get();
    assert(metrics.calls==1 && metrics.bytes==sizeof(response)-1 && !metrics.failures);
    assert(__wrap_media_lib_socket_close(fds[0])==0 && retire_count==1);
    assert(fcntl(fds[0],F_GETFD)>=0); // shutdown retains the descriptor reservation
    assert(__wrap_media_lib_socket_close(fds[0])==0 && retire_count==1);
    int http[2];assert(socketpair(AF_UNIX,SOCK_STREAM,0,http)==0);
    assert(http[0]!=fds[0] && http[1]!=fds[0]);
    const char rtp[]="$\0stale RTP";
    assert(__wrap_media_lib_socket_send(fds[0],rtp,sizeof(rtp),0)<0);
    metrics=rtsp_socket_stats_get();
    assert(metrics.calls==2 && metrics.failures==1 && metrics.last_errno==EPIPE);
    assert(errno==EPIPE); // instrumentation must preserve socket error behavior
    struct pollfd p={.fd=http[1],.events=POLLIN};assert(poll(&p,1,0)==0);
    // A new session cannot race the previous worker's teardown.
    assert(connect_client(listener,&address,&peers[1])<0);close(peers[1]);
    rtsp_socket_begin_stop();
    assert(__wrap_media_lib_socket_close(fds[0])==0 && retire_count==1);
    rtsp_socket_release_clients();
    assert(fcntl(fds[0],F_GETFD)<0 && errno==EBADF);close(peers[0]);
    // After the stop barrier, a new session is accepted and can stop cleanly.
    fds[0]=connect_client(listener,&address,&peers[0]);assert(fds[0]>=0);
    assert(connect_client(listener,&address,&peers[1])<0);close(peers[1]);
    rtsp_socket_begin_stop();
    char byte;
    assert(read(peers[0],&byte,1)==0); // force EOF without releasing the fd
    assert(!rtsp_socket_wait_retired(0)); // control worker has not closed yet
    assert(__wrap_media_lib_socket_close(fds[0])==0 && retire_count==1);
    assert(rtsp_socket_wait_retired(0));
    rtsp_socket_release_clients();close(peers[0]);
    // SDP may expand on the wire; the library still receives its input length.
    fds[0]=connect_client(listener,&address,&peers[0]);assert(fds[0]>=0);
    const uint8_t au[]={0,0,1,0x67,0x64,0,0x28,0xac,0xd9,0x40,0,0,1,0x68,0xee,0x3c,0x80};
    assert(h264_params_ingest(&cached,au,sizeof(au)));
    const char body[]="v=0\r\na=fmtp:96 packetization-mode=1;\r\n";
    char reply[256];int reply_len=snprintf(reply,sizeof(reply),
        "RTSP/1.0 200 OK\r\nCSeq: 2\r\nContent-Type: application/sdp\r\nContent-Length: %zu \r\n\r\n%s",strlen(body),body);
    uint64_t prior_bytes=rtsp_socket_stats_get().bytes;
    assert(__wrap_media_lib_socket_send(fds[0],reply,reply_len,0)==reply_len);
    char expanded[1024]={0};ssize_t wire_len=read(peers[0],expanded,sizeof(expanded)-1);
    assert(wire_len>reply_len && strstr(expanded,"sprop-parameter-sets=Z2QAKKzZQA==,aO48gA==;"));
    char *payload=strstr(expanded,"\r\n\r\n")+4;
    assert(strtoul(strstr(expanded,"Content-Length: ")+16,NULL,10)==strlen(payload));
    assert(rtsp_socket_stats_get().bytes-prior_bytes==(uint64_t)wire_len);
    rtsp_socket_begin_stop();assert(__wrap_media_lib_socket_close(fds[0])==0);
    rtsp_socket_release_clients();close(peers[0]);
    close(http[0]);close(http[1]);
    assert(__wrap_media_lib_socket_close(listener)==0);unlink(address.sun_path);
    // Server stop and TCP transport destroy close the same listener twice.
    // HTTP must not inherit that descriptor between those two calls.
    listener=socket(AF_UNIX,SOCK_STREAM,0);assert(listener>=0);
    assert(bind(listener,(struct sockaddr *)&address,sizeof(address))==0);
    assert(__wrap_media_lib_socket_listen(listener,8)==0);
    rtsp_socket_begin_stop();
    assert(__wrap_media_lib_socket_close(listener)==0);
    assert(fcntl(listener,F_GETFD)>=0);
    assert(socketpair(AF_UNIX,SOCK_STREAM,0,http)==0);
    assert(http[0]!=listener && http[1]!=listener);
    assert(__wrap_media_lib_socket_close(listener)==0);
    assert(write(http[0],"HTTP",4)==4);
    char message[4];assert(read(http[1],message,4)==4 && !memcmp(message,"HTTP",4));
    assert(rtsp_socket_wait_retired(0)); // listener is released AFTER server stop
    rtsp_socket_release_clients();
    assert(fcntl(listener,F_GETFD)<0 && errno==EBADF);
    int reused=socket(AF_UNIX,SOCK_STREAM,0);assert(reused==listener);
    assert(write(http[0],"safe",4)==4 && read(http[1],message,4)==4);
    close(reused);close(http[0]);close(http[1]);unlink(address.sun_path);
    assert(rtsp_socket_stats_get().failures==1); // recovery retains diagnostic evidence
    rtsp_socket_stats_reset();assert(rtsp_socket_stats_get().calls==0);
    puts("RTSP client/listener pinning, duplicate-close isolation, stop barrier and single-session handoff passed");
}
