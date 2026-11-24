// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "h264_sdp.h"
int main(void)
{
    const uint8_t au[] = {0,0,0,1,0x67,0x64,0,0x28,0xac,0xd9,0x40,
                         0,0,1,0x68,0xee,0x3c,0x80,0,0,0,1,0x65,1,2,3};
    h264_params_t params = {0};
    assert(h264_params_ingest(&params, au, sizeof(au)));
    assert(params.sps_len == 7 && params.pps_len == 4);
    assert(!memcmp(params.sps, au+4, 7) && !memcmp(params.pps, au+14, 4));
    h264_params_t split = {0};
    assert(!h264_params_ingest(&split, au, 11));
    assert(h264_params_ingest(&split, au+11, sizeof(au)-11));
    assert(!memcmp(&split, &params, sizeof(params)));
    const char body[] = "v=0\r\nm=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\n"
                        "a=fmtp:96 level-asymmetry-allowed=1;packetization-mode=1;\r\n";
    char input[1024];
    int length = snprintf(input,sizeof(input),"RTSP/1.0 200 OK\r\nCSeq: 2\r\n"
        "Content-Type: application/sdp\r\nContent-Length: %zu \r\n\r\n%s",strlen(body),body);
    uint8_t output[2048];size_t written=0;
    assert(h264_sdp_response((uint8_t *)input,length,&params,output,sizeof(output),&written));
    output[written]=0;
    assert(strstr((char *)output,"CSeq: 2\r\n"));
    assert(strstr((char *)output,"profile-level-id=640028;sprop-parameter-sets=Z2QAKKzZQA==,aO48gA==;\r\n"));
    char *payload=strstr((char *)output,"\r\n\r\n")+4;
    char *size=strstr((char *)output,"Content-Length: ")+16;
    assert(strtoul(size,NULL,10)==strlen(payload) && strlen((char *)output)==written);
    size_t duplicate=0;
    assert(!h264_sdp_response(output,written,&params,(uint8_t *)input,sizeof(input),&duplicate));
    // Every truncated input and undersized output must fail without overrun.
    length=snprintf(input,sizeof(input),"RTSP/1.0 200 OK\r\nContent-Type: application/sdp\r\n"
        "Content-Length: %zu\r\n\r\n%s",strlen(body),body);
    for(int n=0;n<length;n++)assert(!h264_sdp_response((uint8_t *)input,n,&params,output,sizeof(output),&duplicate));
    assert(h264_sdp_response((uint8_t *)input,length,&params,output,sizeof(output),&written));
    for(size_t n=0;n<written;n++)assert(!h264_sdp_response((uint8_t *)input,length,&params,output,n,&duplicate));
    memset(&split,0,sizeof(split));
    assert(!h264_sdp_response((uint8_t *)input,length,&split,output,sizeof(output),&duplicate));
    uint8_t oversized[300];memset(oversized,0x55,sizeof(oversized));
    oversized[0]=0;oversized[1]=0;oversized[2]=1;oversized[3]=0x67;
    assert(!h264_params_ingest(&split,oversized,sizeof(oversized)) && !split.sps_len);
    const uint8_t slice[]={0,0,1,0x65,0,0,0,1,0x67,1,2,3};
    assert(!h264_params_ingest(&split,slice,sizeof(slice))); // no scanning beyond the first slice
    puts("H264 Annex-B cache, SDP/base64/content length, truncation and capacity checks passed");
}
