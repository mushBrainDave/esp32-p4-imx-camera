"""Run the existing transport/age regressions against bundled source."""
import os
from pathlib import Path
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[1]
tests = repo / 'tests'
main = repo / 'components/esp_cam_sensor_imx/examples/imx219_rtsp_eth/main'
env = {**os.environ, 'ASAN_OPTIONS': 'detect_leaks=0'}
with tempfile.TemporaryDirectory(prefix='imx219-release-tests-') as temporary:
    for case in ['stream_limits', 'rtsp_wire', 'h264_sdp', 'rtsp_socket',
                 'rtsp_timer', 'rtsp_alloc', 'fullview_scale', 'image_request']:
        sources = [tests / ('test_' + case + '.c')]
        if case != 'stream_limits':
            sources.append(main / (case + '.c'))
        if case == 'rtsp_socket':
            sources.extend([main / 'rtsp_wire.c', main / 'h264_sdp.c'])
        executable = Path(temporary) / case
        subprocess.run(['cc', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-pthread',
                        '-I', str(tests / 'rtsp_socket_host'), '-I', str(main),
                        *map(str, sources), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, env=env)
        print(case + ' PASS', flush=True)
