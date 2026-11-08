# Firmware releases

Build from the intended Git tag using the pinned SDK and `tools/build-camera.sh`.
Run `python3 tools/host-tests.py` and validate the resulting image on the supported
board before describing it as hardware-tested.

Release assets should include the application, matching bootloader and partition
table, their flashing offsets, checksums, SDK/dependency revisions and instructions.
Record the source commit and distinguish built artifacts from hardware-tested
artifacts. Do not include local planning records, private deployment configuration,
raw camera captures or development-session logs.

The current partition layout provides wired flashing, without OTA rollback slots.
Use the generated flashing arguments for the matching image set. Large binaries
belong in GitHub Release assets rather than source Git.
