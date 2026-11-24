# Pinned multimedia compatibility dependency

Snapshot IPA source: Espressif esp-video-components commit
`1f7d39e37b69be18ce977dbab83d1717abcebbe4`.

Fetch from the repository root:

```bash
git clone --depth 1 --revision=1f7d39e37b69be18ce977dbab83d1717abcebbe4 \
  https://github.com/espressif/esp-video-components.git third_party/esp-video-components
```

The snapshot manifest overrides `espressif/esp_ipa` with this checkout's
`esp_ipa` directory. The published 2.4.0 library crashes on our P4 v1.3 under
IDF 6.x at opcode `0x20fac7b3` in `create_ipa_array`. This upstream commit
provides/selects pre-v3 compatible archives. No managed-component edits are
required. Keep upstream license files and the pinned source revision.

Reference: https://github.com/espressif/esp-video-components/issues/99
