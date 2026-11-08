# Upstream contribution review

The camera milestone is maintained in the fclushio fork. Prepare focused PRs
from current upstream/main, independently of the complete streaming application.

- Review IMX219 formats and the pre-ISP initial-format setter as generic changes.
- Known baseline issue: Kconfig permits default indices 0–8 but imx219.c still
  rejects defaults >= 6. Runtime profile selection uses the setter and the tested
  default is 2. Correct and validate this separately from preserving the milestone.
- Keep Guition defaults separate from defaults for upstream-supported boards.
- Treat AE tuning as scene/board-specific until broader validation exists.
- The registry workflow strips all override_path entries. The IPA compatibility
  override must survive through another supported mechanism before publishing the
  RTSP example through the component registry.
- Ask the maintainer whether the complete RTSP example fits the project scope.
