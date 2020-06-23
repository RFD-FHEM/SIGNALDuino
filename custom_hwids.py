Import("env")

board_config = env.BoardConfig()
# should be array of VID:PID pairs
# https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html#override-board-configuration
# https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html
board_config.update("build.hwids", [
  ["0x0483", "0x0003"],  # 1st pair
  ["0x0483", "0x0004"].  # 2nd pair, etc.
])