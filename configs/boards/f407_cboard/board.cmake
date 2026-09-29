# Build identity only; hardware bindings live in board.json.
set(PNX_BOARD_FAMILY "stm32f4")
set(PNX_TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(PNX_BOARD_OPENOCD_TARGET "target/stm32f4x.cfg")
