# Build identity only; hardware bindings live in board.json.
set(PNX_BOARD_FAMILY "stm32h7")
set(PNX_TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")
set(PNX_BOARD_OPENOCD_TARGET "target/stm32h7x.cfg")
