# NW Blue FPV bring-up

State of the first prototype (CubeNode v3 with carrier):

## Working

- USB CDC ACM
- microSD
- Internal mag (I2C3)
- External mag on GPS connector (I2C4)
- ICM45686 IMU (SPI3, rotation correct)
- ADC3 voltages (VSYS, 5v, 3v3)
- DroneCAN on CAN1
- LEDs
- RC input (UART4)

## Issues

- DPS368 baro intermittent (SPI3, sometimes after reflow of CS pin)
- TEL (UART7, flow control pins wrong)
- GPS (UART8, Tx ok, Rx not working)
- DShot outputs (Timer 3 not working)
- Buzzer (weak crackling with passive piezo)
- USART1 ESC telemetry (Rx not working)

## Other todos

- Vendor and product name
- USB Vid and pid
- Board id
- Current ADC not tested yet
- Internal mag needs rotation checked

## Console

Has to be enabled manually, disabled by default.

## Related fixes

- `src/drivers/barometer/dps310/`: accept product ID 0x11 (DPS368) in
  addition to 0x10 (DPS310) — register-compatible parts.

## Related links

- https://docs.cubepilot.org/user-guides/cubenode/pin-descriptions
