# Hardware Interface Notes

These notes describe the current wall-hack v1.0 hardware and the lessons from
bench work. Keep pin assignments and electrical assumptions close to the code
that uses them. Clearly label measurements, assumptions, and unverified
behavior.

## Board and display

- The target is the original ESP32 on the LCDWiki 4.0-inch ESP32-32E board,
  model E32R40T. Its onboard display is an ST7796S at 320 x 480 pixels with an
  XPT2046 resistive touch controller.
- The current firmware uses portrait orientation (`setRotation(0)`). Keep the
  display geometry, touch calibration, hit testing, and UI coordinates in the
  same orientation.
- The TFT_eSPI setup used for this board is `Setup_E32R40T.h`, selected through
  `User_Setup_Select.h`. It configures ST7796, width 320, height 480, MISO 12,
  MOSI 13, SCLK 14, CS 15, DC 2, RST -1 (reset is shared with ESP32 EN), touch
  CS 33, and 27 MHz display SPI. Library updates can overwrite the setup
  selector; check it when the display or touch suddenly stops working.
- The onboard backlight is GPIO27 and is active high. Turn it on after display
  initialization and off during shutdown.
- GPIO25 is also the board's exposed I2C SCL signal. This firmware assigns it
  to PowerBoost EN, so do not use that I2C signal concurrently.
- The onboard touch controller uses CS GPIO33 and IRQ GPIO36. The current
  TFT_eSPI setup uses the touch controller over SPI with CS 33.
- The board's SD SPI signals are CS 5, SCLK 18, MOSI 23, and MISO 19. GPIO19
  and GPIO23 are reassigned to radar UART while the LD2450 is connected, so do
  not use the SD card at the same time.
- Select **ESP32 Dev Module**, with 4 MB flash and PSRAM disabled, in Arduino
  IDE. USB serial uses the board's UART0 pins; leave GPIO1/GPIO3 available for
  programming and diagnostics.

## LD2450 UART and sensor behavior

| LD2450 signal | E32R40T signal |
| --- | --- |
| TX (radar output) | GPIO19, UART2 RX |
| RX (radar input) | GPIO23, UART2 TX |
| VCC | PowerBoost 500C 5 V output |
| GND | Common ground |

- Cross TX to RX and RX to TX. Firmware uses UART2, 256000 baud, `SERIAL_8N1`.
- The user has reported successful LD2450 serial operation with this wiring.
- The LD2450 reports up to three moving target slots with X/Y coordinates and
  velocity. The UI derives range and angle from X/Y. Do not describe these
  tracks as reliable stationary-presence or biometric measurements.
- Keep GPIO19/GPIO23 free of SD or other SPI activity while they serve as
  UART2. Stop UART2 before powering down the radar.

## Power, battery sensing, and shutdown

- The reported setup uses a single-cell LiPo shared by the E32R40T battery
  input and a PowerBoost 500C. Confirm the actual cell connector polarity and
  power-path wiring on the hardware before changing it.
- PowerBoost EN is GPIO25: HIGH enables the boost output; LOW disables it.
  GPIO25 is RTC-capable and the firmware holds it low during deep sleep. The
  board RESET button is the intended restart; no touch or GPIO wake source is
  configured.
- Before pulling EN low, end `RadarSerial` and make both radar UART pins,
  GPIO19 and GPIO23, high impedance. UART signals can feed power into an
  unpowered radar and leave voltage on the boost output.
- In bench testing, EN LOW produced about 0 V with the radar disconnected but
  about 1.2 V with it connected. The user observed that each UART data line
  contributed roughly half that voltage and added a hard switch to the
  battery. Preserve this observation when changing shutdown or power wiring;
  measure the assembled device rather than assuming EN LOW isolates every
  rail.
- The firmware shutdown sequence also sends ST7796S display-off (`0x28`) and
  sleep-in (`0x10`) commands, turns off GPIO27 backlight, then enters ESP32
  deep sleep. Actual sleep current and complete power isolation have not been
  established by measurement.
- Battery ADC is GPIO34. The code assumes a 2.0x divider and calibration 1.0;
  neither is confirmed. The percentage is a rough voltage-based estimate and
  maps an estimated 3.3-4.2 V cell voltage to 0-100%. It is not a measured
  state of charge and does not measure charge current, charge completion, or
  cell temperature.
- Do not connect the PowerBoost charger input while the LCDWiki onboard
  charger is charging the same cell unless the full power path has been
  validated. Confirm charger behavior, current, and cell temperature before
  relying on unattended charging.

## Touchscreen

- The firmware calibrates in portrait orientation and stores its calibration
  in Preferences namespace `radar-p`, key `touch`. Keep the profile distinct
  from landscape calibration values; calibration data is orientation-specific.
- On first boot with no portrait calibration, follow the TFT_eSPI calibration
  prompts. Send `c` over USB serial to recalibrate. The firmware prints
  touch-down coordinates over USB serial to help debug hit-box alignment.
- The status bar is only 15 pixels high, so the menu hit target is wider than
  its visible button. Preserve a forgiving hit area if changing the UI scale.

## Display behavior

- Keep the dark background and green interface palette. Target markers and
  tracking details share one palette: target 1 green, target 2 red, target 3
  blue.
- Draw the static grid once and update only changed markers and text to avoid
  visible full-screen flashing. When clearing a moved marker, redraw the grid
  pixels and any other markers that intersect the cleared region.
- Do not transfer `invertDisplay(true)` from other ST7796S projects without
  checking this board. Inversion behavior is panel and library configuration
  dependent.

## Build and verification boundaries

- The firmware depends on Arduino ESP32 core APIs, TFT_eSPI, Preferences, and
  ESP32 RTC GPIO/deep-sleep APIs. It also depends on the local TFT_eSPI
  `Setup_E32R40T.h`; that file is not stored in this repository.
- A successful compile does not verify touch hit-box alignment, target packet
  reception, GPIO25 hold across RESET, charger operation, UART back-powering,
  or deep-sleep current. Verify those behaviors on the actual assembled board
  and report which checks were performed.
- Radar target display is for moving-target awareness. Do not claim validated
  biometric accuracy, medical use, or dependable safety alerts.
