# ESP32-S3 STEREO DSP BOARD

A board with ESP32-S3, ADC, DAC, LCD display for audio application and beyond. 
While these components can be connected in principle on any breakout board for prototyping, proper power decoupling and series terminator resistors are necessary to avoid noise and interference.
This board ensures that all signals are appropriately decoupled and routed, on a single layer PCB.

The board was used to develop the [PiratESP32 FM RDS STEREO ENCODER](https://github.com/MarcFinns/PiratESP32-FM-RDS-STEREO-ENCODER)

## Features

### ESP32 Board
- **ESP32-S3 N16R8** with dual-core processor with SIMD (vector) instructions to boost DSP applications
 **ADC**: PCM1808 I2S audio ADC (24 bit, up to 96 kHz sample rate, I2S slave)
- **DAC**: PCM5102A I2S audio DAC (16/24/32 bit, up to 192 kHz sample rate, I2S slave)
- **Display (optional)** ILI9341 320×240 TFT LCD (SPI interface)
- **GPIO connector** for LED/buttons or anything else

## Pin Configuration

### I2S Audio
```
Master Clock (shared):  GPIO 8  (tested at 24.576 MHz MCLK)

ADC Input:
  BCK  (Bit Clock):     GPIO 4
  LRCK (Word Select):   GPIO 6
  DIN  (Serial Data):   GPIO 5

DAC Output:
  BCK  (Bit Clock):     GPIO 9
  LRCK (Word Select):   GPIO 11
  DOUT (Serial Data):   GPIO 10

ILI9341 TFT Display
	SCK  (SPI Clock):       GPIO 40
	MOS	I (SPI Data):        GPIO 41
	DC   (Data/Command):    GPIO 42
	CS   (Chip Select):     GPIO 1
	RST  (Reset):           GPIO 2

GPIO Connector
	3V3
	GPIO39
	GPIO38
	GPIO48
	GPIO47
	GPIO21
```

## Important notes

- Some PCM1808 ADC breakout boards have a **very aggressive low pass filter** (the filter is optional from the datasheet, and they chose a wrong capacitor value). This results in a lowpass filter at 6Khz! If sound is muffled, you can just desolder the caps (or break them, it is fine). See picture

- The board is designed to be powered 
both from USB on the ESP and from the Vin, 
without conflicts. However, to enable the board 
to work when powered via USB, it is often 
necessary to connect a jumper on the ESP32-
S3 board that enables 5V out. 
The location of the jumper depends on 
the flavour of the development board! **Look it up!**

- The board can be powered with a power supply connected to the Vin connector (12v max) so that it can work standalone / when not connected to a PC via USB. To that end, a LDO is included in the schematic. It can be skipped if only USB power is necessary. The LDO (AMS1117 family) can be either ADJ or 5V fixed version. A solder jumper on the PCB is provided to select the version. If fixed, connect SJ1 jumper to GND. If adj, connect to R divider. **If left not connected, bad things will happen**

- The PCB and schematic in the folder are correct. The pictures are of my first prototybe that had some mistakes, corrected later!

- SMD components are 0805 size. If uneasy with SMD, one could just redesign the PCB with through hole, keeping the schematic and even the routing - just make it larger


## Breakout boards

The following breakout boards are matching the PCB.
If different ones are used, schematic is still valid but re-routing of the PCB might become necessary.

- [ILI9341 Display](https://nl.aliexpress.com/item/32913890225.html)
- [PCM1808 ADC breakout board](https://nl.aliexpress.com/item/1005006749086540.html)
- [PCM5102A DAC breakout board](https://nl.aliexpress.com/item/1005009648364414.html)
- [ESP32-S3 N16R8](https://nl.aliexpress.com/item/1005007319706057.html)

## References

- [PCM1808 ADC datasheet](https://www.ti.com/lit/ds/symlink/pcm1808.pdf)
- [PCM5102A DAC datasheet](https://www.ti.com/lit/ds/symlink/pcm5102a.pdf)
- [ESP32-S3 documentation](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP32 I2S Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html)



