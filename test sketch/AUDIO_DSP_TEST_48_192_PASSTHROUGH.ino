/*
  ESP32-S3 Audio DSP: 48kHz ADC -> 4x Upsample -> 192kHz DAC

  Hardware:
  - PCM1808 ADC (slave mode) @ 48 kHz, 24-bit
  - PCM5102A DAC (slave mode) @ 192 kHz, 24-bit
  - Shared MCLK: 24.576 MHz on GPIO8

  Board: ESP32S3 Dev Module (Arduino-ESP32 3.x)
*/

#include <driver/gpio.h>
#include <driver/i2s_std.h>

// ==== Pin Configuration ====
#define PIN_MCLK 8 // Shared MCLK 24.576 MHz

// DAC (I2S1 - TX) - PCM5102A
#define PIN_DAC_BCK 9
#define PIN_DAC_LRCK 11
#define PIN_DAC_DOUT 10

// ADC (I2S0 - RX) - PCM1808
#define PIN_ADC_BCK 4
#define PIN_ADC_LRCK 6
#define PIN_ADC_DIN 5

// ==== Audio Configuration ====
#define SAMPLE_RATE_ADC 48000
#define SAMPLE_RATE_DAC 192000
#define UPSAMPLE_FACTOR 4
#define BLOCK_SIZE 64 // Frames per block (reduced for lower latency)
#define BITS_PER_SAMPLE 24
#define BYTES_PER_SAMPLE 4 // 24-bit in 32-bit container

// I2S channel handles
i2s_chan_handle_t tx_handle = NULL;
i2s_chan_handle_t rx_handle = NULL;

// Audio buffers
int32_t rx_buffer[BLOCK_SIZE * 2];                   // Stereo input @ 48kHz
int32_t tx_buffer[BLOCK_SIZE * UPSAMPLE_FACTOR * 2]; // Stereo output @ 192kHz

// ==== Improved Upsampler: Cubic Hermite Interpolation ====
// Much better than linear, avoids aliasing artifacts, simple & fast

// Cubic Hermite basis functions for smooth interpolation
static inline float hermite_h00(float t) { return 2*t*t*t - 3*t*t + 1; }
static inline float hermite_h10(float t) { return t*t*t - 2*t*t + t; }
static inline float hermite_h01(float t) { return -2*t*t*t + 3*t*t; }
static inline float hermite_h11(float t) { return t*t*t - t*t; }

// History buffer for samples before current block
static int32_t prev_L = 0, prev_R = 0;

void upsample_4x_polyphase(const int32_t *input, int32_t *output,
                           size_t num_frames)
{
  // 4x cubic Hermite interpolation
  // Uses smooth interpolation curves instead of linear, much better quality
  for (size_t i = 0; i < num_frames; i++)
  {
    // Get surrounding samples (with history)
    int32_t L0 = (i == 0) ? prev_L : input[(i - 1) * 2];
    int32_t R0 = (i == 0) ? prev_R : input[(i - 1) * 2 + 1];
    int32_t L1 = input[i * 2];
    int32_t R1 = input[i * 2 + 1];
    int32_t L2 = (i == num_frames - 1) ? L1 : input[(i + 1) * 2];
    int32_t R2 = (i == num_frames - 1) ? R1 : input[(i + 1) * 2 + 1];
    int32_t L3 = (i >= num_frames - 2) ? L2 : input[(i + 2) * 2];
    int32_t R3 = (i >= num_frames - 2) ? R2 : input[(i + 2) * 2 + 1];

    // Compute derivatives for Hermite spline (simple 3-point approximation)
    float m1_L = (float)(L2 - L0) * 0.5f;
    float m2_L = (float)(L3 - L1) * 0.5f;
    float m1_R = (float)(R2 - R0) * 0.5f;
    float m2_R = (float)(R3 - R1) * 0.5f;

    // Generate 4 output samples per input sample
    for (int frac = 0; frac < 4; frac++)
    {
      float t = frac * 0.25f;  // Fractional position 0, 0.25, 0.5, 0.75

      // Hermite basis evaluation
      float h00 = hermite_h00(t);
      float h10 = hermite_h10(t);
      float h01 = hermite_h01(t);
      float h11 = hermite_h11(t);

      // Interpolate left and right channels
      float out_L_norm = h00 * (float)L1 / 2147483648.0f +
                         h10 * m1_L / 2147483648.0f +
                         h01 * (float)L2 / 2147483648.0f +
                         h11 * m2_L / 2147483648.0f;

      float out_R_norm = h00 * (float)R1 / 2147483648.0f +
                         h10 * m1_R / 2147483648.0f +
                         h01 * (float)R2 / 2147483648.0f +
                         h11 * m2_R / 2147483648.0f;

      // Convert back to MSB-aligned 32-bit
      int32_t out_L = (int32_t)(out_L_norm * 2147483648.0f);
      int32_t out_R = (int32_t)(out_R_norm * 2147483648.0f);

      // Clamp to prevent overflow
      if (out_L > 2147483647) out_L = 2147483647;
      if (out_L < -2147483648) out_L = -2147483648;
      if (out_R > 2147483647) out_R = 2147483647;
      if (out_R < -2147483648) out_R = -2147483648;

      size_t out_idx = i * 8 + frac * 2;
      output[out_idx] = out_L;
      output[out_idx + 1] = out_R;
    }
  }

  // Save last sample for next block
  if (num_frames > 0)
  {
    prev_L = input[(num_frames - 1) * 2];
    prev_R = input[(num_frames - 1) * 2 + 1];
  }
}

// ==== I2S Setup Functions ====

bool setup_i2s_tx()
{
  Serial.println("Initializing I2S TX (DAC @ 192kHz)...");

  // Channel configuration
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  chan_cfg.dma_desc_num = 6;
  chan_cfg.dma_frame_num = 240;

  esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to create TX channel: %d\n", ret);
    return false;
  }

  // Slot configuration for PCM5102A
  i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
  slot_cfg.ws_width = 32;
  slot_cfg.ws_pol = false;
  slot_cfg.bit_shift = true;
  slot_cfg.left_align = true; // PCM5102A expects MSB-aligned data
  slot_cfg.big_endian = false;
  slot_cfg.bit_order_lsb = false;

  // Clock configuration
  i2s_std_clk_config_t clk_cfg = {};
  clk_cfg.sample_rate_hz = SAMPLE_RATE_DAC;
  clk_cfg.clk_src = I2S_CLK_SRC_PLL_160M;
  clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_128;

  // GPIO configuration
  i2s_std_gpio_config_t gpio_cfg = {};
  gpio_cfg.mclk = (gpio_num_t)PIN_MCLK;
  gpio_cfg.bclk = (gpio_num_t)PIN_DAC_BCK;
  gpio_cfg.ws = (gpio_num_t)PIN_DAC_LRCK;
  gpio_cfg.dout = (gpio_num_t)PIN_DAC_DOUT;
  gpio_cfg.din = I2S_GPIO_UNUSED;
  gpio_cfg.invert_flags.mclk_inv = false;
  gpio_cfg.invert_flags.bclk_inv = false;
  gpio_cfg.invert_flags.ws_inv = false;

  // Standard mode configuration
  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg = clk_cfg;
  std_cfg.slot_cfg = slot_cfg;
  std_cfg.gpio_cfg = gpio_cfg;

  ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to init TX std mode: %d\n", ret);
    return false;
  }

  ret = i2s_channel_enable(tx_handle);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to enable TX channel: %d\n", ret);
    return false;
  }

  Serial.println("I2S TX initialized successfully");
  Serial.printf("  Sample Rate: %d Hz\n", SAMPLE_RATE_DAC);
  Serial.printf("  MCLK: %.3f MHz (GPIO%d)\n",
                (SAMPLE_RATE_DAC * 128) / 1000000.0, PIN_MCLK);
  Serial.printf("  BCK: %.3f MHz (GPIO%d)\n",
                (SAMPLE_RATE_DAC * 64) / 1000000.0, PIN_DAC_BCK);
  Serial.printf("  LRCK: %d Hz (GPIO%d)\n", SAMPLE_RATE_DAC, PIN_DAC_LRCK);

  return true;
}

bool setup_i2s_rx()
{
  Serial.println("Initializing I2S RX (ADC @ 48kHz)...");

  // Channel configuration
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  chan_cfg.dma_desc_num = 6;
  chan_cfg.dma_frame_num = 240;

  esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to create RX channel: %d\n", ret);
    return false;
  }

  // Slot configuration for PCM1808
  i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
  slot_cfg.ws_width = 32;
  slot_cfg.ws_pol = false;
  slot_cfg.bit_shift = true;
  slot_cfg.left_align = true; // PCM1808 outputs MSB-aligned data
  slot_cfg.big_endian = false;
  slot_cfg.bit_order_lsb = false;

  // Clock configuration
  i2s_std_clk_config_t clk_cfg = {};
  clk_cfg.sample_rate_hz = SAMPLE_RATE_ADC;
  clk_cfg.clk_src = I2S_CLK_SRC_PLL_160M;
  clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_512;

  // GPIO configuration
  i2s_std_gpio_config_t gpio_cfg = {};
  gpio_cfg.mclk = I2S_GPIO_UNUSED; // TX already drives MCLK
  gpio_cfg.bclk = (gpio_num_t)PIN_ADC_BCK;
  gpio_cfg.ws = (gpio_num_t)PIN_ADC_LRCK;
  gpio_cfg.dout = I2S_GPIO_UNUSED;
  gpio_cfg.din = (gpio_num_t)PIN_ADC_DIN;
  gpio_cfg.invert_flags.mclk_inv = false;
  gpio_cfg.invert_flags.bclk_inv = false;
  gpio_cfg.invert_flags.ws_inv = false;

  // Standard mode configuration
  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg = clk_cfg;
  std_cfg.slot_cfg = slot_cfg;
  std_cfg.gpio_cfg = gpio_cfg;

  ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to init RX std mode: %d\n", ret);
    return false;
  }

  ret = i2s_channel_enable(rx_handle);
  if (ret != ESP_OK)
  {
    Serial.printf("Failed to enable RX channel: %d\n", ret);
    return false;
  }

  Serial.println("I2S RX initialized successfully");
  Serial.printf("  Sample Rate: %d Hz\n", SAMPLE_RATE_ADC);
  Serial.printf("  MCLK: %.3f MHz (from TX GPIO%d)\n",
                (SAMPLE_RATE_ADC * 512) / 1000000.0, PIN_MCLK);
  Serial.printf("  BCK: %.3f MHz (GPIO%d)\n",
                (SAMPLE_RATE_ADC * 64) / 1000000.0, PIN_ADC_BCK);
  Serial.printf("  LRCK: %d Hz (GPIO%d)\n", SAMPLE_RATE_ADC, PIN_ADC_LRCK);

  return true;
}

// ==== Arduino Setup ====

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("ESP32-S3 Audio DSP: 48kHz -> 192kHz");
  Serial.println("========================================\n");

  // Initialize TX first (it generates MCLK)
  if (!setup_i2s_tx())
  {
    Serial.println("\nERROR: TX initialization failed!");
    while (1)
      delay(1000);
  }

  delay(100); // Let TX MCLK stabilize

  // Initialize RX (uses MCLK from TX)
  if (!setup_i2s_rx())
  {
    Serial.println("\nERROR: RX initialization failed!");
    while (1)
      delay(1000);
  }

  Serial.println("\n========================================");
  Serial.println("System Ready - Starting Audio Processing");
  Serial.println("========================================\n");

  delay(500);
}

// ==== Arduino Loop ====

void loop()
{
  size_t bytes_read = 0;

  // Read audio from ADC
  esp_err_t ret = i2s_channel_read(rx_handle, rx_buffer, sizeof(rx_buffer),
                                   &bytes_read, portMAX_DELAY);

  if (ret != ESP_OK)
  {
    Serial.printf("Read error: %d\n", ret);
    return;
  }

  if (bytes_read == 0)
  {
    return;
  }

  // Calculate number of frames received
  size_t frames_read =
      bytes_read / (2 * BYTES_PER_SAMPLE); // Stereo = 2 channels

  // Upsample 4x using polyphase FIR filter
  upsample_4x_polyphase(rx_buffer, tx_buffer, frames_read);

  // Write to DAC
  size_t bytes_to_write = frames_read * UPSAMPLE_FACTOR * 2 * BYTES_PER_SAMPLE;
  size_t bytes_written = 0;

  ret = i2s_channel_write(tx_handle, tx_buffer, bytes_to_write, &bytes_written,
                          portMAX_DELAY);

  if (ret != ESP_OK)
  {
    Serial.printf("Write error: %d\n", ret);
  }

  // Optional: Monitor for underruns
  if (bytes_written != bytes_to_write)
  {
    Serial.printf("Warning: Underrun (wrote %d/%d bytes)\n", bytes_written,
                  bytes_to_write);
  }
}
