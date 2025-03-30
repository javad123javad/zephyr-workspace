#ifndef _SI5351_H_
#define _SI5351_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

typedef enum {
    SI5351_PLL_A = 0,
    SI5351_PLL_B,
} si5351PLL_t;

typedef enum {
    SI5351_R_DIV_1   = 0,
    SI5351_R_DIV_2   = 1,
    SI5351_R_DIV_4   = 2,
    SI5351_R_DIV_8   = 3,
    SI5351_R_DIV_16  = 4,
    SI5351_R_DIV_32  = 5,
    SI5351_R_DIV_64  = 6,
    SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

typedef enum {
    SI5351_DRIVE_STRENGTH_2MA = 0x00,
    SI5351_DRIVE_STRENGTH_4MA = 0x01,
    SI5351_DRIVE_STRENGTH_6MA = 0x02,
    SI5351_DRIVE_STRENGTH_8MA = 0x03,
} si5351DriveStrength_t;

typedef struct {
    int32_t mult;
    int32_t num;
    int32_t denom;
} si5351PLLConfig_t;

typedef struct {
    uint8_t allowIntegerMode;
    int32_t div;
    int32_t num;
    int32_t denom;
    si5351RDiv_t rdiv;
} si5351OutputConfig_t;

struct si5351_dev {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
    int32_t correction;
};

/* Basic Interface */
int si5351_init(struct si5351_dev *dev, int32_t correction);
int si5351_setup_clk0(struct si5351_dev *dev, int32_t fclk, si5351DriveStrength_t drive_strength);
int si5351_setup_clk2(struct si5351_dev *dev, int32_t fclk, si5351DriveStrength_t drive_strength);
int si5351_enable_outputs(struct si5351_dev *dev, uint8_t enabled);

/* Advanced Interface */
int si5351_calc(int32_t fclk, int32_t correction, si5351PLLConfig_t *pll_conf, si5351OutputConfig_t *out_conf);
int si5351_setup_pll(struct si5351_dev *dev, si5351PLL_t pll, si5351PLLConfig_t *conf);
int si5351_setup_output(struct si5351_dev *dev, uint8_t output, si5351PLL_t pll_source,
                        si5351DriveStrength_t drive_strength, si5351OutputConfig_t *conf, uint8_t phase_offset);

#endif /* _SI5351_H_ */