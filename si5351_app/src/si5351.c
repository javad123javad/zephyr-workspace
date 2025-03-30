#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "si5351.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(si5351, LOG_LEVEL_DBG);

// Register definitions (from sample code)

enum {
    SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL               = 3,
    SI5351_REGISTER_16_CLK0_CONTROL                       = 16,
    SI5351_REGISTER_17_CLK1_CONTROL                       = 17,
    SI5351_REGISTER_18_CLK2_CONTROL                       = 18,
    SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1           = 42,
    SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1           = 50,
    SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1           = 58,
    SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET         = 165,
    SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET         = 166,
    SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET         = 167,
    SI5351_REGISTER_177_PLL_RESET                         = 177,
    SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183,
};
typedef enum {
	SI5351_CRYSTAL_LOAD_10PF = (3 << 6),
} si5351CrystalLoad_t;

// Private functions
static int si5351_write(struct si5351_dev *dev, uint8_t reg, uint8_t value);
static int si5351_write_bulk(struct si5351_dev *dev, uint8_t baseaddr, int32_t p1, int32_t p2,
			     int32_t p3, uint8_t div_by_4, si5351RDiv_t rdiv);

// Initialize Si5351
int si5351_init(struct si5351_dev *dev, int32_t correction)
{
	if (!dev || !device_is_ready(dev->i2c_dev)) {
		return -ENODEV;
	}

	dev->correction = correction;

	// Disable all outputs
	int ret = si5351_write(dev, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);
	if (ret) {
		return ret;
	}

	// Power down all output drivers
	for (uint8_t i = 0; i < 8; i++) {
		ret = si5351_write(dev, SI5351_REGISTER_16_CLK0_CONTROL + i, 0x80);
		if (ret) {
			return ret;
		}
	}

	// Set crystal load capacitance (10pF)
	ret = si5351_write(dev, SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
			   SI5351_CRYSTAL_LOAD_10PF);
	return ret;
}

// Setup PLL
int si5351_setup_pll(struct si5351_dev *dev, si5351PLL_t pll, si5351PLLConfig_t *conf) {
    int32_t p1 = 128 * conf->mult + (128 * conf->num) / conf->denom - 512;
    int32_t p2 = (128 * conf->num) % conf->denom;
    int32_t p3 = conf->denom;

    uint8_t baseaddr = (pll == SI5351_PLL_A) ? 26 : 34;
    int ret = si5351_write_bulk(dev, baseaddr, p1, p2, p3, 0, 0);
    if (ret) return ret;

    ret = si5351_write(dev, SI5351_REGISTER_177_PLL_RESET, (1 << 7) | (1 << 5));
    if (ret) return ret;

    LOG_INF("PLL%d set: mult=%d, num=%d, denom=%d", pll, conf->mult, conf->num, conf->denom);
    return 0;
}



int si5351_setup_output(struct si5351_dev *dev, uint8_t output, si5351PLL_t pll_source,
                        si5351DriveStrength_t drive_strength, si5351OutputConfig_t *conf, uint8_t phase_offset) {
    if (output > 2) return -EINVAL;

    int32_t div = conf->div;
    int32_t num = conf->num;
    int32_t denom = conf->denom;
    uint8_t div_by_4 = 0;//(div == 4) ? 0x3 : 0;
    // int32_t p1 = (div == 4) ? 0 : (128 * div + (128 * num) / denom - 512);
    // int32_t p2 = (div == 4) ? 0 : ((128 * num) % denom);
    // int32_t p3 = (div == 4) ? 1 : denom;
    int32_t p1,p2,p3;
    if((!conf->allowIntegerMode) && ((div < 8) || ((div == 8) && (num == 0)))) {
        // div in { 4, 6, 8 } is possible only in integer mode
        return 2;
    }

    if(div == 4) {
        // special DIVBY4 case, see AN619 4.1.3
        p1 = 0;
        p2 = 0;
        p3 = 1;
        div_by_4 = 0x3;
    } else {
        p1 = 128 * div + ((128 * num)/denom) - 512;
        // P2 = 128 * num - denom * (128 * num)/denom;
        p2 = (128 * num) % denom;
        p3 = denom;
    }
    uint8_t baseaddr, phase_offset_reg, clk_control_reg;
    switch (output) {
    case 0:
        baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
        phase_offset_reg = SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET;
        clk_control_reg = SI5351_REGISTER_16_CLK0_CONTROL;
        break;
    case 1:
        baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
        phase_offset_reg = SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET;
        clk_control_reg = SI5351_REGISTER_17_CLK1_CONTROL;
        break;
    case 2:
        baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
        phase_offset_reg = SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET;
        clk_control_reg = SI5351_REGISTER_18_CLK2_CONTROL;
        break;
        default:
            return -EINVAL;
    }

    uint8_t clk_control = 0x0C | drive_strength;
    if (pll_source == SI5351_PLL_B) clk_control |= (1 << 5);
    if (conf->allowIntegerMode && (num == 0 || div == 4)) clk_control |= (1 << 6);

    int ret = si5351_write(dev, clk_control_reg, clk_control);
    if (ret) return ret;
    ret = si5351_write_bulk(dev, baseaddr, p1, p2, p3, div_by_4, conf->rdiv);
    if (ret) return ret;
    ret = si5351_write(dev, phase_offset_reg, phase_offset & 0x7F);
    if (ret) return ret;

    LOG_INF("CLK%d set: div=%d, num=%d, denom=%d, rdiv=%d, PLL=%d, control=0x%02x",
            output, conf->div, conf->num, conf->denom, conf->rdiv, pll_source, clk_control);
    return 0;
}

// Calculate PLL and Output settings
int si5351_calc(int32_t fclk, int32_t correction, si5351PLLConfig_t *pll_conf,
		si5351OutputConfig_t *out_conf)
{
	if (fclk < 8000) fclk = 8000;
    else if (fclk > 160000000) fclk = 160000000;

    out_conf->allowIntegerMode = 1;
    if(fclk < 1000000) {
        // For frequencies in [8_000, 500_000] range we can use si5351_Calc(Fclk*64, ...) and SI5351_R_DIV_64.
        // In practice it's worth doing for any frequency below 1 MHz, since it reduces the error.
        fclk *= 64;
        out_conf->rdiv = SI5351_R_DIV_64;
    } else {
        out_conf->rdiv = SI5351_R_DIV_1;
    }

    fclk -= (int32_t)((((double)fclk) / 100000000.0) * ((double)correction));

    const int32_t fxtal = 25000000;
    int32_t a, b, c, x, y, z, t;

    if(fclk < 81000000) {
        // Valid for Fclk in 0.5..112.5 MHz range
        // However an error is > 6 Hz above 81 MHz
        a = 36; // PLL runs @ 900 MHz
        b = 0;
        c = 1;
        int32_t fpll = 900000000;
        x = fpll/fclk;
        t = (fclk >> 20) + 1;
        y = (fpll % fclk) / t;
        z = fclk / t;
    } else {
        // Valid for Fclk in 75..160 MHz range
        if(fclk >= 150000000) {
            x = 4;
        } else if (fclk >= 100000000) {
            x = 6;
        } else {
            x = 8;
        }
        y = 0;
        z = 1;
        
        int32_t numerator = x*fclk;
        a = numerator/fxtal;
        t = (fxtal >> 20) + 1;
        b = (numerator % fxtal) / t;
        c = fxtal / t;
    }

    pll_conf->mult = a;
    pll_conf->num = b;
    pll_conf->denom = c;
    out_conf->div = x;
    out_conf->num = y;
    out_conf->denom = z;

    LOG_INF("Calc for %d Hz: PLL a=%d, b=%d, c=%d; Out x=%d, y=%d, z=%d, rdiv=%d",
            fclk, a, b, c, x, y, z, out_conf->rdiv);
    return 0;
}

// Setup CLK0
int si5351_setup_clk0(struct si5351_dev *dev, int32_t fclk, si5351DriveStrength_t drive_strength)
{
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;

	int ret = si5351_calc(fclk, dev->correction, &pll_conf, &out_conf);
	if (ret) {
		return ret;
	}

	ret = si5351_setup_pll(dev, SI5351_PLL_A, &pll_conf);
	if (ret) {
		return ret;
	}

	return si5351_setup_output(dev, 0, SI5351_PLL_A, drive_strength, &out_conf, 0);
}

// Setup CLK2
int si5351_setup_clk2(struct si5351_dev *dev, int32_t fclk, si5351DriveStrength_t drive_strength)
{
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;

	int ret = si5351_calc(fclk, dev->correction, &pll_conf, &out_conf);
	if (ret) {
		return ret;
	}

	ret = si5351_setup_pll(dev, SI5351_PLL_B, &pll_conf);
	if (ret) {
		return ret;
	}

	return si5351_setup_output(dev, 2, SI5351_PLL_B, drive_strength, &out_conf, 0);
}

// Enable Outputs
int si5351_enable_outputs(struct si5351_dev *dev, uint8_t enabled)
{
	return si5351_write(dev, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, ~enabled);
}

// Write single register
static int si5351_write(struct si5351_dev *dev, uint8_t reg, uint8_t value)
{
	int ret = i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, reg, value);
	if (ret) {
		LOG_ERR("Write failed at reg 0x%02x: %d", reg, ret);
	} else {
		LOG_DBG("Wrote 0x%02x to reg 0x%02x", value, reg);
	}
	return ret;
}

// Write bulk registers for PLL/Output
static int si5351_write_bulk(struct si5351_dev *dev, uint8_t baseaddr, int32_t P1, int32_t P2,
			     int32_t P3, uint8_t div_by_4, si5351RDiv_t rdiv)
{
	// uint8_t data[8] = {(p3 >> 8) & 0xFF,
	// 		   p3 & 0xFF,
	// 		   ((p1 >> 16) & 0x3) | ((div_by_4 & 0x3) << 2) | ((rdiv & 0x7) << 4),
	// 		   (p1 >> 8) & 0xFF,
	// 		   p1 & 0xFF,
	// 		   ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0xF),
	// 		   (p2 >> 8) & 0xFF,
	// 		   p2 & 0xFF};
	// int ret = i2c_write(dev->i2c_dev, data, sizeof(data), dev->i2c_addr);
	// if (ret) {
	// 	LOG_ERR("Bulk write failed at baseaddr 0x%02x: %d", baseaddr, ret);
	// } else {
	// 	LOG_DBG("Bulk wrote to 0x%02x: %02x %02x %02x %02x %02x %02x %02x %02x", baseaddr,
	// 		data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	// }
    int ret = 0; 
    ret |= si5351_write(dev,baseaddr,   (P3 >> 8) & 0xFF);
    ret |= si5351_write(dev, baseaddr+1, P3 & 0xFF);
    ret |= si5351_write(dev, baseaddr+2, ((P1 >> 16) & 0x3) | ((div_by_4 & 0x3) << 2) | ((rdiv & 0x7) << 4));
    ret |= si5351_write(dev, baseaddr+3, (P1 >> 8) & 0xFF);
    ret |= si5351_write(dev, baseaddr+4, P1 & 0xFF);
    ret |= si5351_write(dev, baseaddr+5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0xF));
    ret |= si5351_write(dev, baseaddr+6, (P2 >> 8) & 0xFF);
    ret |= si5351_write(dev, baseaddr+7, P2 & 0xFF);
	return ret;
}