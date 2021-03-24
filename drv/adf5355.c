/***************************************************************************//**
 *   @file   adf5355.c
 *   @brief  Implementation of adf5355 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <inttypes.h>
#include <stdio.h>
#include "error.h"
#include <malloc.h>
#include "delay.h"
#include "util.h"
#include "adf5355.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * SPI register write to device.
 * @param dev - The device structure.
 * @param reg - The register address.
 * @param val - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
static int32_t adf5355_write(struct adf5355_dev *dev,
			     uint8_t reg_addr,
			     uint32_t data)
{
	uint8_t buf[ADF5355_SPI_NO_BYTES];
	data = data | reg_addr;

	buf[0] = data >> 24;
	buf[1] = data >> 16;
	buf[2] = data >> 8;
	buf[3] = data;

	return spi_write_and_read(dev->spi_desc, buf, ARRAY_SIZE(buf));
}

/**
 * Compute PLL parameters.
 * @param vco - The VCO frequency.
 * @param pfd - The PFD frequency.
 * @param integer - The integer division factor.
 * @param fract1 - The fractionality.
 * @param fract2 - The auxiliary fractionality.
 * @param mod2 - The auxiliary modulus.
 * @return None.
 */
static void adf5355_pll_fract_n_compute(uint64_t vco,
					uint64_t pfd,
					uint32_t *integer,
					uint32_t *fract1,
					uint32_t *fract2,
					uint32_t *mod2)
{
	uint64_t tmp;
	uint32_t gcd_div;

	tmp = do_div(&vco, pfd);
	tmp = tmp * ADF5355_MODULUS1;
	*fract2 = do_div(&tmp, pfd);

	*integer = vco;
	*fract1 = tmp;

	*mod2 = pfd;

	while (*mod2 > ADF5355_MAX_MODULUS2) {
		*mod2 >>= 1;
		*fract2 >>= 1;
	}

	gcd_div = greatest_common_divisor(*fract2, *mod2);
	*mod2 /= gcd_div;
	*fract2 /= gcd_div;
}

/**
 * ADF5355 Register configuration
 * @param dev - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
static int32_t adf5355_reg_config(struct adf5355_dev *dev)
{
	int32_t ret, i;

	for (i = ADF5355_REG(12); i >= ADF5355_REG(1); i--) {
		ret = adf5355_write(dev, ADF5355_REG(i), dev->regs[i]);
		if (ret != SUCCESS)
			return ret;
	}

	udelay(dev->delay_us);

	return adf5355_write(dev, ADF5355_REG(0), dev->regs[0]);
}

/**
 * Set the output frequency for one channel.
 * @param dev - The device structure.
 * @param freq - The output frequency.
 * @param chan - The selected channel.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
static int32_t adf5355_set_freq(struct adf5355_dev *dev,
				uint64_t freq,
				uint8_t chan)
{
	uint32_t cp_bleed;
	bool prescaler;

	if (chan > dev->num_channels)
		return FAILURE;

	if (chan == 0) {
		if ((freq > dev->max_out_freq) || (freq < dev->min_out_freq))
			return -EINVAL;

		dev->rf_div_sel = 0;

		while (freq < dev->min_vco_freq) {
			freq <<= 1;
			dev->rf_div_sel++;
		}
	} else {
		/* ADF5355 RFoutB 6800...13600 MHz */
		if ((freq > ADF5355_MAX_OUTB_FREQ) || (freq < ADF5355_MIN_OUTB_FREQ))
			return -EINVAL;

		freq >>= 1;
	}

	adf5355_pll_fract_n_compute(freq, dev->fpfd, &dev->integer, &dev->fract1,
				    &dev->fract2, &dev->mod2);

	prescaler = (dev->integer >= ADF5355_MIN_INT_PRESCALER_89);

	cp_bleed = DIV_ROUND_UP(400 * dev->cp_ua, dev->integer * 375);
	cp_bleed = clamp(cp_bleed, 1U, 255U);

	dev->regs[ADF5355_REG(0)] = ADF5355_REG0_INT(dev->integer) |
				    ADF5355_REG0_PRESCALER(prescaler) |
				    ADF5355_REG0_AUTOCAL(1);

	dev->regs[ADF5355_REG(1)] = ADF5355_REG1_FRACT(dev->fract1);

	dev->regs[ADF5355_REG(2)] = ADF5355_REG2_MOD2(dev->mod2) |
				    ADF5355_REG2_FRAC2(dev->fract2);

	dev->regs[ADF5355_REG(6)] = ADF5355_REG6_OUTPUT_PWR(dev->outa_power) |
				    ADF5355_REG6_RF_OUT_EN(dev->outa_en) |
				    (dev->dev_id == ADF5355 ? ADF5355_REG6_RF_OUTB_EN(!dev->outb_en) :
				     ADF4355_REG6_OUTPUTB_PWR(dev->outb_power) |
				     ADF4355_REG6_RF_OUTB_EN(dev->outb_en)) |
				    ADF5355_REG6_MUTE_TILL_LOCK_EN(dev->mute_till_lock_en) |
				    ADF5355_REG6_CP_BLEED_CURR(cp_bleed) |
				    ADF5355_REG6_RF_DIV_SEL(dev->rf_div_sel) |
				    ADF5355_REG6_FEEDBACK_FUND(1) |
				    ADF5355_REG6_NEG_BLEED_EN(dev->cp_neg_bleed_en) |
				    ADF5355_REG6_GATED_BLEED_EN(dev->cp_gated_bleed_en) |
				    ADF5355_REG6_DEFAULT;

	dev->freq_req = freq;

	return adf5355_reg_config(dev);
}

/**
 * Get the output frequency of one channel.
 * @param dev - The device structure.
 * @param channel - The selected channel.
 * @return The ouput frequency.
 */
static uint64_t adf5355_pll_fract_n_get_rate(struct adf5355_dev *dev,
		uint32_t channel)
{
	uint64_t val, tmp;

	val = (((uint64_t)dev->integer * ADF5355_MODULUS1) + dev->fract1) * dev->fpfd;
	tmp = (uint64_t)dev->fract2 * dev->fpfd;
	do_div(&tmp, dev->mod2);
	val += tmp + ADF5355_MODULUS1 / 2;

	do_div(&val, ADF5355_MODULUS1 *
	       (1 << (channel == 1 ? 0 : dev->rf_div_sel)));

	if (channel == 1)
		val <<= 1;

	return val;
}

/**
 * Recalculate rate corresponding to a channel.
 * @param dev - The device structure.
 * @param chan - Channel number.
 * @param rate - Channel rate.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t adf5355_clk_recalc_rate(struct adf5355_dev *dev, uint32_t chan,
				uint64_t *rate)
{
	if (chan > dev->num_channels)
		return FAILURE;

	*rate = adf5355_pll_fract_n_get_rate(dev, chan);

	return SUCCESS;
}

/**
 * Set channel rate.
 * @param dev - The device structure.
 * @param chan - Channel number.
 * @param rate - Channel rate.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t adf5355_clk_set_rate(struct adf5355_dev *dev, uint32_t chan,
			     uint64_t rate)
{
	if (chan >= dev->num_channels)
		return FAILURE;

	return adf5355_set_freq(dev, rate, chan);
}

/**
 * Setup the device.
 * @param dev - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
static int32_t adf5355_setup(struct adf5355_dev *dev)
{
	uint32_t tmp;

	dev->ref_div_factor = 0;

	/* Calculate and maximize PFD frequency */
	do {
		dev->ref_div_factor++;
		dev->fpfd = (dev->clkin_freq * (dev->ref_doubler_en ? 2 : 1)) /
			    (dev->ref_div_factor * (dev->ref_div2_en ? 2 : 1));
	} while (dev->fpfd > ADF5355_MAX_FREQ_PFD);

	tmp = DIV_ROUND_CLOSEST(dev->cp_ua - 315, 315U);
	tmp = clamp(tmp, 0U, 15U);

	dev->regs[ADF5355_REG(4)] = ADF5355_REG4_COUNTER_RESET_EN(0) |
				    ADF5355_REG4_CP_THREESTATE_EN(0) |
				    ADF5355_REG4_POWER_DOWN_EN(0) |
				    ADF5355_REG4_PD_POLARITY_POS(!dev->phase_detector_polarity_neg) |
				    ADF5355_REG4_MUX_LOGIC(dev->mux_out_3v3_en) |
				    ADF5355_REG4_REFIN_MODE_DIFF(dev->ref_diff_en) |
				    ADF5355_REG4_CHARGE_PUMP_CURR(tmp) |
				    ADF5355_REG4_DOUBLE_BUFF_EN(1) |
				    ADF5355_REG4_10BIT_R_CNT(dev->ref_div_factor) |
				    ADF5355_REG4_RDIV2_EN(dev->ref_div2_en) |
				    ADF5355_REG4_RMULT2_EN(dev->ref_doubler_en) |
				    ADF5355_REG4_MUXOUT(dev->mux_out_sel);

	dev->regs[ADF5355_REG(5)] = ADF5355_REG5_DEFAULT;

	dev->regs[ADF5355_REG(7)] = ADF5355_REG7_LD_MODE_INT_N_EN(0) |
				    ADF5355_REG7_FACT_N_LD_PRECISION(3) |
				    ADF5355_REG7_LOL_MODE_EN(0) |
				    ADF5355_REG7_LD_CYCLE_CNT(0) |
				    ADF5355_REG7_LE_SYNCED_REFIN_EN(1) |
				    ADF5355_REG7_DEFAULT;

	dev->regs[ADF5355_REG(8)] = ADF5355_REG8_DEFAULT;

	/* Calculate Timeouts */
	tmp = DIV_ROUND_UP(dev->fpfd, 20000U * 30U);
	tmp = clamp(tmp, 1U, 1023U);

	dev->regs[ADF5355_REG(9)] = ADF5355_REG9_TIMEOUT(tmp) |
				    ADF5355_REG9_SYNTH_LOCK_TIMEOUT(DIV_ROUND_UP(dev->fpfd * 2U, 100000U * tmp)) |
				    ADF5355_REG9_ALC_TIMEOUT(DIV_ROUND_UP(dev->fpfd * 5U, 100000U * tmp)) |
				    ADF5355_REG9_VCO_BAND_DIV(DIV_ROUND_UP(dev->fpfd, 2400000U));

	tmp = DIV_ROUND_UP(dev->fpfd / 100000U - 2, 4);
	tmp = clamp(tmp, 1U, 255U);

	/* Delay > 16 ADC_CLK cycles */
	dev->delay_us = DIV_ROUND_UP(16000000UL, dev->fpfd / (4 * tmp + 2));

	dev->regs[ADF5355_REG(10)] = ADF5355_REG10_ADC_EN(1) |
				     ADF5355_REG10_ADC_CONV_EN(1) |
				     ADF5355_REG10_ADC_CLK_DIV(tmp) |
				     ADF5355_REG10_DEFAULT;

	dev->regs[ADF5355_REG(11)] = ADF5355_REG11_DEFAULT;

	dev->regs[ADF5355_REG(12)] = ADF5355_REG12_PHASE_RESYNC_CLK_DIV(1) |
				     ADF5355_REG12_DEFAULT;

	return adf5355_set_freq(dev, dev->freq_req, dev->freq_req_chan);
}


/**
 * @brief Initializes the ADF5355.
 * @param device - The device structure.
 * @param init_param - The structure containing the device initial parameters.
 * @return Returns 0 in case of success or negative error code.
 */
int32_t adf5355_init(struct adf5355_dev **device,
		     const struct adf5355_init_param *init_param)
{
	int32_t ret;
	struct adf5355_dev *dev;

	dev = (struct adf5355_dev *)calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	/* SPI */
	ret = spi_init(&dev->spi_desc, init_param->spi_init);
	if (ret != SUCCESS)
		goto error_dev;

	dev->dev_id = init_param->dev_id;
	dev->freq_req = init_param->freq_req;
	dev->freq_req_chan = init_param->freq_req_chan;
	dev->clkin_freq = init_param->clkin_freq;
	dev->cp_ua = init_param->cp_ua;
	dev->cp_neg_bleed_en = init_param->cp_neg_bleed_en;
	dev->cp_gated_bleed_en = init_param->cp_gated_bleed_en;
	dev->mute_till_lock_en = init_param->mute_till_lock_en;
	dev->outa_en = init_param->outa_en;
	dev->outb_en = init_param->outb_en;
	dev->outa_power = init_param->outb_power;
	dev->phase_detector_polarity_neg = init_param->phase_detector_polarity_neg;
	dev->ref_diff_en = init_param->ref_diff_en;
	dev->mux_out_3v3_en = init_param->mux_out_3v3_en;
	dev->ref_doubler_en = init_param->ref_doubler_en;
	dev->ref_div2_en = init_param->ref_div2_en;
	dev->mux_out_sel = init_param->mux_out_sel;
	dev->num_channels = 2;

	switch (dev->dev_id) {
	case ADF5355:
		dev->max_out_freq = ADF5355_MAX_OUT_FREQ;
		dev->min_out_freq = ADF5355_MIN_OUT_FREQ;
		dev->min_vco_freq = ADF5355_MIN_VCO_FREQ;
		break;
	case ADF4355:
		dev->max_out_freq = ADF4355_MAX_OUT_FREQ;
		dev->min_out_freq = ADF4355_MIN_OUT_FREQ;
		dev->min_vco_freq = ADF4355_MIN_VCO_FREQ;
		break;
	case ADF4355_2:
		dev->max_out_freq = ADF4355_2_MAX_OUT_FREQ;
		dev->min_out_freq = ADF4355_2_MIN_OUT_FREQ;
		dev->min_vco_freq = ADF4355_2_MIN_VCO_FREQ;
		break;
	case ADF4355_3:
		dev->max_out_freq = ADF4355_3_MAX_OUT_FREQ;
		dev->min_out_freq = ADF4355_3_MIN_OUT_FREQ;
		dev->min_vco_freq = ADF4355_3_MIN_VCO_FREQ;
		break;
	}

	ret = adf5355_setup(dev);
	if (ret != SUCCESS)
		goto error_spi;

	*device = dev;

	return ret;

error_spi:
	spi_remove(dev->spi_desc);
error_dev:
	free(dev);

	return ret;
}

/**
 * Remove the device - release resources.
 * @param device - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t adf5355_remove(struct adf5355_dev *device)
{
	int32_t ret;

	if (device->spi_desc)
		ret = spi_remove(device->spi_desc);

	free(device);

	return ret;
}