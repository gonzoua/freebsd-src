/*-
 * Copyright (c) 2019 Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Driver for Realtek rt5640 audio codec
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/endian.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/extres/clk/clk.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <sys/sysctl.h>

#include <dev/sound/fdt/audio_dai.h>
#include <dev/sound/pcm/sound.h>

#include "iicbus_if.h"
#include "mixer_if.h"
#include "audio_dai_if.h"

#define	RT5640_RESET		0x00
#define	RT5640_HP_VOL		0x02
#define		MAX_HPO_VOLUME		(0x27)
#define		HP_VOL_MU_HPO_L		(1 << 15)
#define		HP_VOL_MU_HPOVOLL_IN	(1 << 14)
#define		HP_VOL_MU_HPO_R		(1 << 7)
#define		HP_VOL_MU_HPOVOLR_IN	(1 << 6)
#define		HP_VOL_VOLUME(l, r)	((l) << 8 | (r))
#define		HP_VOL_VOLUME_MASK	HP_VOL_VOLUME(0x3f, 0x3f)
#define	RT5640_IN1_IN2		0x0d
#define		IN1_IN2_EN_IN1_DF	(1 << 7)
#define	RT5640_DAC1_DIG_VOL	0x19
#define		MAX_DIG_VOLUME		0xAF
#define		DAC1_DIG_VOL(l, r)	(((l) << 8) | (r))
#define		DAC1_DIG_VOL_MASK	DAC1_DIG_VOL(0xff, 0xff)
#define		DAC1_DIG_VOL_R(v)	((v) & 0xff)
#define		DAC1_DIG_VOL_L(v)	(((v) >> 8) & 0xff)
#define	RT5640_ADC_DIG_VOL	0x1C
#define		ADC_DIG_VOL_MU_ADC_VOL_L	(1 << 15)
#define		ADC_DIG_VOL_AD_GAIN_L(v)	((v) << 8)
#define		ADC_DIG_VOL_AD_GAIN_L_MASK	(0x7f << 8)
#define		ADC_DIG_VOL_MU_ADC_VOL_R	(1 << 7)
#define		ADC_DIG_VOL_AD_GAIN_R(v)	((v) << 0)
#define		ADC_DIG_VOL_AD_GAIN_R_MASK	(0x7f << 0)
#define	RT5640_STO_ADC_MIXER	0x27
#define		STO_ADC_MIXER_MU_STEREO_ADCL1	(1 << 14)
#define		STO_ADC_MIXER_SEL_STEREO_ADC1	(1 << 12)
#define		STO_ADC_MIXER_MU_STEREO_ADCR1	(1 << 6)
#define	RT5640_AD_DA_MIXER	0x29
#define		AD_DA_MIXER_MU_IF1_DAC_L		(1 << 14)
#define		AD_DA_MIXER_MU_IF1_DAC_R		(1 << 6)
#define	RT5640_STO_DAC_MIXER	0x2a
#define		STO_DAC_MIXER_MU_STEREO_DACL1	(1 << 14)
#define		STO_DAC_MIXER_MU_STEREO_DACR1	(1 << 6)
#define	RT5640_DIG_MIXER	0x2c
#define		DIG_MIXER_MU_DACL1_TO_DACL		(1 << 15)
#define		DIG_MIXER_MU_DACR1_TO_DACR		(1 << 11)
#define	RT5640_DSP_PATH2	0x2e
#define	RT5640_REC_L2_MIXER	0x3c
#define		L2_MIXER_MU_BST1_RECMIXL	(1 << 1)
#define	RT5640_REC_R2_MIXER	0x3e
#define		R2_MIXER_MU_BST1_RECMIXR	(1 << 1)
#define	RT5640_HPO_MIXER	0x45
#define		HPO_MIXER_MU_HPOVOL_HPOMIX	(1 << 13)
#define	RT5640_OUT_L3_MIXER	0x4f
#define		OUT_L3_MIXER_MU_DACL1_OUTMIXL	(1 << 0)
#define	RT5640_OUT_R3_MIXER	0x52
#define		OUT_R3_MIXER_MU_DACR1_OUTMIXR	(1 << 0)
#define	RT5640_LOUT_MIXER	0x53
#define	RT5640_PWR_DIG1		0x61
#define		PWR_DIG1_EN_I2S1		(1 << 15)
#define		PWR_DIG1_POWER_DAC_L_1		(1 << 12)
#define		PWR_DIG1_POWER_DAC_R_1		(1 << 11)
#define		PWR_DIG1_POWER_ADC_L		(1 << 2)
#define		PWR_DIG1_POWER_ADC_R		(1 << 1)
#define	RT5640_PWR_DIG2		0x62
#define		PWR_DIG2_POW_ADC_STEREO_FILTER	(1 << 15)
#define	RT5640_PWR_ANLG1	0x63
#define		PWR_ANLG1_EN_FASTB1	(1 << 14)
#define		PWR_ANLG1_POW_MAIN_BIAS	(1 << 13)
#define		PWR_ANLG1_EN_L_HP	(1 << 7)
#define		PWR_ANLG1_EN_R_HP	(1 << 6)
#define		PWR_ANLG1_EN_AMP_HP	(1 << 5)
#define		PWR_ANLG1_EN_FASTB2	(1 << 3)
#define	RT5640_PWR_ANLG2	0x64
#define		PWR_ANLG2_POW_BST1	(1 << 15)
#define		PWR_ANLG2_POW_MICBIAS1	(1 << 11)
#define		PWR_ANLG2_POW_PLL	(1 << 9)
#define	RT5640_PWR_MIXER	0x65
#define		PWR_MIXER_POW_OUTMIXL	(1 << 15)
#define		PWR_MIXER_POW_OUTMIXR	(1 << 14)
#define		PWR_MIXER_POW_RECMIXL	(1 << 11)
#define		PWR_MIXER_POW_RECMIXR	(1 << 10)
#define	RT5640_PWR_VOL		0x66
#define		PWR_VOL_POW_HPOVOLL	(1 << 11)
#define		PWR_VOL_POW_HPOVOLR	(1 << 10)
#define	RT5640_PR_INDEX		0x6a
#define	RT5640_PR_DATA		0x6c
#define	RT5640_I2S1_SDP		0x70
#define		I2S1_SDP_SEL_I2S1_MS_SLAVE	(1 << 15)
#define		I2S1_SDP_INV_I2S1_BCLK		(1 << 7)
#define		I2S1_SDP_SEL_I2S1_LEN_MASK	(3 << 2)
#define		I2S1_SDP_SEL_I2S1_FORMAT_MASK	(3 << 0)
#define		I2S1_SDP_SEL_I2S1_FORMAT_I2S	(0 << 0)
#define		I2S1_SDP_SEL_I2S1_FORMAT_I2S_LJ	(1 << 0)
#define		I2S1_SDP_SEL_I2S1_FORMAT_PCM_A	(2 << 0)
#define		I2S1_SDP_SEL_I2S1_FORMAT_PCM_B	(3 << 0)
#define	RT5640_ADDA_CLK1	0x73
#define		ADDA_CLK1_SEL_I2S_PRE_DIV1_1	(0 << 12)
#define		ADDA_CLK1_SEL_I2S_PRE_DIV2_1	(0 << 8)
#define		ADDA_CLK1_SEL_I2S_PRE_DIV2_2	(1 << 8)
#define		ADDA_CLK1_SEL_DAC_OCR_128	(0 << 2)
#define		ADDA_CLK1_SEL_DAC_OCR_64	(1 << 2)
#define		ADDA_CLK1_SEL_DAC_OCR_32	(2 << 2)
#define		ADDA_CLK1_SEL_DAC_OCR_16	(3 << 2)
#define		ADDA_CLK1_SEL_ADC_OCR_128	(0 << 0)
#define		ADDA_CLK1_SEL_ADC_OCR_64	(1 << 0)
#define		ADDA_CLK1_SEL_ADC_OCR_32	(2 << 0)
#define		ADDA_CLK1_SEL_ADC_OCR_16	(3 << 0)
#define	RT5640_GLB_CLK		0x80
#define	RT5640_DEPOP_M1		0x8e
#define		DEPOP_M1_POW_PUMP_HP	(1 << 3)
#define		DEPOP_M1_EN_SOFTGEN_HP	(1 << 2)
#define		DEPOP_M1_POW_CAPLESS	(1 << 0)
#define		DEPOP_M1_POWER_MASK	(0xf)
#define	RT5640_DEPOP_M2		0x8f
#define		DEPOP_M2_HP_MODE_1	(0 << 13)
#define		DEPOP_M2_HP_MODE_2	(1 << 13)
#define		DEPOP_M2_HP_MODE_MASK	(1 << 13)
#define		DEPOP_M2_EN_DEPOP_MODE1	(1 << 6)
#define	RT5640_CHARGE_PUMP	0x91
#define		CHARGE_PUMP_MODE_MASK	(3 << 8)
#define		CHARGE_PUMP_MODE_LOW	(0 << 8)
#define		CHARGE_PUMP_MODE_MIDDLE	(1 << 8)
#define		CHARGE_PUMP_MODE_HIGH	(2 << 8)
#define	RT5640_GCTRL1		0xfa
#define		GCTRL1_EN_IN1_SE	(1 << 9)
#define		GCTRL1_EN_IN2_SE	(1 << 8)
#define		GCTRL1_DIGITAL_GATE_CTRL	(1 << 0)
#define	RT5640_VENDOR_ID2	0xff

#define	MAX_BUFFER	16

#define	RT5640_MIXER_DEVS ((1 << SOUND_MIXER_VOLUME) | \
	(1 << SOUND_MIXER_PHONEOUT))

struct rt5640_softc {
	device_t	dev;
	device_t	busdev;
	struct intr_config_hook
			init_hook;
	clk_t		clk;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"realtek,rt5640", 1},
	{NULL,           0},
};
#endif

static void	rt5640_init(void *arg);
static int	rt5640_probe(device_t dev);
static int	rt5640_attach(device_t dev);
static int	rt5640_detach(device_t dev);

static int	rt5640_writeto(device_t slavedev, uint8_t regaddr,
		    void *buffer, uint16_t buflen, int waithow);

static int
rt5640_writeto(device_t slavedev, uint8_t regaddr, void *buffer,
    uint16_t buflen, int waithow)
{
	struct iic_msg	msgs;
	uint8_t		slaveaddr;
	uint8_t		newbuf[MAX_BUFFER];

	if (buflen > MAX_BUFFER - 1)
		return (EINVAL);

	slaveaddr = iicbus_get_addr(slavedev);

	newbuf[0] = regaddr;
	memcpy(newbuf + 1, buffer, buflen);
	msgs.slave = slaveaddr;
	msgs.flags = IIC_M_WR;
	msgs.len   = 1 + buflen;
	msgs.buf   = newbuf;

	return (iicbus_transfer_excl(slavedev, &msgs, 1, waithow));
}

static inline int
rt5640_read2(struct rt5640_softc *sc, uint8_t reg, uint16_t *data)
{
	int res;
	res = iicdev_readfrom(sc->dev, reg, data, 2, IIC_WAIT);
	if (res == 0)
		*data = be16toh(*data);
	return (res);
}

static inline int
rt5640_write2(struct rt5640_softc *sc, uint8_t reg, uint16_t val)
{
	val = htobe16(val);

	return (rt5640_writeto(sc->dev, reg, &val, 2, IIC_WAIT));
}

static inline int
rt5640_pr_read2(struct rt5640_softc *sc, uint8_t reg, uint16_t *data)
{
	int res;
	res = rt5640_write2(sc, RT5640_PR_INDEX, reg);
	if (res != 0)
		return (res);
	res = rt5640_read2(sc, RT5640_PR_DATA, data);
	return res;
}

static inline int
rt5640_pr_write2(struct rt5640_softc *sc, uint8_t reg, uint16_t val)
{
	int res;
	res = rt5640_write2(sc, RT5640_PR_INDEX, reg);
	if (res != 0)
		return (res);
	res = rt5640_write2(sc, RT5640_PR_DATA, val);
	return res;
}

static void
rt5640_powerup(struct rt5640_softc *sc)
{
	uint16_t reg;

        rt5640_pr_read2(sc, 0x24, &reg);
	reg &= ~0x0700;
	reg |= 0x0200;
        rt5640_pr_write2(sc, 0x24, reg);

	rt5640_read2(sc, RT5640_DEPOP_M2, &reg);
	reg |= DEPOP_M2_HP_MODE_2;
	rt5640_write2(sc, RT5640_DEPOP_M2, reg);

	rt5640_read2(sc, RT5640_DEPOP_M1, &reg);
	reg &= ~(DEPOP_M1_POWER_MASK);
	reg |= DEPOP_M1_POW_CAPLESS | DEPOP_M1_POW_PUMP_HP;
	rt5640_write2(sc, RT5640_DEPOP_M1, reg);

        rt5640_pr_write2(sc, 0x77, 0x9f00);

	/* VREF1/VREF2 are slow */
	rt5640_read2(sc, RT5640_PWR_ANLG1, &reg);
	reg &= ~(PWR_ANLG1_EN_FASTB1 | PWR_ANLG1_EN_FASTB2);
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	/* Improve HP Amp driving */
	reg |= PWR_ANLG1_EN_AMP_HP;
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	pause("pwrup", hz/100);

	reg |= (PWR_ANLG1_EN_FASTB1 | PWR_ANLG1_EN_FASTB2);
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	/* depop */
	rt5640_read2(sc, RT5640_DEPOP_M2, &reg);
	reg &= ~DEPOP_M2_HP_MODE_MASK;
	reg |= DEPOP_M2_HP_MODE_1;
	reg |= DEPOP_M2_EN_DEPOP_MODE1;
	rt5640_write2(sc, RT5640_DEPOP_M2, reg);

	rt5640_read2(sc, RT5640_CHARGE_PUMP, &reg);
	reg &= ~CHARGE_PUMP_MODE_MASK;
	reg |= CHARGE_PUMP_MODE_HIGH;
	rt5640_write2(sc, RT5640_CHARGE_PUMP, reg);

        rt5640_pr_write2(sc, 0x37, 0x1c00);

	rt5640_read2(sc, RT5640_DEPOP_M1, &reg);
	reg &= ~DEPOP_M1_POW_PUMP_HP;
	reg |= DEPOP_M1_EN_SOFTGEN_HP;
	rt5640_write2(sc, RT5640_DEPOP_M1, reg);

        rt5640_pr_read2(sc, 0x24, &reg);
	reg &= ~0x0700;
	reg |= 0x0400;
        rt5640_pr_write2(sc, 0x24, reg);
}

static void
rt5640_set_hpo_vol(struct rt5640_softc *sc, unsigned int left, unsigned int right)
{
	uint16_t reg;
	unsigned int l, r;

	if (left > 100)
		left = 100;
	if (right > 100)
		right = 100;

	l = MAX_HPO_VOLUME - (left * MAX_HPO_VOLUME / 100);
	r = MAX_HPO_VOLUME - (right * MAX_HPO_VOLUME / 100);

	rt5640_read2(sc, RT5640_HP_VOL, &reg);
	reg &= ~(HP_VOL_VOLUME_MASK);
	reg |= HP_VOL_VOLUME(l, r);
	rt5640_write2(sc, RT5640_HP_VOL, reg);
}

static void
rt5640_set_if1_vol(struct rt5640_softc *sc, unsigned int left, unsigned int right)
{
	uint16_t reg;
	unsigned int l, r;

	if (left > 100)
		left = 100;
	if (right > 100)
		right = 100;

	l = (left * MAX_DIG_VOLUME / 100);
	r = (right * MAX_DIG_VOLUME / 100);

	rt5640_read2(sc, RT5640_DAC1_DIG_VOL, &reg);
	reg &= ~(DAC1_DIG_VOL_MASK);
	reg |= DAC1_DIG_VOL(l, r);
	rt5640_write2(sc, RT5640_DAC1_DIG_VOL, reg);
}

static void
rt5640_setup_microphone(struct rt5640_softc *sc)
{
	uint16_t reg;

	/* Power up MICBIAS1 */
	rt5640_read2(sc, RT5640_PWR_ANLG2, &reg);
	reg |= PWR_ANLG2_POW_MICBIAS1;
	rt5640_write2(sc, RT5640_PWR_ANLG2, reg);

	/* Enable power for ADCL and ADCR */
	rt5640_read2(sc, RT5640_PWR_DIG1, &reg);
	reg |= (PWR_DIG1_POWER_ADC_L | PWR_DIG1_POWER_ADC_R);
	rt5640_write2(sc, RT5640_PWR_DIG1, reg);

	// MX-0D[7] - set differential mode
	rt5640_read2(sc, RT5640_IN1_IN2, &reg);
	reg |= IN1_IN2_EN_IN1_DF;
	rt5640_write2(sc, RT5640_IN1_IN2, reg);

	// MX-0D[15:12] - MICBST1 volume

	// MX-64[15] - MICBST1 power
	rt5640_read2(sc, RT5640_PWR_ANLG2, &reg);
	reg |= PWR_ANLG2_POW_BST1;
	rt5640_write2(sc, RT5640_PWR_ANLG2, reg);

	// MX-3C[1] - MICBST1 input in RECMIXL
	rt5640_read2(sc, RT5640_REC_L2_MIXER, &reg);
	reg &= ~(L2_MIXER_MU_BST1_RECMIXL);
	rt5640_write2(sc, RT5640_REC_L2_MIXER, reg);

	// MX-3E[1] - MICBST1 input in RECMIXR
	rt5640_read2(sc, RT5640_REC_R2_MIXER, &reg);
	reg &= ~(R2_MIXER_MU_BST1_RECMIXR);
	rt5640_write2(sc, RT5640_REC_R2_MIXER, reg);

	// MX-65[10/11] - RECMIXL/RECMIXR power
	rt5640_read2(sc, RT5640_PWR_MIXER, &reg);
	reg |= (PWR_MIXER_POW_RECMIXL | PWR_MIXER_POW_RECMIXR);
	rt5640_write2(sc, RT5640_PWR_MIXER, reg);

	// internals
	rt5640_read2(sc, RT5640_PWR_DIG2, &reg);
	reg |= PWR_DIG2_POW_ADC_STEREO_FILTER;
	rt5640_write2(sc, RT5640_PWR_DIG2, reg);
	// MX-27[12] to select ADCR
	// MX-27[14] to unmute
	// MX-27[6] to unmute
	rt5640_read2(sc, RT5640_STO_ADC_MIXER, &reg);
	reg |= STO_ADC_MIXER_SEL_STEREO_ADC1;
	reg &= ~(STO_ADC_MIXER_MU_STEREO_ADCL1 | STO_ADC_MIXER_MU_STEREO_ADCR1);
	rt5640_write2(sc, RT5640_STO_ADC_MIXER, reg);

	// MX-1C[15] to unmute
	// MX-1C[14:8] - volume
	// MX-1C[6:0]  - volume
	rt5640_read2(sc, RT5640_ADC_DIG_VOL, &reg);
	reg &= ~(ADC_DIG_VOL_MU_ADC_VOL_L | ADC_DIG_VOL_MU_ADC_VOL_R);
	reg &= ~(ADC_DIG_VOL_AD_GAIN_R_MASK | ADC_DIG_VOL_MU_ADC_VOL_R);
	reg |= ADC_DIG_VOL_AD_GAIN_L(0x7f);
	reg |= ADC_DIG_VOL_AD_GAIN_R(0x7f);
	rt5640_write2(sc, RT5640_ADC_DIG_VOL, reg);
}

static void
rt5640_init(void *arg)
{
	struct rt5640_softc *sc;
	uint16_t reg;

	sc = (struct rt5640_softc*)arg;
	config_intrhook_disestablish(&sc->init_hook);

	rt5640_read2(sc, RT5640_VENDOR_ID2, &reg);

	/* Reset codec */
	rt5640_write2(sc, RT5640_RESET, 0);

        rt5640_pr_write2(sc, 0x3d, 0x3600);
        rt5640_pr_write2(sc, 0x12, 0x0aa8);
        rt5640_pr_write2(sc, 0x14, 0x0aaa);
        rt5640_pr_write2(sc, 0x20, 0x6110);
        rt5640_pr_write2(sc, 0x21, 0xe0e0);
        rt5640_pr_write2(sc, 0x23, 0x1804);

	rt5640_read2(sc, RT5640_DSP_PATH2, &reg);
	reg &= ~0xfc00;
	reg |= 0x0c00;
	rt5640_write2(sc, RT5640_DSP_PATH2, reg);

	rt5640_read2(sc, RT5640_GCTRL1, &reg);
	reg |= GCTRL1_DIGITAL_GATE_CTRL;
	rt5640_write2(sc, RT5640_GCTRL1, reg);

	/* HP L/R amp */
	rt5640_read2(sc, RT5640_PWR_ANLG1, &reg);
	reg |= (PWR_ANLG1_EN_L_HP | PWR_ANLG1_EN_R_HP);
	/* XXX: not clear why we need to enable this for playback */
	reg |= PWR_ANLG1_POW_MAIN_BIAS;
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	rt5640_read2(sc, RT5640_HP_VOL, &reg);
	/* unmute HPO */
	reg &= ~(HP_VOL_MU_HPO_L | HP_VOL_MU_HPO_R);
	/* unmute HPOVOL */
	reg &= ~(HP_VOL_MU_HPOVOLL_IN | HP_VOL_MU_HPOVOLR_IN);
	rt5640_write2(sc, RT5640_HP_VOL, reg);

	rt5640_read2(sc, RT5640_HPO_MIXER, &reg);
	reg &= ~(HPO_MIXER_MU_HPOVOL_HPOMIX);
	rt5640_write2(sc, RT5640_HPO_MIXER, reg);

	/* Enable power for DAC_R1 and DAC_L1 */
	rt5640_read2(sc, RT5640_PWR_DIG1, &reg);
	reg |= (PWR_DIG1_POWER_DAC_L_1 | PWR_DIG1_POWER_DAC_R_1);
	rt5640_write2(sc, RT5640_PWR_DIG1, reg);

	rt5640_read2(sc, RT5640_AD_DA_MIXER, &reg);
	reg &= ~(AD_DA_MIXER_MU_IF1_DAC_L | AD_DA_MIXER_MU_IF1_DAC_R);
	rt5640_write2(sc, RT5640_AD_DA_MIXER, reg);

	/* I2S1 power */
	rt5640_read2(sc, RT5640_PWR_DIG1, &reg);
	reg |= PWR_DIG1_EN_I2S1;
	rt5640_write2(sc, RT5640_PWR_DIG1, reg);

	/* Unmute DAC L1/R1 Switch */
	rt5640_read2(sc, RT5640_DIG_MIXER, &reg);
	reg &= ~(DIG_MIXER_MU_DACL1_TO_DACL | DIG_MIXER_MU_DACR1_TO_DACR);
	rt5640_write2(sc, RT5640_DIG_MIXER, reg);

	/* Power up HPOVOL L/R */
	rt5640_read2(sc, RT5640_PWR_VOL, &reg);
	reg |= (PWR_VOL_POW_HPOVOLL | PWR_VOL_POW_HPOVOLR);
	rt5640_write2(sc, RT5640_PWR_VOL, reg);

	/* Power up OUT MIX L/R */
	rt5640_read2(sc, RT5640_PWR_MIXER, &reg);
	reg |= (PWR_MIXER_POW_OUTMIXL | PWR_MIXER_POW_OUTMIXR);
	rt5640_write2(sc, RT5640_PWR_MIXER, reg);

	/* Unmute OUT MIX R */
	rt5640_read2(sc, RT5640_OUT_R3_MIXER, &reg);
	reg &= ~(OUT_R3_MIXER_MU_DACR1_OUTMIXR);
	rt5640_write2(sc, RT5640_OUT_R3_MIXER, reg);

	/* Unmute OUT MIX L */
	rt5640_read2(sc, RT5640_OUT_L3_MIXER, &reg);
	reg &= ~(OUT_L3_MIXER_MU_DACL1_OUTMIXL);
	rt5640_write2(sc, RT5640_OUT_L3_MIXER, reg);

	/* Stereo DAC MIX L/R */
	rt5640_read2(sc, RT5640_STO_DAC_MIXER, &reg);
	reg &= ~(STO_DAC_MIXER_MU_STEREO_DACL1 | STO_DAC_MIXER_MU_STEREO_DACR1);
	rt5640_write2(sc, RT5640_STO_DAC_MIXER, reg);

	// PLL1
	rt5640_read2(sc, RT5640_PWR_ANLG2, &reg);
	reg |= PWR_ANLG2_POW_PLL;
	rt5640_write2(sc, RT5640_PWR_ANLG2, reg);

	reg = ADDA_CLK1_SEL_I2S_PRE_DIV1_1 |
		ADDA_CLK1_SEL_I2S_PRE_DIV2_2 |
		ADDA_CLK1_SEL_DAC_OCR_64 |
		ADDA_CLK1_SEL_ADC_OCR_128;
	rt5640_write2(sc, RT5640_ADDA_CLK1, reg);

	rt5640_setup_microphone(sc);

	rt5640_powerup(sc);
}

static int
rt5640_mixer_init(struct snd_mixer *m)
{
	mix_setdevs(m, RT5640_MIXER_DEVS);

	return (0);
}

static int
rt5640_mixer_uninit(struct snd_mixer *m)
{

	return (0);
}

static int
rt5640_mixer_reinit(struct snd_mixer *m)
{

	return (0);
}

static int
rt5640_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct rt5640_softc *sc;
	struct mtx *mixer_lock;
	int locked;

	sc = device_get_softc(mix_getdevinfo(m));
	mixer_lock = mixer_get_lock(m);
	locked = mtx_owned(mixer_lock);

	/*
	 * We need to unlock the mixer lock because iicbus_transfer()
	 * may sleep. The mixer lock itself is unnecessary here
	 * because it is meant to serialize hardware access, which
	 * is taken care of by the I2C layer, so this is safe.
	 */
	if (locked)
		mtx_unlock(mixer_lock);
	switch (dev) {
	case SOUND_MIXER_VOLUME:
		rt5640_set_if1_vol(sc, left, right);
		return (left | (right << 8));
	case SOUND_MIXER_PHONEOUT:
		rt5640_set_hpo_vol(sc, left, right);
		return (left | (right << 8));
	default:
		break;
	}

	if (locked)
		mtx_lock(mixer_lock);

	return (0);
}

static u_int32_t
rt5640_mixer_setrecsrc(struct snd_mixer *m, u_int32_t src)
{

	return (0);
}


static kobj_method_t rt5640_mixer_methods[] = {
	KOBJMETHOD(mixer_init, 		rt5640_mixer_init),
	KOBJMETHOD(mixer_uninit, 	rt5640_mixer_uninit),
	KOBJMETHOD(mixer_reinit, 	rt5640_mixer_reinit),
	KOBJMETHOD(mixer_set, 		rt5640_mixer_set),
	KOBJMETHOD(mixer_setrecsrc, 	rt5640_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(rt5640_mixer);

static int
rt5640_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Realtek RT5640");
		return (BUS_PROBE_DEFAULT);
	}
#endif
	return (ENXIO);
}

static int
rt5640_attach(device_t dev)
{
	struct rt5640_softc	*sc;
	int error;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->busdev = device_get_parent(sc->dev);

	error = clk_get_by_ofw_name(dev, 0, "mclk", &sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot get mclk clock\n");
		return (ENXIO);
	}
	clk_set_freq(sc->clk, 12288000, 0);

	clk_enable(sc->clk);

	/*
	 * Wait until i2c is ready to set up the chip
	 */
	sc->init_hook.ich_func = rt5640_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);
}

static int
rt5640_detach(device_t dev)
{

	return (0);
}

static int
rt5640_dai_init(device_t dev, uint32_t format)
{
	struct rt5640_softc* sc;
	int fmt, pol, clk;
	uint16_t reg;

	sc = (struct rt5640_softc*)device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	/* I2S1 setup */
	rt5640_read2(sc, RT5640_I2S1_SDP, &reg);

	/* channel mapping */
	reg &= ~(0x7 << 12);

	switch (pol) {
	case AUDIO_DAI_POLARITY_IB_NF:
		reg |= I2S1_SDP_INV_I2S1_BCLK;
		break;
	case AUDIO_DAI_POLARITY_NB_NF:
		reg &= ~I2S1_SDP_INV_I2S1_BCLK;
		break;
	default:
		return (EINVAL);
	}

	switch (clk) {
	case AUDIO_DAI_CLOCK_CBM_CFM:
		reg &= ~I2S1_SDP_SEL_I2S1_MS_SLAVE;
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		reg |= I2S1_SDP_SEL_I2S1_MS_SLAVE;
		break;
	default:
		return (EINVAL);
	}

	/* 16 bit samples */
	reg &= ~I2S1_SDP_SEL_I2S1_LEN_MASK;
	/* set format */
	reg &= ~I2S1_SDP_SEL_I2S1_FORMAT_MASK;
	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		reg |= I2S1_SDP_SEL_I2S1_FORMAT_I2S;
		break;
	case AUDIO_DAI_FORMAT_LJ:
		reg |= I2S1_SDP_SEL_I2S1_FORMAT_I2S_LJ;
		break;
	case AUDIO_DAI_FORMAT_DSPA:
		reg |= I2S1_SDP_SEL_I2S1_FORMAT_PCM_A;
		break;
	case AUDIO_DAI_FORMAT_DSPB:
		reg |= I2S1_SDP_SEL_I2S1_FORMAT_PCM_B;
		break;
	default:
		return EINVAL;
	}

	rt5640_write2(sc, RT5640_I2S1_SDP, reg);

	return (0);
}

static int
rt5640_dai_setup_mixer(device_t dev, device_t pcmdev)
{

	mixer_init(pcmdev, &rt5640_mixer_class, dev);

	return (0);
}

static int
rt5640_dai_trigger(device_t dev, int go, int pcm_dir)
{

	switch (go) {
	case PCMTRIG_START:
		/* TODO: power up amps */
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		/* TODO: power down amps */
		break;
	}

	return (0);
}

static device_method_t rt5640_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		rt5640_probe),
	DEVMETHOD(device_attach,	rt5640_attach),
	DEVMETHOD(device_detach,	rt5640_detach),

	DEVMETHOD(audio_dai_init,	rt5640_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	rt5640_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	rt5640_dai_trigger),

	DEVMETHOD_END,
};

static driver_t rt5640_driver = {
	"rt5640codec",
	rt5640_methods,
	sizeof(struct rt5640_softc),
};
static devclass_t rt5640_devclass;

DRIVER_MODULE(rt5640codec, iicbus, rt5640_driver, rt5640_devclass, NULL, NULL);
MODULE_VERSION(rt5640codec, 1);
MODULE_DEPEND(rt5640codec, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
