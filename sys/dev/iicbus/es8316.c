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
 * Driver for Realtek es8316 audio codec
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

#define	MAX_BUFFER	16

#define	ESCODEC_RESET_REG		0x00
#define	 RESET_ALL				0x3f
#define	 RESET_CSM_ON				(1 << 7)
#define	ESCODEC_CLKMAN1_REG		0x01
#define	 CLKMAN1_MCLK_ON			(1 << 6)
#define	 CLKMAN1_BCLK_ON			(1 << 5)
#define	 CLKMAN1_CLK_CP_ON			(1 << 4)
#define	 CLKMAN1_CLK_ADC_ON			(1 << 3)
#define	 CLKMAN1_CLK_DAC_ON			(1 << 2)
#define	 CLKMAN1_ANACLK_ADC_ON			(1 << 1)
#define	 CLKMAN1_ANACLK_DAC_ON			(1 << 0)
#define	ESCODEC_ADC_OSR_REG		0x03
#define	ESCODEC_SD_CLK_REG		0x09
#define	 SD_CLK_MSC				(1 << 7)
#define	 SD_CLK_BCLK_INV			(1 << 5)
#define	ESCODEC_SD_ADC_REG		0x0a
#define	ESCODEC_SD_DAC_REG		0x0b
#define	 SD_FMT_LRP				(1 << 5)
#define	 SD_FMT_WL_MASK				(7 << 2)
#define	 SD_FMT_WL_SHIFT			2
#define	  SD_FMT_WL_16				3
#define	 SD_FMT_MASK				(3 << 0)
#define	 SD_FMT_SHIFT				0
#define	  SD_FMT_I2S				0
#define	ESCODEC_VMID_REG		0x0c
#define	ESCODEC_PDN_REG			0x0d
#define	ESCODEC_LP1_REG			0x0e
#define	ESCODEC_LP2_REG			0x0f
#define	ESCODEC_HPSEL_REG		0x13
#define	ESCODEC_HPMIXRT_REG		0x14
#define	 HPMIXRT_LD2LHPMIX			(1 << 7)
#define	 HPMIXRT_RD2RHPMIX			(1 << 3)
#define	ESCODEC_HPMIX_REG		0x15
#define	 HPMIX_LHPMIX_MUTE			(1 << 5)
#define	 HPMIX_PDN_LHP_MIX			(1 << 4)
#define	 HPMIX_RHPMIX_MUTE			(1 << 1)
#define	 HPMIX_PDN_RHP_MIX			(1 << 0)
#define	ESCODEC_HPMIXVOL_REG		0x16
#define	 HPMIXVOL_LHPMIXVOL_MASK		(0xf << 4)
#define	 HPMIXVOL_LHPMIXVOL(v)			(((v) & 0xf) << 4)
#define	 HPMIXVOL_RHPMIXVOL_MASK		(0xf << 0)
#define	 HPMIXVOL_RHPMIXVOL(v)			(((v) & 0xf) << 0)
#define	ESCODEC_HPOUTEN_REG		0x17
#define	 HPOUTEN_EN_HPL				(1 << 6)
#define	 HPOUTEN_HPL_OUTEN			(1 << 5)
#define	 HPOUTEN_EN_HPR				(1 << 2)
#define	 HPOUTEN_HPR_OUTEN			(1 << 1)
#define	ESCODEC_HPVOL_REG		0x18
#define	 HPVOL_PDN_LICAL			(1 << 7)
#define	 HPVOL_HPLVOL				__BITS(5,4)
#define	 HPVOL_PDN_RICAL			(1 << 3)
#define	 HPVOL_HPRVOL				__BITS(1,0)
#define	ESCODEC_HPPWR_REG		0x19
#define	 HPPWR_PDN_CPHP				(1 << 2)
#define	ESCODEC_CPPWR_REG		0x1a
#define	 CPPWR_PDN_CP				(1 << 5)
#define	 CPPWR_CI_HIPWR				(1 << 4)
#define	ESCODEC_LDOCTL_REG		0x1b
#define	  LDOCTL_LDOLVL_1_45			(3 << 4)
#define	ESCODEC_ADCSEL_REG		0x22
#define	  ADCSEL_PDN_AINL			(1 << 7)
#define	  ADCSEL_PDN_MODL			(1 << 6)
#define	  ADCSEL_LINSEL_MASK			(3 << 4)
#define	  ADCSEL_LINSEL_LIN1			(0 << 4)
#define	  ADCSEL_LINSEL_LIN2			(1 << 4)
#define	ESCODEC_ADC_VOLUME		0x27
#define	 ADC_VOLUME_ADCVOLUMEL_MAX		0xc0
#define	ESCODEC_DACPWR_REG		0x2f
#define	 DACPWR_PDN_DAC_L			(1 << 4)
#define	 DACPWR_PDN_DAC_R			(1 << 0)
#define	ESCODEC_DACCTL1_REG		0x30
#define	 DACCTL1_MUTE				(1 << 5)
#define	ESCODEC_DACVOL_L_REG		0x33
#define	 DACVOL_L_DACVOLUME_MASK		0xff
#define	 DACVOL_L_DACVOLUME_MAX			0xc0
#define	ESCODEC_DACVOL_R_REG		0x34
#define	 DACVOL_R_DACVOLUME_MASK		0xff
#define	 DACVOL_R_DACVOLUME_MAX			0xc0

#define	ES8316_MIXER_DEVS (SOUND_MASK_VOLUME | SOUND_MASK_MIC)

#define	ADC_SRC_LIN1_RIN1	1
#define	ADC_SRC_LIN2_RIN2	2

struct es8316_softc {
	device_t	dev;
	device_t	busdev;
	struct intr_config_hook
			init_hook;
	clk_t		clk;
	int		adc_src;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"everest,es8316", 1},
	{NULL,           0},
};
#endif

static void	es8316_init(void *arg);
static int	es8316_probe(device_t dev);
static int	es8316_attach(device_t dev);
static int	es8316_detach(device_t dev);

static inline int
es8316_read(struct es8316_softc *sc, uint8_t reg, uint8_t *data)
{
	int res;
	res = iicdev_readfrom(sc->dev, reg, data, 1, IIC_WAIT);
	if (res != 0)
		*data = 0xff;
	return (res);
}

static inline int
es8316_write(struct es8316_softc *sc, uint8_t reg, uint8_t val)
{

	return (iicdev_writeto(sc->dev, reg, &val, 1, IIC_WAIT));
}

static void
es8316_init(void *arg)
{
	struct es8316_softc *sc;

	sc = (struct es8316_softc*)arg;
	config_intrhook_disestablish(&sc->init_hook);
}

static int
es8316_mixer_init(struct snd_mixer *m)
{
	mix_setdevs(m, ES8316_MIXER_DEVS);

	return (0);
}

static int
es8316_mixer_uninit(struct snd_mixer *m)
{

	return (0);
}

static int
es8316_mixer_reinit(struct snd_mixer *m)
{

	return (0);
}

static int
es8316_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct es8316_softc *sc;
	struct mtx *mixer_lock;
	int locked;
	uint8_t val;

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
		val = DACVOL_L_DACVOLUME_MAX - (left * DACVOL_L_DACVOLUME_MAX / 100);
		es8316_write(sc, ESCODEC_DACVOL_L_REG, val);
		val = DACVOL_R_DACVOLUME_MAX - (right * DACVOL_R_DACVOLUME_MAX / 100);
		es8316_write(sc, ESCODEC_DACVOL_R_REG, val);
		return (left | (right << 8));
	case SOUND_MIXER_MIC:
		/*
		 * Force 100% for now. Without boost microphone level declines prety rapidly
		 * and at 85% it's mostly silence. For demonstration purposes just keep at 100%
		 * for now.
		 */
		left = right = 100;
		val = ADC_VOLUME_ADCVOLUMEL_MAX - (left * ADC_VOLUME_ADCVOLUMEL_MAX / 100);
		es8316_write(sc, ESCODEC_ADC_VOLUME, val);
		return (left | (right << 8));
	default:
		break;
	}

	if (locked)
		mtx_lock(mixer_lock);

	return (0);
}

static u_int32_t
es8316_mixer_setrecsrc(struct snd_mixer *m, u_int32_t src)
{

	return (0);
}

static kobj_method_t es8316_mixer_methods[] = {
	KOBJMETHOD(mixer_init, 		es8316_mixer_init),
	KOBJMETHOD(mixer_uninit, 	es8316_mixer_uninit),
	KOBJMETHOD(mixer_reinit, 	es8316_mixer_reinit),
	KOBJMETHOD(mixer_set, 		es8316_mixer_set),
	KOBJMETHOD(mixer_setrecsrc, 	es8316_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(es8316_mixer);

static int
es8316_set_rec_src(struct es8316_softc *sc, int lin)
{
	uint8_t val;

	es8316_read(sc, ESCODEC_ADCSEL_REG, &val);
	val &= ~(ADCSEL_LINSEL_MASK);
	if (lin == ADC_SRC_LIN1_RIN1)
		val |= ADCSEL_LINSEL_LIN1;
	else if (lin == ADC_SRC_LIN2_RIN2)
		val |= ADCSEL_LINSEL_LIN2;
	else
		return (EINVAL);

	sc->adc_src = lin;
	es8316_write(sc, ESCODEC_ADCSEL_REG, val);

	return (0);
}

static int
es8316_sysctl_audio_src(SYSCTL_HANDLER_ARGS)
{
	struct es8316_softc *sc = arg1;
	int val;
	int err;

	val = sc->adc_src;
	err = sysctl_handle_int(oidp, &val, 0, req);
	if (err || !req->newptr) /* error || read request */
		return (err);

	return es8316_set_rec_src(sc, val);
}

static int
es8316_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Everest Semi ES8316 CODEC");
		return (BUS_PROBE_DEFAULT);
	}
#endif
	return (ENXIO);
}

static int
es8316_attach(device_t dev)
{
	struct es8316_softc	*sc;
	int error;
	phandle_t node;
	uint8_t val;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree_node;
	struct sysctl_oid_list *tree;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->busdev = device_get_parent(sc->dev);

	error = clk_get_by_ofw_name(dev, 0, "mclk", &sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot get mclk clock\n");
		return (ENXIO);
	}

	clk_enable(sc->clk);

	/*
	 * Wait until i2c is ready to set up the chip
	 */
	sc->init_hook.ich_func = es8316_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	es8316_write(sc, ESCODEC_RESET_REG, RESET_ALL);
	DELAY(5);
	es8316_write(sc, ESCODEC_RESET_REG, RESET_CSM_ON);
	DELAY(30);

	es8316_write(sc, ESCODEC_VMID_REG, 0xff);
	es8316_write(sc, ESCODEC_ADC_OSR_REG, 0x32);

	es8316_read(sc, ESCODEC_SD_ADC_REG, &val);
	val &= ~SD_FMT_WL_MASK;
	val |= (SD_FMT_WL_16 << SD_FMT_WL_SHIFT);
	es8316_write(sc, ESCODEC_SD_ADC_REG, val);

	es8316_read(sc, ESCODEC_SD_DAC_REG, &val);
	val &= ~SD_FMT_WL_MASK;
	val |= (SD_FMT_WL_16 << SD_FMT_WL_SHIFT);
	es8316_write(sc, ESCODEC_SD_DAC_REG, val);

	/* Power up */
	es8316_write(sc, ESCODEC_PDN_REG, 0);

	/* Route DAC signal to HP mixer */
	val = HPMIXRT_LD2LHPMIX | HPMIXRT_RD2RHPMIX;
	es8316_write(sc, ESCODEC_HPMIXRT_REG, val);

	/* Power up DAC */
	es8316_write(sc, ESCODEC_DACPWR_REG, 0);

	/* Power up HP mixer and unmute */
	es8316_write(sc, ESCODEC_HPMIX_REG, (1 << 3) | (1<<7));

	/* Power up HP output driver */
	es8316_read(sc, ESCODEC_HPPWR_REG, &val);
	val &= ~HPPWR_PDN_CPHP;
	es8316_write(sc, ESCODEC_HPPWR_REG, val);

	/* Power up HP charge pump circuits */
	es8316_read(sc, ESCODEC_CPPWR_REG, &val);
	val &= ~CPPWR_PDN_CP;
	val |= CPPWR_CI_HIPWR;
	es8316_write(sc, ESCODEC_CPPWR_REG, val);

	/* Power up CP negative regulator */
	es8316_write(sc, ESCODEC_LDOCTL_REG, LDOCTL_LDOLVL_1_45);

	/* Enable low power mode */
	es8316_write(sc, ESCODEC_LP1_REG, 0x3f);
	es8316_write(sc, ESCODEC_LP2_REG, 0x1f);

	/* Set LIN1/RIN1 as inputs for HP mixer */
	es8316_write(sc, ESCODEC_HPSEL_REG, 0);

	/* Set LIN2/RIN2 as inputs for ADC and power up ADC/PGA */
	es8316_read(sc, ESCODEC_ADCSEL_REG, &val);
	val &= ~(ADCSEL_PDN_AINL | ADCSEL_PDN_MODL);
	es8316_write(sc, ESCODEC_ADCSEL_REG, val);

	es8316_set_rec_src(sc, ADC_SRC_LIN2_RIN2);

	/* Power up HP output driver calibration */
	es8316_read(sc, ESCODEC_HPVOL_REG, &val);
	val &= ~HPVOL_PDN_LICAL;
	val &= ~HPVOL_PDN_RICAL;
	es8316_write(sc, ESCODEC_HPVOL_REG, val);

	/* Set headphone mixer to 0dB */
	es8316_write(sc, ESCODEC_HPMIXVOL_REG, 0xbb);

	/* Set charge pump headphone to 0dB */
	es8316_write(sc, ESCODEC_HPVOL_REG, 0x00);

	/* Set DAC to 0dB */
	es8316_write(sc, ESCODEC_DACVOL_L_REG, 0);
	es8316_write(sc, ESCODEC_DACVOL_R_REG, 0);

	/* Enable HP output */
	val = HPOUTEN_EN_HPL | HPOUTEN_EN_HPR |
	    HPOUTEN_HPL_OUTEN | HPOUTEN_HPR_OUTEN;
	es8316_write(sc, ESCODEC_HPOUTEN_REG, val);

	/*
	 * Add system sysctl tree/handlers.
	 */
	ctx = device_get_sysctl_ctx(sc->dev);
	tree_node = device_get_sysctl_tree(sc->dev);
	tree = SYSCTL_CHILDREN(tree_node);
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "recsrc",
	    CTLFLAG_RW | CTLTYPE_UINT | CTLFLAG_NEEDGIANT, sc, sizeof(*sc),
	    es8316_sysctl_audio_src, "IU", "ADC source, "
	    "1 - LIN1/RIN1, 2 - LIN2/RIN2");

	return (0);
}

static int
es8316_detach(device_t dev)
{

	return (0);
}

static int
es8316_dai_init(device_t dev, uint32_t format)
{
	struct es8316_softc* sc;
	uint8_t sd_clk, sd_fmt, val;
	int fmt, pol, clk;

	sc = (struct es8316_softc*)device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	if (fmt != AUDIO_DAI_FORMAT_I2S)
		return EINVAL;

	if (clk != AUDIO_DAI_CLOCK_CBS_CFS)
		return EINVAL;

	switch (pol) {
	case AUDIO_DAI_POLARITY_NB_NF:
		sd_clk = 0;
		sd_fmt = 0;
		break;
	case AUDIO_DAI_POLARITY_NB_IF:
		sd_clk = 0;
		sd_fmt = SD_FMT_LRP;
		break;
	case AUDIO_DAI_POLARITY_IB_NF:
		sd_clk = SD_CLK_BCLK_INV;
		sd_fmt = 0;
		break;
	case AUDIO_DAI_POLARITY_IB_IF:
		sd_clk = SD_CLK_BCLK_INV;
		sd_fmt = SD_FMT_LRP;
		break;
	}

	es8316_read(sc, ESCODEC_SD_CLK_REG, &val);
	val &= ~(SD_CLK_MSC|SD_CLK_BCLK_INV);
	val |= sd_clk;
	es8316_write(sc, ESCODEC_SD_CLK_REG, val);

	es8316_read(sc, ESCODEC_SD_ADC_REG, &val);
	val &= ~SD_FMT_MASK;
	val |= (SD_FMT_I2S << SD_FMT_SHIFT);
	val &= ~SD_FMT_LRP;
	val |= sd_fmt;
	es8316_write(sc, ESCODEC_SD_ADC_REG, val);

	es8316_read(sc, ESCODEC_SD_DAC_REG, &val);
	val &= ~SD_FMT_MASK;
	val |= (SD_FMT_I2S << SD_FMT_SHIFT);
	val &= ~SD_FMT_LRP;
	val |= sd_fmt;
	es8316_write(sc, ESCODEC_SD_DAC_REG, val);

	es8316_read(sc, ESCODEC_CLKMAN1_REG, &val);
	val |= CLKMAN1_MCLK_ON;
	val |= CLKMAN1_BCLK_ON;
	val |= CLKMAN1_CLK_CP_ON;
	val |= CLKMAN1_CLK_DAC_ON;
	val |= CLKMAN1_ANACLK_DAC_ON;
	val |= CLKMAN1_CLK_ADC_ON;
	val |= CLKMAN1_ANACLK_ADC_ON;
	es8316_write(sc, ESCODEC_CLKMAN1_REG, val);

	return (0);
}

static int
es8316_dai_setup_mixer(device_t dev, device_t pcmdev)
{

	mixer_init(pcmdev, &es8316_mixer_class, dev);

	return (0);
}

static int
es8316_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct es8316_softc *sc;
	sc = device_get_softc(dev);

	if ((pcm_dir != PCMDIR_PLAY) && (pcm_dir != PCMDIR_REC))
		return (EINVAL);

	/* TODO: may be move amps/clocks enable/disable here */
	switch (go) {
	case PCMTRIG_START:
		if (pcm_dir == PCMDIR_PLAY) {
		}
		else if (pcm_dir == PCMDIR_REC) {
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (pcm_dir == PCMDIR_PLAY) {
		}
		else if (pcm_dir == PCMDIR_REC) {
		}
		break;
	}

	return (0);
}

static int
es8316_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{
	struct es8316_softc *sc;
	int error;

	sc = device_get_softc(dev);

	error = clk_disable(sc->clk);
	if (error != 0) {
		device_printf(sc->dev, "could not disable mclk clock\n");
		return (error);
	}

	error = clk_set_freq(sc->clk, rate, CLK_SET_ROUND_DOWN);
	if (error != 0)
		device_printf(sc->dev, "could not set mclk freq\n");

	error = clk_enable(sc->clk);
	if (error != 0) {
		device_printf(sc->dev, "could not enable mclk clock\n");
		return (error);
	}

	return (0);
}

static device_method_t es8316_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		es8316_probe),
	DEVMETHOD(device_attach,	es8316_attach),
	DEVMETHOD(device_detach,	es8316_detach),

	DEVMETHOD(audio_dai_init,	es8316_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	es8316_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	es8316_dai_trigger),
	DEVMETHOD(audio_dai_set_sysclk,	es8316_dai_set_sysclk),

	DEVMETHOD_END,
};

static driver_t es8316_driver = {
	"es8316codec",
	es8316_methods,
	sizeof(struct es8316_softc),
};
static devclass_t es8316_devclass;

DRIVER_MODULE(es8316codec, iicbus, es8316_driver, es8316_devclass, NULL, NULL);
MODULE_VERSION(es8316codec, 1);
MODULE_DEPEND(es8316codec, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
