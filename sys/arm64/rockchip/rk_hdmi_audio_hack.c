/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Oleksandr Tymoshenko <gonzo@FreeBSD.org>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

#define	HDMI_FC_AUDICONF0	0x1025
#define	HDMI_FC_AUDICONF1	0x1026
#define	HDMI_FC_AUDICONF2	0x1027
#define	HDMI_FC_AUDICONF3	0x1028
#define	HDMI_FC_AUDSCHNLS7	0x106e
#define	HDMI_FC_AUDSCHNLS8	0x106f
#define	HDMI_AUD_CONF0		0x3100
#define	HDMI_AUD_CONF1		0x3101
#define	HDMI_AUD_INT		0x3102
#define	HDMI_AUD_CONF2		0x3103
#define	HDMI_AUD_INPUTCLKFS	0x3206
#define	HDMI_AUD_N1		0x3200
#define	HDMI_AUD_N2		0x3201
#define	HDMI_AUD_N3		0x3202
#define	HDMI_AUD_CTS1		0x3203
#define	HDMI_AUD_CTS2		0x3204
#define	HDMI_AUD_CTS3		0x3205
#define	HDMI_MC_CLKDIS		0x4001
#define	HDMI_MC_SWRSTZ		0x4002

/*
 * These values are calculated for 1080p@60Hz,
 * TDMS is 148.5MHz and Fs is 48KHz, see table in 33.4.6.1
 * https://people.freebsd.org/~gonzo/arm/iMX6-HDMI.pdf
 */
#define N_1080P_48K		6144
#define CTS_1080P_48K		148500

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-dw-hdmi",	1},
	{ NULL,				0}
};

struct hdmi_audio_softc {
	device_t	dev;
	struct resource	*mem_res;
};

#define	HDMI_AUDIO_READ4(_sc, reg)	bus_read_4((_sc)->mem_res, (reg)*4)
#define	HDMI_AUDIO_WRITE4(_sc, reg, value)	\
    bus_write_4((_sc)->mem_res, (reg)*4, (value))

static int hdmi_audio_probe(device_t dev);
static int hdmi_audio_attach(device_t dev);
static int hdmi_audio_detach(device_t dev);

static int
hdmi_audio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Rockchip HDMI Audio Hack");
	return (BUS_PROBE_DEFAULT);
}

static int
hdmi_audio_attach(device_t dev)
{
	struct hdmi_audio_softc *sc;
	phandle_t node;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(sc->dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);
}

static int
hdmi_audio_detach(device_t dev)
{

	return (0);
}

static void
hdmi_audio_disable_audio_clock(struct hdmi_audio_softc *sc)
{
	uint32_t reg;

	reg = HDMI_AUDIO_READ4(sc, HDMI_MC_CLKDIS);
	reg |= (1 << 3);
	HDMI_AUDIO_WRITE4(sc, HDMI_MC_CLKDIS, reg);
}

static void
hdmi_audio_enable_audio_clock(struct hdmi_audio_softc *sc)
{
	uint32_t reg;

	reg = HDMI_AUDIO_READ4(sc, HDMI_MC_CLKDIS);
	reg &= ~(1 << 3);
	HDMI_AUDIO_WRITE4(sc, HDMI_MC_CLKDIS, reg);
}

static void
hdmi_audio_set_n_cts(struct hdmi_audio_softc *sc, uint32_t n, uint32_t cts)
{
	uint32_t reg;

	reg = HDMI_AUDIO_READ4(sc, HDMI_AUD_CTS3);
	reg &= ~(0x10); /* manual */
	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CTS3, reg);
	reg &= ~(0xe0);
	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CTS3, reg);

	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CTS3, ((cts >> 16) & 0xf) | 0x10);
	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CTS2, (cts >>  8) & 0xff);
	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CTS1, (cts >>  0) & 0xff);

	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_N3, (n >> 16) & 0xf);
	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_N2, (n >>  8) & 0xff);
	HDMI_AUDIO_WRITE4(sc, HDMI_AUD_N1, (n >>  0) & 0xff);
}

static int
hdmi_audio_dai_trigger(device_t dev, int go, int pcm_dir)
{
	uint32_t reg;
	struct hdmi_audio_softc *sc;

	sc = device_get_softc(dev);

	if (pcm_dir != PCMDIR_PLAY)
		return (EINVAL);

	switch (go) {
	case PCMTRIG_START:
		/* Reset I2S subsystem */
		reg = HDMI_AUDIO_READ4(sc, HDMI_AUD_CONF0);
		reg |= 0x80;
		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CONF0, reg);
		reg = HDMI_AUDIO_READ4(sc, HDMI_MC_SWRSTZ);
		reg |= 0x08;
		HDMI_AUDIO_WRITE4(sc, HDMI_MC_SWRSTZ, reg);
	
		hdmi_audio_disable_audio_clock(sc);
	
		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CONF2, 0x04); /* insert PCUV, important!!! */
	
		/* Disable audio */
		hdmi_audio_set_n_cts(sc, 0, CTS_1080P_48K);
	
		reg = 0x20;
		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CONF0, reg); /* select */
		reg |= 0x01;
		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CONF0, reg); /* 2 channels */

		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CONF0, reg); /* 2 channels */
		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_CONF1, 0x10); /* 16 bit word */
	
		HDMI_AUDIO_WRITE4(sc, HDMI_AUD_INPUTCLKFS, 0x00); /* Fs = 128 */
	
		HDMI_AUDIO_WRITE4(sc, HDMI_FC_AUDSCHNLS7, 0x02); /* 48KHz */
		HDMI_AUDIO_WRITE4(sc, HDMI_FC_AUDSCHNLS8, 0xd0); /* 48KHz */
		HDMI_AUDIO_WRITE4(sc, HDMI_FC_AUDICONF0, 0x10); /* 2 channels */
	
		HDMI_AUDIO_WRITE4(sc, HDMI_FC_AUDICONF1, 0x00);
		HDMI_AUDIO_WRITE4(sc, HDMI_FC_AUDICONF2, 0x00);
		HDMI_AUDIO_WRITE4(sc, HDMI_FC_AUDICONF3, 0x00);
	
		hdmi_audio_set_n_cts(sc, N_1080P_48K, CTS_1080P_48K);
		hdmi_audio_enable_audio_clock(sc);
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		hdmi_audio_set_n_cts(sc, 0, CTS_1080P_48K);
		hdmi_audio_disable_audio_clock(sc);
		break;
	default:
		break;
	}

	return (0);
}

static int
hdmi_audio_dai_init(device_t dev, uint32_t format)
{

	return (0);
}

static device_method_t hdmi_audio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		hdmi_audio_probe),
	DEVMETHOD(device_attach,	hdmi_audio_attach),
	DEVMETHOD(device_detach,	hdmi_audio_detach),

	DEVMETHOD(audio_dai_init,	hdmi_audio_dai_init),
	DEVMETHOD(audio_dai_trigger,	hdmi_audio_dai_trigger),

	DEVMETHOD_END
};

static driver_t hdmi_audio_driver = {
	"hdmiaudio",
	hdmi_audio_methods,
	sizeof(struct hdmi_audio_softc),
};

static devclass_t hdmi_audio_devclass;

DRIVER_MODULE(hdmi_audio, simplebus, hdmi_audio_driver, hdmi_audio_devclass, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
