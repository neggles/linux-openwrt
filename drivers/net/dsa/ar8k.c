// SPDX-License-Identifier: GPL-2.0
/*
 * This is a somewhat-heavily-modified version of the qca8k driver, written
 * for the older AR8216/AR8236/AR8136 switch chips, which do not support
 * independent VLAN learning.
 *
 * Copyright (C) 2009 Felix Fietkau <nbd@nbd.name>
 * Copyright (C) 2011-2012 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (c) 2015, 2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2016 John Crispin <john@phrozen.org>
 * Copyright (c) 2022 Andrew Powers-Holmes <aholmes@omnom.net>
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <net/dsa.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/if_bridge.h>
#include <linux/mdio.h>
#include <linux/phylink.h>
#include <linux/gpio/consumer.h>
#include <linux/etherdevice.h>

#include "ar8k.h"

#define MIB_DESC(_s , _o, _n)	\
	{			\
		.size = (_s),	\
		.offset = (_o),	\
		.name = (_n),	\
	}

const struct ar8k_mib_desc ar8k_mib[] = {
	MIB_DESC(1, 0x00, "RxBroad"),
	MIB_DESC(1, 0x04, "RxPause"),
	MIB_DESC(1, 0x08, "RxMulti"),
	MIB_DESC(1, 0x0c, "RxFcsErr"),
	MIB_DESC(1, 0x10, "RxAlignErr"),
	MIB_DESC(1, 0x14, "RxRunt"),
	MIB_DESC(1, 0x18, "RxFragment"),
	MIB_DESC(1, 0x1c, "Rx64Byte"),
	MIB_DESC(1, 0x20, "Rx128Byte"),
	MIB_DESC(1, 0x24, "Rx256Byte"),
	MIB_DESC(1, 0x28, "Rx512Byte"),
	MIB_DESC(1, 0x2c, "Rx1024Byte"),
	MIB_DESC(1, 0x30, "Rx1518Byte"),
	MIB_DESC(1, 0x34, "RxMaxByte"),
	MIB_DESC(1, 0x38, "RxTooLong"),
	MIB_DESC(2, 0x3c, "RxGoodByte"),
	MIB_DESC(2, 0x44, "RxBadByte"),
	MIB_DESC(1, 0x4c, "RxOverFlow"),
	MIB_DESC(1, 0x50, "Filtered"),
	MIB_DESC(1, 0x54, "TxBroad"),
	MIB_DESC(1, 0x58, "TxPause"),
	MIB_DESC(1, 0x5c, "TxMulti"),
	MIB_DESC(1, 0x60, "TxUnderRun"),
	MIB_DESC(1, 0x64, "Tx64Byte"),
	MIB_DESC(1, 0x68, "Tx128Byte"),
	MIB_DESC(1, 0x6c, "Tx256Byte"),
	MIB_DESC(1, 0x70, "Tx512Byte"),
	MIB_DESC(1, 0x74, "Tx1024Byte"),
	MIB_DESC(1, 0x78, "Tx1518Byte"),
	MIB_DESC(1, 0x7c, "TxMaxByte"),
	MIB_DESC(1, 0x80, "TxOverSize"),
	MIB_DESC(2, 0x84, "TxByte"),
	MIB_DESC(1, 0x8c, "TxCollision"),
	MIB_DESC(1, 0x90, "TxAbortCol"),
	MIB_DESC(1, 0x94, "TxMultiCol"),
	MIB_DESC(1, 0x98, "TxSingleCol"),
	MIB_DESC(1, 0x9c, "TxExcDefer"),
	MIB_DESC(1, 0xa0, "TxDefer"),
	MIB_DESC(1, 0xa4, "TxLateCol"),
};

/* The 32bit switch registers are accessed indirectly. To achieve this we need
 * to set the page of the register. Track the last page that was set to reduce
 * mdio writes
 */
static u16 ar8k_current_page = 0xffff;

static void
ar8k_split_addr(u32 regaddr, u16 *r1, u16 *r2, u16 *page)
{
	regaddr >>= 1;
	*r1 = regaddr & 0x1e;

	regaddr >>= 5;
	*r2 = regaddr & 0x7;

	regaddr >>= 3;
	*page = regaddr & 0x3ff;
}


static int
ar8k_mii_read32(struct mii_bus *bus, int phy_id, u32 regnum, u32 *val)
{
	int ret;

	ret = bus->read(bus, phy_id, regnum);
	if (ret >= 0) {
		*val = ret;
		ret = bus->read(bus, phy_id, regnum + 1);
		*val |= ret << 16;
	}

	if (ret < 0) {
		dev_err_ratelimited(&bus->dev,
				    "failed to read ar8k 32bit register\n");
		*val = 0;
		return ret;
	}

	return 0;
}

static void
ar8k_mii_write32(struct mii_bus *bus, int phy_id, u32 regnum, u32 val)
{
	u16 lo, hi;
	int ret;

	lo = val & 0xffff;
	hi = (u16)(val >> 16);

	ret = bus->write(bus, phy_id, regnum, lo);
	if (ret >= 0)
		ret = bus->write(bus, phy_id, regnum + 1, hi);
	if (ret < 0)
		dev_err_ratelimited(&bus->dev,
				    "failed to write ar8k 32bit register\n");
}

static int
ar8k_set_page(struct mii_bus *bus, u16 page)
{
	int ret;

	if (page == ar8k_current_page)
		return 0;

	ret = bus->write(bus, 0x18, 0, page);
	if (ret < 0) {
		dev_err_ratelimited(&bus->dev,
				    "failed to set ar8k page\n");
		return ret;
	}

	ar8k_current_page = page;
	usleep_range(1000, 2000);
	return 0;
}

static int
ar8k_read(struct ar8k_priv *priv, u32 reg, u32 *val)
{
	return regmap_read(priv->regmap, reg, val);
}

static int
ar8k_write(struct ar8k_priv *priv, u32 reg, u32 val)
{
	return regmap_write(priv->regmap, reg, val);
}

static int
ar8k_rmw(struct ar8k_priv *priv, u32 reg, u32 mask, u32 write_val)
{
	return regmap_update_bits(priv->regmap, reg, mask, write_val);
}

static int
ar8k_regmap_read(void *ctx, uint32_t reg, uint32_t *val)
{
	struct ar8k_priv *priv = (struct ar8k_priv *)ctx;
	struct mii_bus *bus = priv->bus;
	u16 r1, r2, page;
	int ret;

	ar8k_split_addr(reg, &r1, &r2, &page);

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = ar8k_set_page(bus, page);
	if (ret < 0)
		goto exit;

	ret = ar8k_mii_read32(bus, 0x10 | r2, r1, val);

exit:
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static int
ar8k_regmap_write(void *ctx, uint32_t reg, uint32_t val)
{
	struct ar8k_priv *priv = (struct ar8k_priv *)ctx;
	struct mii_bus *bus = priv->bus;
	u16 r1, r2, page;
	int ret;

	ar8k_split_addr(reg, &r1, &r2, &page);

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = ar8k_set_page(bus, page);
	if (ret < 0)
		goto exit;

	ar8k_mii_write32(bus, 0x10 | r2, r1, val);

exit:
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static int
ar8k_regmap_update_bits(void *ctx, uint32_t reg, uint32_t mask, uint32_t write_val)
{
	struct ar8k_priv *priv = (struct ar8k_priv *)ctx;
	struct mii_bus *bus = priv->bus;
	u16 r1, r2, page;
	u32 val;
	int ret;

	ar8k_split_addr(reg, &r1, &r2, &page);

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = ar8k_set_page(bus, page);
	if (ret < 0)
		goto exit;

	ret = ar8k_mii_read32(bus, 0x10 | r2, r1, &val);
	if (ret < 0)
		goto exit;

	val &= ~mask;
	val |= write_val;
	ar8k_mii_write32(bus, 0x10 | r2, r1, val);

exit:
	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static const struct regmap_range ar8k_readable_ranges[] = {
	regmap_reg_range(0x000, 0x0fc), /* Global Control */

	regmap_reg_range(0x100, 0x12c), /* Port 0 Control */
	regmap_reg_range(0x200, 0x22c), /* Port 1 Control */
	regmap_reg_range(0x300, 0x32c), /* Port 2 Control */
	regmap_reg_range(0x400, 0x42c), /* Port 3 Control */
	regmap_reg_range(0x500, 0x52c), /* Port 4 Control */
	regmap_reg_range(0x600, 0x62c), /* Port 5 Control */

	regmap_reg_range(0x20000, 0x200a4), /* MIB - Port0 */
	regmap_reg_range(0x20100, 0x201a4), /* MIB - Port1 */
	regmap_reg_range(0x20200, 0x202a4), /* MIB - Port2 */
	regmap_reg_range(0x20300, 0x203a4), /* MIB - Port3 */
	regmap_reg_range(0x20400, 0x204a4), /* MIB - Port4 */
	regmap_reg_range(0x20500, 0x205a4), /* MIB - Port5 */

	/* dummy page selector reg */
	regmap_reg_range(AR8K_SW_REG_PAGE, AR8K_SW_REG_PAGE),
};

static const struct regmap_access_table ar8k_readable_table = {
	.yes_ranges = ar8k_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ar8k_readable_ranges),
};

static struct regmap_config ar8k_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x205a4, /* end MIB - Port5 range */
	.reg_read = ar8k_regmap_read,
	.reg_write = ar8k_regmap_write,
	.reg_update_bits = ar8k_regmap_update_bits,
	.rd_table = &ar8k_readable_table,
	.disable_locking = true, /* Locking is handled by ar8k read/write */
	.cache_type = REGCACHE_NONE, /* Explicitly disable CACHE */
};

static int
ar8k_busy_wait(struct ar8k_priv *priv, u32 reg, u32 mask)
{
	u32 val;

	return regmap_read_poll_timeout(priv->regmap, reg, val, !(val & mask), 0,
				       AR8K_BUSY_WAIT_TIMEOUT * USEC_PER_MSEC);
}

static void
ar8k_port_set_status(struct ar8k_priv *priv, int port, int enable)
{
	u32 mask = AR8K_PORT_STATUS_TXMAC | AR8K_PORT_STATUS_RXMAC;

	/* Port 0 has no internal PHY */
	if (port != 0)
		mask |= AR8K_PORT_STATUS_LINK_AUTO;

	if (enable)
		regmap_set_bits(priv->regmap, AR8K_REG_PORT_STATUS(port), mask);
	else
		regmap_clear_bits(priv->regmap, AR8K_REG_PORT_STATUS(port), mask);
}

static u32
ar8k_port_to_phy(int port)
{
	/* From Andrew Lunn:
	 * Port 0 has no internal phy.
	 * Port 1 has an internal PHY at MDIO address 0.
	 * Port 2 has an internal PHY at MDIO address 1.
	 * ...
	 * Port 5 has an internal PHY at MDIO address 4.
	 */

	return port - 1;
}

static int
ar8k_mdio_busy_wait(struct mii_bus *bus, u32 reg, u32 mask)
{
	u16 r1, r2, page;
	u32 val;
	int ret, ret1;

	ar8k_split_addr(reg, &r1, &r2, &page);

	ret = read_poll_timeout(ar8k_mii_read32, ret1, !(val & mask), 0,
				AR8K_BUSY_WAIT_TIMEOUT * USEC_PER_MSEC, false,
				bus, 0x10 | r2, r1, &val);

	/* Check if ar8k_read has failed for a different reason
	 * before returnting -ETIMEDOUT
	 */
	if (ret < 0 && ret1 < 0)
		return ret1;

	return ret;
}

static int
ar8k_mdio_write(struct mii_bus *bus, int phy, int regnum, u16 data)
{
	u16 r1, r2, page;
	u32 val;
	int ret;

	if (regnum >= AR8K_MDIO_MASTER_MAX_REG)
		return -EINVAL;

	val = AR8K_MDIO_MASTER_BUSY | AR8K_MDIO_MASTER_EN |
	      AR8K_MDIO_MASTER_WRITE | AR8K_MDIO_MASTER_PHY_ADDR(phy) |
	      AR8K_MDIO_MASTER_REG_ADDR(regnum) |
	      AR8K_MDIO_MASTER_DATA(data);

	ar8k_split_addr(AR8K_REG_MDIO_MASTER_CTRL, &r1, &r2, &page);

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = ar8k_set_page(bus, page);
	if (ret)
		goto exit;

	ar8k_mii_write32(bus, 0x10 | r2, r1, val);

	ret = ar8k_mdio_busy_wait(bus, AR8K_REG_MDIO_MASTER_CTRL,
				   AR8K_MDIO_MASTER_BUSY);

exit:
	/* even if the busy_wait timeouts try to clear the MASTER_EN */
	ar8k_mii_write32(bus, 0x10 | r2, r1, 0);

	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static int
ar8k_mdio_read(struct mii_bus *bus, int phy, int regnum)
{
	u16 r1, r2, page;
	u32 val;
	int ret;

	if (regnum >= AR8K_MDIO_MASTER_MAX_REG)
		return -EINVAL;

	val = AR8K_MDIO_MASTER_BUSY | AR8K_MDIO_MASTER_EN |
	      AR8K_MDIO_MASTER_READ | AR8K_MDIO_MASTER_PHY_ADDR(phy) |
	      AR8K_MDIO_MASTER_REG_ADDR(regnum);

	ar8k_split_addr(AR8K_REG_MDIO_MASTER_CTRL, &r1, &r2, &page);

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = ar8k_set_page(bus, page);
	if (ret)
		goto exit;

	ar8k_mii_write32(bus, 0x10 | r2, r1, val);

	ret = ar8k_mdio_busy_wait(bus, AR8K_REG_MDIO_MASTER_CTRL,
				   AR8K_MDIO_MASTER_BUSY);
	if (ret)
		goto exit;

	ret = ar8k_mii_read32(bus, 0x10 | r2, r1, &val);

exit:
	/* even if the busy_wait timeouts try to clear the MASTER_EN */
	ar8k_mii_write32(bus, 0x10 | r2, r1, 0);

	mutex_unlock(&bus->mdio_lock);

	if (ret >= 0)
		ret = val & AR8K_MDIO_MASTER_DATA_MASK;

	return ret;
}

static int
ar8k_internal_mdio_write(struct mii_bus *slave_bus, int phy, int regnum, u16 data)
{
	struct ar8k_priv *priv = slave_bus->priv;
	struct mii_bus *bus = priv->bus;

	return ar8k_mdio_write(bus, phy, regnum, data);
}

static int
ar8k_internal_mdio_read(struct mii_bus *slave_bus, int phy, int regnum)
{
	struct ar8k_priv *priv = slave_bus->priv;
	struct mii_bus *bus = priv->bus;

	return ar8k_mdio_read(bus, phy, regnum);
}

static int
ar8k_legacy_mdio_write(struct mii_bus *slave_bus, int port, int regnum, u16 data)
{
	port = ar8k_port_to_phy(port) % PHY_MAX_ADDR;

	return ar8k_internal_mdio_write(slave_bus, port, regnum, data);
}

static int
ar8k_legacy_mdio_read(struct mii_bus *slave_bus, int port, int regnum)
{
	port = ar8k_port_to_phy(port) % PHY_MAX_ADDR;

	return ar8k_internal_mdio_read(slave_bus, port, regnum);
}

static int
ar8k_mdio_register(struct ar8k_priv *priv)
{
	struct dsa_switch *ds = priv->ds;
	struct device_node *mdio;
	struct mii_bus *bus;

	bus = devm_mdiobus_alloc(ds->dev);
	if (!bus)
		return -ENOMEM;

	bus->priv = (void *)priv;
	snprintf(bus->id, MII_BUS_ID_SIZE, "ar8k-%d.%d",
		 ds->dst->index, ds->index);

	bus->parent = ds->dev;
	bus->phy_mask = ~ds->phys_mii_mask;
	ds->slave_mii_bus = bus;

	/* Check if the devicetree declare the port:phy mapping */
	mdio = of_get_child_by_name(priv->dev->of_node, "mdio");
	if (of_device_is_available(mdio)) {
		bus->name = "ar8k slave mii";
		bus->read = ar8k_internal_mdio_read;
		bus->write = ar8k_internal_mdio_write;
		return devm_of_mdiobus_register(priv->dev, bus, mdio);
	}

	/* If a mapping can't be found the legacy mapping is used,
	 * using the ar8k_port_to_phy function
	 */
	bus->name = "ar8k-legacy slave mii";
	bus->read = ar8k_legacy_mdio_read;
	bus->write = ar8k_legacy_mdio_write;
	return devm_mdiobus_register(priv->dev, bus);
}

static int
ar8k_setup_mdio_bus(struct ar8k_priv *priv)
{
	u32 internal_mdio_mask = 0, external_mdio_mask = 0, reg;
	struct device_node *ports, *port;
	phy_interface_t mode;
	int err;

	ports = of_get_child_by_name(priv->dev->of_node, "ports");
	if (!ports)
		ports = of_get_child_by_name(priv->dev->of_node, "ethernet-ports");

	if (!ports)
		return -EINVAL;

	for_each_available_child_of_node(ports, port) {
		err = of_property_read_u32(port, "reg", &reg);
		if (err) {
			of_node_put(port);
			of_node_put(ports);
			return err;
		}

		if (!dsa_is_user_port(priv->ds, reg))
			continue;

		of_get_phy_mode(port, &mode);

		if (of_property_read_bool(port, "phy-handle") &&
		    mode != PHY_INTERFACE_MODE_INTERNAL)
			external_mdio_mask |= BIT(reg);
		else
			internal_mdio_mask |= BIT(reg);
	}

	of_node_put(ports);
	if (!external_mdio_mask && !internal_mdio_mask) {
		dev_err(priv->dev, "no PHYs are defined.\n");
		return -EINVAL;
	}

	/* The AR8K_MDIO_MASTER_EN Bit, which grants access to PHYs through
	 * the MDIO_MASTER register also _disconnects_ the external MDC
	 * passthrough to the internal PHYs. It's not possible to use both
	 * configurations at the same time!
	 *
	 * Because this came up during the review process:
	 * If the external mdio-bus driver is capable magically disabling
	 * the AR8K_MDIO_MASTER_EN and mutex/spin-locking out the qca8k's
	 * accessors for the time being, it would be possible to pull this
	 * off.
	 */
	if (!!external_mdio_mask && !!internal_mdio_mask) {
		dev_err(priv->dev, "either internal or external mdio bus configuration is supported.\n");
		return -EINVAL;
	}

	if (external_mdio_mask) {
		/* Make sure to disable the internal mdio bus in cases
		 * a dt-overlay and driver reload changed the configuration
		 */

		return regmap_clear_bits(priv->regmap, AR8K_REG_MDIO_MASTER_CTRL,
					 AR8K_MDIO_MASTER_EN);
	}

	return ar8k_mdio_register(priv);
}


static int
ar8k_setup(struct dsa_switch *ds)
{
	struct ar8k_priv *priv = (struct ar8k_priv *)ds->priv;
	int ret, i;
	u32 mask;

	ret = ar8k_setup_mdio_bus(priv);
	if (ret)
		return ret;

	/* Enable CPU Port */
	ret = regmap_set_bits(priv->regmap, AR8K_REG_CPU_PORT, AR8K_CPU_PORT_EN);
	if (ret) {
		dev_err(priv->dev, "failed enabling CPU port");
		return ret;
	}

	/* Initial setup of all ports */
	for (i = 0; i < AR8K_NUM_PORTS; i++) {
		/* Enable QCA header mode on all cpu ports */
		if (dsa_is_cpu_port(ds, i)) {
			ret = ar8k_write(priv, AR8K_REG_PORT_CTRL(i), AR8K_PORT_CTRL_HEADER_EN);
			if (ret) {
				dev_err(priv->dev, "failed enabling QCA header mode");
				return ret;
			}
		}

		/* Disable MAC by default on all user ports */
		if (dsa_is_user_port(ds, i))
			ar8k_port_set_status(priv, i, 0);
	}

	/* Setup our port MTUs to match power on defaults */
	ret = ar8k_rmw(priv, AR8K_REG_GCR, AR8K_GCR_MAX_FRAME_SIZE_MASK,
			 AR8K_GCR_MAX_FRAME_SIZE(ETH_FRAME_LEN + ETH_FCS_LEN));
	if (ret)
		dev_warn(priv->dev, "failed setting MTU settings");

	/* Flush the FDB table */
	// ! ar8k_fdb_flush(priv);

	/* We don't have interrupts for link changes, so we need to poll */
	ds->pcs_poll = true;

	/* Set min a max ageing value supported */
	ds->ageing_time_min = 7000;
	ds->ageing_time_max = 458745000;

	return 0;
}

static void
ar8k_phylink_mac_config(struct dsa_switch *ds, int port, unsigned int mode,
			 const struct phylink_link_state *state)
{
	struct ar8k_priv *priv = ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = ar8k_regmap_update_bits(regmap, AR8K_REG_PORT_STATUS(port),
				 AR8K_PORT_STATUS_LINK_AUTO |
				 AR8K_PORT_STATUS_FLOW_AUTO, 0);
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
}

static void
ar8k_phylink_validate(struct dsa_switch *ds, int port,
		       unsigned long *supported,
		       struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	switch (port) {
	case 0: /* 1st CPU port */
		if (state->interface != PHY_INTERFACE_MODE_NA &&
		    state->interface != PHY_INTERFACE_MODE_MII &&
		    state->interface != PHY_INTERFACE_MODE_RMII)
			goto unsupported;
		break;
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		/* Internal PHY */
		if (state->interface != PHY_INTERFACE_MODE_NA &&
		    state->interface != PHY_INTERFACE_MODE_MII &&
		    state->interface != PHY_INTERFACE_MODE_INTERNAL)
			goto unsupported;
		break;
	default:
unsupported:
		linkmode_zero(supported);
		return;
	}

	phylink_set_port_modes(mask);
	phylink_set(mask, Autoneg);

	phylink_set(mask, 10baseT_Half);
	phylink_set(mask, 10baseT_Full);
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	linkmode_and(supported, supported, mask);
	linkmode_and(state->advertising, state->advertising, mask);
}

static int
ar8k_phylink_mac_link_state(struct dsa_switch *ds, int port,
			     struct phylink_link_state *state)
{
	struct ar8k_priv *priv = ds->priv;
	u32 reg;
	int ret;

	ret = ar8k_read(priv, AR8K_REG_PORT_STATUS(port), &reg);
	if (ret < 0)
		return ret;

	state->link = !!(reg & AR8K_PORT_STATUS_LINK_UP);
	state->an_complete = state->link;
	state->an_enabled = !!(reg & AR8K_PORT_STATUS_LINK_AUTO);
	state->duplex = (reg & AR8K_PORT_STATUS_DUPLEX) ? DUPLEX_FULL :
							   DUPLEX_HALF;

	switch (reg & AR8K_PORT_STATUS_SPEED) {
	case AR8K_PORT_STATUS_SPEED_10:
		state->speed = SPEED_10;
		break;
	case AR8K_PORT_STATUS_SPEED_100:
		state->speed = SPEED_100;
		break;
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}

	state->pause = MLO_PAUSE_NONE;
	if (reg & AR8K_PORT_STATUS_RXFLOW)
		state->pause |= MLO_PAUSE_RX;
	if (reg & AR8K_PORT_STATUS_TXFLOW)
		state->pause |= MLO_PAUSE_TX;

	return 1;
}

static void
ar8k_phylink_mac_link_down(struct dsa_switch *ds, int port, unsigned int mode,
			    phy_interface_t interface)
{
	struct ar8k_priv *priv = ds->priv;

	ar8k_port_set_status(priv, port, 0);
}

static void
ar8k_phylink_mac_link_up(struct dsa_switch *ds, int port, unsigned int mode,
			  phy_interface_t interface, struct phy_device *phydev,
			  int speed, int duplex, bool tx_pause, bool rx_pause)
{
	struct ar8k_priv *priv = ds->priv;
	u32 reg;

	if (phylink_autoneg_inband(mode)) {
		reg = AR8K_PORT_STATUS_LINK_AUTO;
	} else {
		switch (speed) {
		case SPEED_10:
			reg = AR8K_PORT_STATUS_SPEED_10;
			break;
		case SPEED_100:
			reg = AR8K_PORT_STATUS_SPEED_100;
			break;
		default:
			reg = AR8K_PORT_STATUS_LINK_AUTO;
			break;
		}

		if (duplex == DUPLEX_FULL)
			reg |= AR8K_PORT_STATUS_DUPLEX;

		if (rx_pause || dsa_is_cpu_port(ds, port))
			reg |= AR8K_PORT_STATUS_RXFLOW;

		if (tx_pause || dsa_is_cpu_port(ds, port))
			reg |= AR8K_PORT_STATUS_TXFLOW;
	}

	reg |= AR8K_PORT_STATUS_TXMAC | AR8K_PORT_STATUS_RXMAC;

	ar8k_write(priv, AR8K_REG_PORT_STATUS(port), reg);
}

static int
ar8k_set_ageing_time(struct dsa_switch *ds, unsigned int msecs)
{
	struct ar8k_priv *priv = ds->priv;
	unsigned int secs = msecs / 1000;
	u32 val;

	/* AGE_TIME reg is set in 7s step */
	val = secs / 7;

	/* Handle case with 0 as val to NOT disable learning */
	if (!val)
		val = 1;

	return regmap_update_bits(priv->regmap, AR8K_REG_ATU_CTRL,
				  AR8K_ATU_AGE_TIME_MASK,
				  AR8K_ATU_AGE_TIME(val));
}

static int
ar8k_port_enable(struct dsa_switch *ds, int port,
		  struct phy_device *phy)
{
	struct ar8k_priv *priv = (struct ar8k_priv *)ds->priv;

	ar8k_port_set_status(priv, port, 1);
	priv->port_enabled_map |= BIT(port);

	if (dsa_is_user_port(ds, port))
		phy_support_asym_pause(phy);

	return 0;
}

static void
ar8k_port_disable(struct dsa_switch *ds, int port)
{
	struct ar8k_priv *priv = (struct ar8k_priv *)ds->priv;

	ar8k_port_set_status(priv, port, 0);
	priv->port_enabled_map &= ~BIT(port);
}

static int
ar8k_port_change_mtu(struct dsa_switch *ds, int port, int new_mtu)
{
	struct ar8k_priv *priv = ds->priv;
	/* We have only have a general MTU setting.
	 * DSA always set the CPU port's MTU to the largest MTU of the salve ports.
	 * Setting MTU just for the CPU port is sufficient to correctly set a
	 * value for every port.
	 */
	if (!dsa_is_cpu_port(ds, port))
		return 0;

	/* Include L2 header / FCS length */
	return ar8k_rmw(priv, AR8K_REG_GCR, AR8K_GCR_MAX_FRAME_SIZE_MASK,
		       AR8K_GCR_MAX_FRAME_SIZE(new_mtu + ETH_HLEN + ETH_FCS_LEN));
}

static int
ar8k_port_max_mtu(struct dsa_switch *ds, int port)
{
	return AR8K_MAX_MTU;
}

static u32 ar8k_get_phy_flags(struct dsa_switch *ds, int port)
{
	struct ar8k_priv *priv = ds->priv;

	/* Communicate to the phy internal driver the switch revision.
	 * Based on the switch revision different values needs to be
	 * set to the dbg and mmd reg on the phy.
	 * The first 2 bit are used to communicate the switch revision
	 * to the phy driver.
	 */
	if (port > 0 && port < 6)
		return priv->switch_revision;

	return 0;
}

static enum dsa_tag_protocol
ar8k_get_tag_protocol(struct dsa_switch *ds, int port,
		       enum dsa_tag_protocol mp)
{
	return DSA_TAG_PROTO_QCA;
}

static const struct dsa_switch_ops ar8k_switch_ops = {
	.get_tag_protocol	= ar8k_get_tag_protocol, // done
	.setup			= ar8k_setup,
	.set_ageing_time	= ar8k_set_ageing_time,
	.port_enable		= ar8k_port_enable,
	.port_disable		= ar8k_port_disable,
	.port_change_mtu	= ar8k_port_change_mtu,
	.port_max_mtu		= ar8k_port_max_mtu,
	.phylink_validate	= ar8k_phylink_validate,
	.phylink_mac_link_state	= ar8k_phylink_mac_link_state,
	.phylink_mac_config	= ar8k_phylink_mac_config,
	.phylink_mac_link_down	= ar8k_phylink_mac_link_down,
	.phylink_mac_link_up	= ar8k_phylink_mac_link_up,
	.get_phy_flags		= ar8k_get_phy_flags,
};

static int ar8k_read_switch_id(struct ar8k_priv *priv)
{
	const struct ar8k_match_data *data;
	u32 val;
	u8 id;
	int ret;

	/* get the switches ID from the compatible */
	data = of_device_get_match_data(priv->dev);
	if (!data)
		return -ENODEV;

	ret = ar8k_read(priv, AR8K_REG_MASK_CTRL, &val);
	if (ret < 0)
		return -ENODEV;

	id = AR8K_MASK_CTRL_DEVICE_ID(val);
	if (id != data->id) {
		dev_err(priv->dev, "Switch id detected %x but expected %x", id, data->id);
		return -ENODEV;
	}

	priv->switch_id = id;

	/* Save revision to communicate to the internal PHY driver */
	priv->switch_revision = AR8K_MASK_CTRL_REV_ID(val);

	return 0;
}

static int
ar8k_sw_probe(struct mdio_device *mdiodev)
{
	struct ar8k_priv *priv;
	int ret;

	/* allocate the private data struct so that we can probe the switches
	 * ID register
	 */
	priv = devm_kzalloc(&mdiodev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->bus = mdiodev->bus;
	priv->dev = &mdiodev->dev;

	priv->reset_gpio = devm_gpiod_get_optional(priv->dev, "reset",
						   GPIOD_ASIS);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		/* The active low duration must be greater than 10 ms
		 * and checkpatch.pl wants 20 ms.
		 */
		msleep(20);
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
	}

	/* Start by setting up the register mapping */
	priv->regmap = devm_regmap_init(&mdiodev->dev, NULL, priv,
					&ar8k_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(priv->dev, "regmap initialization failed");
		return PTR_ERR(priv->regmap);
	}

	/* Check the detected switch id */
	ret = ar8k_read_switch_id(priv);
	if (ret)
		return ret;

	priv->ds = devm_kzalloc(&mdiodev->dev, sizeof(*priv->ds), GFP_KERNEL);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->dev = &mdiodev->dev;
	priv->ds->num_ports = AR8K_NUM_PORTS;
	priv->ds->configure_vlan_while_not_filtering = true;
	priv->ds->priv = priv;
	priv->ops = ar8k_switch_ops;
	priv->ds->ops = &priv->ops;
	mutex_init(&priv->reg_mutex);
	dev_set_drvdata(&mdiodev->dev, priv);

	return dsa_register_switch(priv->ds);
}

static void
ar8k_sw_remove(struct mdio_device *mdiodev)
{
	struct ar8k_priv *priv = dev_get_drvdata(&mdiodev->dev);
	int i;

	for (i = 0; i < AR8K_NUM_PORTS; i++)
		ar8k_port_set_status(priv, i, 0);

	dsa_unregister_switch(priv->ds);
}


static const struct ar8k_match_data qca8236 = {
	.id = AR8K_ID_AR8236,
};

static const struct of_device_id ar8k_of_match[] = {
	{ .compatible = "qca,ar8236-switch", .data = &qca8236 },
	{ /* sentinel */ },
};

static struct mdio_driver ar8k_mdio_driver = {
	.probe = ar8k_sw_probe,
	.remove = ar8k_sw_remove,
	.mdiodrv.driver = {
		.name = "ar8k",
		.of_match_table = ar8k_of_match,
	},
};

mdio_module_driver(ar8k_mdio_driver);

MODULE_AUTHOR("Andrew Powers-Holmes <aholmes@omnom.net>");
MODULE_DESCRIPTION("DSA Driver for Atheros AR8236 switch");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ar8k");
