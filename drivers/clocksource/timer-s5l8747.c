// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Samsung S5L8747 timer driver
 *
 * Copyright (c) 2025, Nick Chan
 */

#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/sched_clock.h>
#include <linux/bitops.h>

#define TIMER_64_HI	0x0
#define TIMER_64_LO	0x4
#define TIMER_64_CTL	0x8

#define TIMER_64_CTL_EN BIT(3)

static void __iomem *sched_clock_base = NULL;

struct s5l8747_timer {
	void __iomem *base;
	struct clocksource clksrc;
};

static inline struct s5l8747_timer *to_s5l8747_timer(struct clocksource *c)
{
	return container_of(c, struct s5l8747_timer, clksrc);
}

static u64 s5l8747_timer64_read(void __iomem *base)
{
	return ((u64)readl_relaxed(base + TIMER_64_HI) << 32)
		| readl_relaxed(base + TIMER_64_LO);
}

static u64 s5l8747_timer64_clksrc_read(struct clocksource *c) {
	return s5l8747_timer64_read(to_s5l8747_timer(c)->base);
}

static u64 s5l8747_timer64_sched_clock_read(void)
{
	return s5l8747_timer64_read(sched_clock_base);
}

static int __init s5l8747_timer64_of_register(struct device_node *np) {
	struct clk *clk;
	struct s5l8747_timer *timer;
	void __iomem *base;
	u32 rate;
	int ret;

	base = of_iomap(np, 0);
	if (!base)
		return -ENXIO;

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto out_unmap;
	}

	ret = clk_prepare_enable(clk);
	if (ret)
		goto out_clk_put;

	rate = clk_get_rate(clk);
	if (!rate) {
		ret = -EINVAL;
		goto out_clk_disable;
	}

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (!timer) {
		ret = -ENOMEM;
		goto out_clk_disable;
	}

	writel(TIMER_64_CTL_EN, base + TIMER_64_CTL);

	timer->base = base;
	timer->clksrc.name = "s5l8747_timer64";
	timer->clksrc.rating = 300;
	timer->clksrc.read = s5l8747_timer64_clksrc_read;
	timer->clksrc.mask = CLOCKSOURCE_MASK(64);
	timer->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	ret = clocksource_register_hz(&timer->clksrc, rate);

	if (ret) {
		pr_err("failed to init clocksource (%d)\n", ret);
		goto out_kfree;
	}

	if (!sched_clock_base) {
		sched_clock_base = base;
		sched_clock_register(s5l8747_timer64_sched_clock_read, 64, rate);
	}

	return 0;

out_kfree:
	kfree(timer);
out_clk_disable:
	clk_disable_unprepare(clk);
out_clk_put:
	clk_put(clk);
out_unmap:
	iounmap(base);
	return ret;
}

TIMER_OF_DECLARE(s5l8747_timer64, "samsung,s5l8747-timer64",
			s5l8747_timer64_of_register);
