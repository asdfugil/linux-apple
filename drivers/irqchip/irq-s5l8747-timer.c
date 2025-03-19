// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Samsung S5L8747 timer interrupt controller
 * Represents the IRQ status registers shared among timers in S5L8747
 *
 * Copyright (C) 2025, Nick Chan <towinchenmi@gmail.com>
 */

#include <linux/irq.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/slab.h>

#define BITS_IN_U32 32
 
struct s5l8747_timer_irq_device {
    struct device *dev;
    struct irq_chip	chip;
    int	irq;
    struct irq_domain *irqd;
    void* __iomem base;
    raw_spinlock_t wa_lock;
};
 
static void s5l8747_timer_irq_noop(struct irq_data *d)
{
    /* Nothing to do here */
}
 
static irqreturn_t s5l8747_timer_irq_handler(int irq, void *s5l8747_timer_irq)
{
    struct s5l8747_timer_irq_device *timer_irq = s5l8747_timer_irq;
    u32 pending;
	unsigned long wa_lock_flags;
    int src, err;

    pr_info("s5l8747_timer_irq_handler: start irq %d\n", irq);
 
    pending = readl(timer_irq->base);
    writel(pending, timer_irq->base);
 
    for (src = 0; src < BITS_IN_U32; src++) {
        if (pending & BIT(src)) {
            raw_spin_lock_irqsave(&timer_irq->wa_lock, wa_lock_flags);
            err = generic_handle_domain_irq(timer_irq->irqd, src);
            raw_spin_unlock_irqrestore(&timer_irq->wa_lock,
                            wa_lock_flags);
 
            if (err)
                pr_warn_ratelimited("spurious irq detected hwirq %d\n",
                            src);
        }
    }
 
    return IRQ_HANDLED;
}
 
static int s5l8747_timer_irq_map(struct irq_domain *h, unsigned int virq,
                irq_hw_number_t hw)
{
    struct s5l8747_timer_irq_device *timer_irq = h->host_data;
 
    irq_set_chip_data(virq, timer_irq);
    irq_set_chip_and_handler(virq, &timer_irq->chip, handle_level_irq);
    irq_set_probe(virq);
    return 0;
}
 
static const struct irq_domain_ops s5l8747_timer_irq_ops = {
    .map	= s5l8747_timer_irq_map,
    .xlate	= irq_domain_xlate_onecell,
};
 
static int s5l8747_timer_of_ic_init(struct device_node *np, struct device_node *parent)
{
    struct s5l8747_timer_irq_device *timer_irq;
    int ret;
 
    if (np == NULL)
        return -EINVAL;
 
    timer_irq = kzalloc(sizeof(*timer_irq), GFP_KERNEL);
    if (!timer_irq)
        return -ENOMEM;

 
    timer_irq->base = of_iomap(np, 0);
    if (IS_ERR(timer_irq->base))
         return PTR_ERR(timer_irq->base);
 

    timer_irq->irq = of_irq_get(np, 0);
    if (timer_irq->irq < 0) {
        if (timer_irq->irq == -ENODEV)
            ret = -EPROBE_DEFER;
        else
            ret = timer_irq->irq;
        goto out_kfree;
    }
 
    timer_irq->chip.name		= "s5l8747-timer-irq";
    /*
     * This "interrupt controller" is just a single register indicating
     * IRQ status so these are no-ops.
     */
    timer_irq->chip.irq_ack	= s5l8747_timer_irq_noop;
    timer_irq->chip.irq_mask	= s5l8747_timer_irq_noop;
    timer_irq->chip.irq_unmask	= s5l8747_timer_irq_noop;
 
    timer_irq->irqd = irq_domain_add_linear(np, BITS_IN_U32,
                        &s5l8747_timer_irq_ops, timer_irq);
    if (!timer_irq->irqd) {
        pr_info("IRQ domain registration failed\n");
        ret = -ENODEV;
        goto out_kfree;
    }
 
    raw_spin_lock_init(&timer_irq->wa_lock);
 
    ret = request_irq(timer_irq->irq, s5l8747_timer_irq_handler,
               0, "s5l8747 timer irq", timer_irq);

    if (ret) {
        irq_domain_remove(timer_irq->irqd);
        goto out_kfree;
    }

    writel(0, timer_irq->base);
 
    pr_info("registered s5l8747 timer irqchip\n");

    return 0;

out_kfree:
    kfree(timer_irq);
    return ret;
}

IRQCHIP_DECLARE(s5l8747_timer_irq, "samsung,s5l8747-timer-intc", s5l8747_timer_of_ic_init);
