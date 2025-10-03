/****************************************************************************
 * arch/arm/src/am67/hardware/am67_i2c.h
 *
 * Created and maintained by T3 Gemstone Dev Team
 * Author(s): Emre CECANPUNAR <emreleno@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_AM67_HARDWARE_AM67_I2C_H
#define __ARCH_ARM_SRC_AM67_HARDWARE_AM67_I2C_H

#include "am67_memorymap.h"

/*
 * Extra information about pinmux from datasheet:
 * PIN		SCL	SDA	SCL-SDA-MUX	SCL-SDA-ADDR
 * I2C0		D23	B22	0-0		0x000F41E0-0x000F41E4
 * I2C1		C24	A22	0-0		0x000F41E8-0x000F41EC
 *
 * All of them have 7 as default
*/

/* 
 * GPIO1_26 - SCL
 * GPIO1_27 - SDA
 * */
#define AM67_I2C0_GPIO_SCL MAIN_GPIO1_BASE + 26
#define AM67_I2C0_GPIO_SDA MAIN_GPIO1_BASE + 27

#define AM67_I2C0_BASE 0x20000000
#define AM67_I2C1_BASE 0x20010000
#define AM67_I2C2_BASE 0x20020000
#define AM67_I2C3_BASE 0x20030000
#define AM67_I2C4_BASE 0x20040000

/* R5FSS0_CORE0_INTR_IN_193 - I2C0_POINTRPEND_0 */
#define AM67_I2C0_IRQ  193
#define AM67_I2C1_IRQ  194
#define AM67_I2C2_IRQ  195
#define AM67_I2C3_IRQ  196

/* Register offsets */
#define AM67_I2C_SYSC    0x10  /* system config */
#define AM67_I2C_IE      0x84  /* interrupt enable */
#define AM67_I2C_STAT    0x88  /* status register */
#define AM67_I2C_SYSS    0x90  /* system status */
#define AM67_I2C_BUF     0x94  /* FIFO buffer configuration */
#define AM67_I2C_CNT     0x98  /* data count register */
#define AM67_I2C_DATA    0x9C  /* data register (TX/RX) */
#define AM67_I2C_CON     0xA4  /* control register */
#define AM67_I2C_SA      0xAC  /* slave address register */
#define AM67_I2C_PSC     0xB0  /* prescaler */
#define AM67_I2C_SCLL    0xB4  /* SCL low time */
#define AM67_I2C_SCLH    0xB8  /* SCL high time */
#define AM67_I2C_BUFSTAT 0xC0  /* FIFO buffer status */

/* CON register bits */
#define I2C_CON_EN            (1 << 15)
#define I2C_CON_BE            (1 << 14)
#define I2C_CON_STB           (1 << 11)
#define I2C_CON_MST           (1 << 10)
#define I2C_CON_TRX           (1 << 9)
#define I2C_CON_XSA           (1 << 8)
#define I2C_CON_STP           (1 << 1)
#define I2C_CON_STT           (1 << 0)

/* BUF register - FIFO Buffer Configuration */
#define I2C_BUF_TXTRSH_SHIFT   0
#define I2C_BUF_TXTRSH_MASK    0x003F
#define I2C_BUF_TXTRSH(n)      (((n) & 0x3F) << I2C_BUF_TXTRSH_SHIFT)
#define I2C_BUF_TXFIFO_CLR     (1 << 6)
#define I2C_BUF_XDMA_EN        (1 << 7)
#define I2C_BUF_RXTRSH_SHIFT   8
#define I2C_BUF_RXTRSH_MASK    0x3F00
#define I2C_BUF_RXTRSH(n)      (((n) & 0x3F) << I2C_BUF_RXTRSH_SHIFT)
#define I2C_BUF_RXFIFO_CLR     (1 << 14)
#define I2C_BUF_RDMA_EN        (1 << 15)

/* BUFSTAT register - FIFO Status */
#define I2C_BUFSTAT_TXSTAT_SHIFT  0
#define I2C_BUFSTAT_TXSTAT_MASK   0x003F
#define I2C_BUFSTAT_RXSTAT_SHIFT  8
#define I2C_BUFSTAT_RXSTAT_MASK   0x3F00

/* STAT register - Status bits */
#define I2C_STAT_AL             (1 << 0)    /* arbitration lost */
#define I2C_STAT_NACK           (1 << 1)    /* nack received */
#define I2C_STAT_ARDY           (1 << 2)    /* access ready / completion */
#define I2C_STAT_RRDY           (1 << 3)    /* receive ready */
#define I2C_STAT_XRDY           (1 << 4)    /* transmit ready */
#define I2C_STAT_RDR            (1 << 13)   /* RX FIFO threshold reached */
#define I2C_STAT_XDR            (1 << 14)   /* TX FIFO draining */

/* IE register - Interrupt enable bits */
#define I2C_IE_AL               (1 << 0)
#define I2C_IE_NACK             (1 << 1)
#define I2C_IE_ARDY             (1 << 2)
#define I2C_IE_RRDY             (1 << 3)
#define I2C_IE_XRDY             (1 << 4)
#define I2C_IE_RDR              (1 << 13)
#define I2C_IE_XDR              (1 << 14)

/* SYSC register bits */
#define I2C_SYSC_SRST           (1 << 1)

/* SYSS register bits */
#define I2C_SYSS_RDONE          (1 << 0)

/* For convenience: macro to get register address */
#define AM67_I2C_REG(base, offset)   ((base) + (offset))

#endif /* ARCH_ARM_SRC_AM67_HARDWARE_AM67_I2C_H */
