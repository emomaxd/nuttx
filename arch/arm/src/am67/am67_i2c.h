/****************************************************************************
 * arch/arm/src/am67/am67_i2c.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_I2C_H
#define __ARCH_ARM_SRC_AM67_AM67_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdint.h>

#include "hardware/am67_i2c.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Function prototypes */
FAR struct i2c_master_s *am67_i2c_initialize(int port, uintptr_t base, int irq,
                                             uint32_t input_clk, uint32_t i2c_clk);
int am67_i2c_uninitialize(FAR struct i2c_master_s *dev);

#endif /* __ARCH_ARM_SRC_AM67_AM67_I2C_H */

