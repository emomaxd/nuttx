/****************************************************************************
 * boards/arm/am67/t3-gem-o1/src/am67_bringup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/fs/fs.h>

#include "t3-gem-o1.h"

#include "am67_i2c.h"


int am67_bringup(void)
{
  int ret = OK;
  
  #ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  /* TODO: use mount(...) which is userspace function and has better error 
   * handling than nx_mount(...)
   * */
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
  }
  
#if 0 /* CHECK FOR CONFIG_RPMSG */
  /* Mount the RPMSG file system 
   * equivalent of this nsh code from documentation:
   * ==> mount -t rpmsgfs -o cpu=master,fs=/proc /proc.master
   * */
  ret = mount(NULL, "/proc.master", "rpmsgfs", 0, "cpu=master,fs=/proc");
  if (ret < 0)
  {
    syslog(LOG_ERR, "ERROR: Failed to mount rpmsgfs at /proc.master: %d\n", ret);
  }
#endif

  #endif /* CONFIG_FS_PROCFS */

  /* Register I2C driver as /dev/i2c0 for nuttx api */
#ifdef CONFIG_AM67_I2C
    struct i2c_master_s *i2c0;
    
    /* Initialize I2C0 */
    i2c0 = am67_i2c_initialize(0, AM67_I2C0_BASE, AM67_I2C0_IRQ
		    AM67_I2C0_GPIO_SCL, AM67_I2C0_GPIO_SDA);
    if (!i2c0)
    {
        syslog(LOG_ERR, "ERROR: Failed to initialize I2C0\n");
        return -ENODEV;
    }
#endif

#ifdef CONFIG_AM67_I2C1
    struct i2c_master_s *i2c1;
    
    i2c1 = am67_i2c_initialize(1, AM67_I2C1_BASE, AM67_I2C1_IRQ, 
		    AM67_I2C1_GPIO_SCL, AM67_I2C1_GPIO_SDA);
    if (!i2c1)
    {
        syslog(LOG_ERR, "ERROR: Failed to initialize I2C1\n");
    }
#endif
    /* Add I2C2, I2C3, I2C4 if needed */

    return ret;
}
