/****************************************************************************
 * arch/arm/src/am67/am67_i2c.c
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

/****************************************************************************
 * DESCRIPTION
 * 
 * This driver implements I2C master mode support for Texas Instruments
 * AM67x SoC family. It provides:
 *
 * - Standard (100kHz) and Fast (400kHz) mode support
 * - Multi-message transfer sequences
 * - Dynamic timeout calculation
 * - GPIO-based bus recovery
 * - Interrupt-driven operation
 * - Reference counting for multi-client access
 *
 * HARDWARE OVERVIEW:
 * 
 * The AM67x I2C controller features:
 * - Programmable clock generation (via PSC, SCLL, SCLH registers)
 * - 7-bit and 10-bit slave addressing
 * - Master transmit/receive modes
 * - Interrupt-driven data transfer
 * - Built-in START/STOP condition generation
 * - Bus arbitration and clock stretching support
 *
 * REGISTER MAP:
 * - SYSC:  System Configuration (reset control)
 * - SYSS:  System Status (reset done indicator)
 * - CON:   Control register (enable, mode, START/STOP)
 * - SA:    Slave Address register
 * - CNT:   Data Count register
 * - DATA:  Data Transmit/Receive register
 * - STAT:  Status register (interrupt flags)
 * - IE:    Interrupt Enable register
 * - PSC:   Clock Prescaler
 * - SCLL:  SCL Low Time
 * - SCLH:  SCL High Time
 *
 * TRANSFER FLOW:
 * 1. Configure frequency (PSC, SCLL, SCLH)
 * 2. Set slave address (SA)
 * 3. Set byte count (CNT)
 * 4. Enable interrupts (IE)
 * 5. Start transfer (CON with STT bit)
 * 6. ISR handles data transfer (XRDY/RRDY interrupts)
 * 7. Transfer completes (ARDY interrupt)
 * 8. Move to next message or finish
 *
 * CLOCK CALCULATION:
 * I2C_CLK = INPUT_CLK / ((PSC + 1) * (SCLL + 7 + SCLH + 5))
 *
 * Where:
 * - INPUT_CLK = 48 MHz (typical for AM67x)
 * - PSC = Prescaler value (0-255)
 * - SCLL = SCL Low period (cycles)
 * - SCLH = SCL High period (cycles)
 * - Constants 7 and 5 are internal filtering delays
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "arm_internal.h"
#include "am67_irq.h"
#include "am67_i2c.h"
#include "am67_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout Configuration
 * 
 * Base timeout covers initialization and small transfers.
 * Per-byte timeout scales with transfer size.
 * Formula: Total = BASE + (bytes * PER_BYTE)
 */
#define I2C_TIMEOUT_BASE_MS      100   /* 100ms base timeout */
#define I2C_TIMEOUT_PER_BYTE_US  100   /* 100us per byte (for 100kHz) */

/* Input clock frequency to I2C peripheral
 * This is the functional clock after PSS (Power and Sleep Controller)
 * Typically 48MHz for AM67x family
 */
#define AM67_I2C_INPUT_CLOCK     48000000

/* Bus Recovery Configuration
 * 
 * When a slave device holds SDA low (stuck), we manually clock SCL
 * to force the slave to release the bus.
 */
#define I2C_RECOVERY_CLOCK_MAX   10    /* Max clock cycles for recovery */
#define I2C_RECOVERY_STRETCH_MAX 10    /* Max stretch wait iterations */

/* FIFO Configuration */
#define I2C_FIFO_SIZE           64      /* Hardware FIFO size */
#define I2C_FIFO_THRESHOLD      32      /* Half-full threshold */
#define I2C_USE_FIFO_MIN_BYTES  8       /* Use FIFO for transfers >= 8 bytes */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2c_priv_s
 *
 * Description:
 *   Private driver state structure for AM67 I2C peripheral.
 *   This structure maintains all runtime state for a single I2C controller.
 *
 * THREAD SAFETY:
 *   - 'lock' mutex protects all fields except those marked volatile
 *   - Volatile fields are accessed from both task and ISR context
 *   - ISR only reads/writes fields under critical section protection
 *
 * MEMORY LAYOUT:
 *   - 'dev' MUST be first member (for type casting)
 *   - Other fields can be reordered for optimal packing
 *
 * Fields:
 *   dev         - Standard NuttX I2C master device (MUST be first)
 *                 Contains ops table pointer for polymorphism
 *
 *   base        - Physical base address of I2C peripheral registers
 *                 Typical values: 0x20000000, 0x20010000, etc.
 *
 *   irq         - Hardware interrupt number for this I2C instance
 *                 Used for irq_attach() and up_enable_irq()
 *
 *   refs        - Reference counter for multi-client support
 *                 Incremented on each i2c_register(), decremented on uninit
 *                 Hardware only disabled when refs reaches 0
 *
 *   lock        - Mutual exclusion mutex
 *                 Serializes access to driver state from multiple tasks
 *                 Prevents concurrent transfers on same I2C bus
 *
 *   waitsem     - Binary semaphore for transfer completion
 *                 Task blocks on this, ISR posts when transfer completes
 *                 Reset to 0 before each transfer
 *
 *   scl_pin     - GPIO pin configuration for SCL signal
 *                 Format: GPIO bank/pin encoding from am67_gpio.h
 *                 Used during bus recovery for manual clock generation
 *
 *   sda_pin     - GPIO pin configuration for SDA signal
 *                 Format: GPIO bank/pin encoding from am67_gpio.h
 *                 Used during bus recovery to monitor/force SDA state
 *
 *   msgs        - Pointer to current message array being processed
 *                 Application provides array of i2c_msg_s structures
 *                 Must remain valid until transfer completes
 *
 *   msgc        - Total number of messages in current transfer sequence
 *                 Range: 1 to INT_MAX (validated in am67_transfer)
 *
 *   msgidx      - Index of currently processing message (0-based)
 *                 Advanced by ISR when ARDY (access ready) occurs
 *                 Reset to 0 at start of each transfer
 *
 *   byteidx     - Index of current byte within current message (0-based)
 *                 Incremented in ISR for each XRDY/RRDY event
 *                 Reset to 0 when moving to next message
 *
 *   frequency   - Currently configured I2C bus frequency (Hz)
 *                 Common values: 100000 (standard), 400000 (fast)
 *                 Used to avoid redundant reconfiguration
 *
 *   result      - Transfer result code (volatile, set by ISR)
 *                 Values: 0 (OK), -EIO (NACK/arbitration), -ETIMEDOUT
 *                 Checked by transfer() after semaphore wakeup
 *
 *   inprogress  - Transfer active flag (volatile, ISR synchronized)
 *                 true: Transfer ongoing, false: Idle
 *                 Used to detect timeout vs completion
 *
 ****************************************************************************/
struct am67_i2c_priv_s
{
    struct i2c_master_s dev;        /* Standard I2C device - must be first */
    uintptr_t base;                 /* I2C peripheral base address */
    int irq;                        /* IRQ number */
    
    /* Multi-client support */
    int refs;                       /* Reference count */
    mutex_t lock;                   /* Mutual exclusion mutex */
    sem_t waitsem;                  /* Wait for transfer completion */
    
    /* GPIO pins for bus recovery */
    uint32_t scl_pin;               /* SCL GPIO configuration */
    uint32_t sda_pin;               /* SDA GPIO configuration */

    /* Current transfer state (protected by lock) */
    struct i2c_msg_s *msgs;         /* Message array */
    int msgc;                       /* Number of messages */
    int msgidx;                     /* Current message index */
    int byteidx;                    /* Current byte index in message */
    uint32_t frequency;             /* Current I2C frequency */
    volatile int result;            /* Transfer result */
    volatile bool inprogress;       /* Transfer in progress flag */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Core I2C operations (required by i2c_ops_s) */
static int am67_transfer(FAR struct i2c_master_s *dev,
                         FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int am67_reset(FAR struct i2c_master_s *dev);
#endif
static int am67_setup(FAR struct i2c_master_s *dev);
static int am67_shutdown(FAR struct i2c_master_s *dev);

/* Hardware control functions */
static void am67_i2c_hw_reset(uintptr_t base);
static void am67_i2c_set_frequency(uintptr_t base, uint32_t frequency);
static void am67_i2c_calc_prescaler(uint32_t frequency,
                                     uint32_t *psc, uint32_t *scll,
                                     uint32_t *sclh);

/* Transfer management */
static void am67_i2c_start_transfer(FAR struct am67_i2c_priv_s *priv);
static clock_t am67_i2c_calc_timeout(int msgc, struct i2c_msg_s *msgs);
static int am67_i2c_isr(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: g_am67_i2c_ops
 *
 * Description:
 *   Static I2C operations table shared by all AM67 I2C instances.
 *   This vtable-like structure provides polymorphic behavior for I2C API.
 *
 *   The NuttX I2C framework calls these functions through the ops pointer.
 *
 * Operations:
 *   transfer  - Execute a sequence of I2C messages (required)
 *   reset     - Perform bus recovery (optional, CONFIG_I2C_RESET)
 *   setup     - Initialize peripheral (called once, currently no-op)
 *   shutdown  - Disable peripheral for power saving (optional)
 *
 ****************************************************************************/
static const struct i2c_ops_s g_am67_i2c_ops =
{
    .transfer  = am67_transfer,
#ifdef CONFIG_I2C_RESET
    .reset     = am67_reset,
#endif
    .setup     = am67_setup,
    .shutdown  = am67_shutdown,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2c_hw_reset
 *
 * Description:
 *   Perform hardware reset of the I2C peripheral module.
 *
 *   This function:
 *   1. Triggers software reset via SYSC register
 *   2. Waits for reset completion (SYSS.RDONE bit)
 *   3. Disables all interrupts
 *   4. Clears all status flags
 *
 * Input Parameters:
 *   base - Physical base address of I2C peripheral registers
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   - Function blocks up to ~10ms waiting for reset
 *   - Safe to call with interrupts disabled
 *   - Does NOT reconfigure clock or enable the module
 *
 * Side Effects:
 *   - All I2C configuration registers reset to default values
 *   - Any ongoing transfer is immediately aborted
 *   - Module enters disabled state
 *
 * POSIX Compliance:
 *   Not applicable (hardware-specific function)
 *
 ****************************************************************************/
static void am67_i2c_hw_reset(uintptr_t base)
{
    uint32_t timeout = 1000;

    putreg32(I2C_SYSC_SRST, AM67_I2C_REG(base, AM67_I2C_SYSC));
    
    while (!(getreg32(AM67_I2C_REG(base, AM67_I2C_SYSS)) & I2C_SYSS_RDONE))
    {
        if (--timeout == 0)
        {
            break;
        }
        up_udelay(10);
    }

    /* Clear FIFO buffers */
    putreg32(I2C_BUF_TXFIFO_CLR | I2C_BUF_RXFIFO_CLR,
             AM67_I2C_REG(base, AM67_I2C_BUF));

    putreg32(0, AM67_I2C_REG(base, AM67_I2C_IE));
    putreg32(0xFFFF, AM67_I2C_REG(base, AM67_I2C_STAT));
}
/****************************************************************************
 * Name: am67_i2c_calc_prescaler
 *
 * Description:
 *   Calculate optimal prescaler and SCL timing values for desired frequency.
 *
 *   This function uses a brute-force search algorithm to find PSC, SCLL, 
 *   and SCLH values that produce the closest match to the requested
 *   I2C bus frequency.
 *
 * ALGORITHM:
 *   1. Iterate through valid SCLL+SCLH combinations (s)
 *   2. For each s, iterate through prescaler values (p)
 *   3. Calculate resulting frequency from PSC/SCLL/SCLH
 *   4. Track combination with minimum frequency error
 *   5. Return best match found
 *
 * CLOCK FORMULA:
 *   I2C_CLK = INPUT_CLK / ((PSC + 1) * (SCLL + 7 + SCLH + 5))
 *
 *   Where:
 *   - INPUT_CLK = 48 MHz (AM67_I2C_INPUT_CLOCK)
 *   - PSC = Prescaler (0-255)
 *   - SCLL = SCL low period in clock cycles
 *   - SCLH = SCL high period in clock cycles
 *   - 7, 5 = Internal filter delays (fixed by hardware)
 *
 * CONSTRAINTS:
 *   - SCLL >= 1 (minimum low period)
 *   - SCLH >= 1 (minimum high period)
 *   - PSC: 0-255 (8-bit register)
 *   - SCLL: 0-255 (8-bit register)
 *   - SCLH: 0-255 (8-bit register)
 *   - Duty cycle ~50% (s/2 split between SCLL and SCLH)
 *
 * Input Parameters:
 *   frequency - Desired I2C frequency in Hz
 *               Common: 100000 (standard), 400000 (fast)
 *   psc       - Pointer to store calculated prescaler value
 *   scll      - Pointer to store calculated SCL low time
 *   sclh      - Pointer to store calculated SCL high time
 *
 * Returned Value:
 *   None (results returned via output pointers)
 *
 * Assumptions/Limitations:
 *   - Input clock is fixed at 48MHz
 *   - Search space limited to s=18..255 (step 2) for performance
 *   - Optimal solution guaranteed within search space
 *   - Execution time: ~10-50us depending on target frequency
 *
 * Example:
 *   For 100kHz (standard mode):
 *   - Best match: PSC=7, SCLL=39, SCLH=41
 *   - Actual: 48MHz / (8 * (39+7+41+5)) = 100.0kHz (exact!)
 *
 *   For 400kHz (fast mode):
 *   - Best match: PSC=1, SCLL=42, SCLH=44
 *   - Actual: 48MHz / (2 * (42+7+44+5)) = 400.0kHz (exact!)
 *
 ****************************************************************************/
static void am67_i2c_calc_prescaler(uint32_t frequency,
                                     uint32_t *psc, uint32_t *scll,
                                     uint32_t *sclh)
{
    uint32_t best_psc = 7;          /* Default for 100kHz */
    uint32_t best_scll = 13;
    uint32_t best_sclh = 15;
    uint32_t best_error = UINT32_MAX;
    uint32_t computed_rate;
    uint32_t error;
    uint32_t p, s;

    /* Search for best prescaler and SCL timing
     * 
     * Loop structure:
     * - Outer: Total SCL period (s) - even values for symmetry
     * - Inner: Prescaler divider (p)
     * 
     * Starting s=18 ensures SCLL and SCLH >= 1 after adjustment
     * Step by 2 maintains 50% duty cycle
     */
    for (s = 18; s < 256; s += 2)  /* FIXED: Was s=14, caused scl_low=0 */
    {
        for (p = 0; p < 256; p++)
        {
            /* Split period approximately 50/50 between low and high
             * Subtract hardware filter delays (7 for low, 5 for high)
             */
            uint32_t scl_low = (s / 2) - 7;
            uint32_t scl_high = (s / 2) - 5;
            
            /* Validate minimum timing requirements */
            if (scl_low < 1 || scl_high < 1)
                continue;

            /* Calculate actual frequency for this configuration
             * Formula: freq = input / ((psc+1) * (scll+7 + sclh+5))
             */
            computed_rate = AM67_I2C_INPUT_CLOCK / (p + 1);
            computed_rate /= scl_low + 7 + scl_high + 5;

            /* Calculate absolute error from target */
            error = (frequency > computed_rate) ?
                    (frequency - computed_rate) :
                    (computed_rate - frequency);

            /* Update best match if this is closer */
            if (error < best_error)
            {
                best_psc = p;
                best_scll = scl_low;
                best_sclh = scl_high;
                best_error = error;

                /* Perfect match - exit early */
                if (error == 0)
                {
                    goto done;
                }
            }
        }
    }

done:
    /* Return best configuration found */
    *psc = best_psc;
    *scll = best_scll;
    *sclh = best_sclh;
}

/****************************************************************************
 * Name: am67_i2c_set_frequency
 *
 * Description:
 *   Configure I2C clock speed with optimal prescaler calculation.
 *
 *   This function:
 *   1. Calculates optimal PSC/SCLL/SCLH values for target frequency
 *   2. Temporarily disables I2C module (required for clock changes)
 *   3. Programs new timing registers
 *   4. Re-enables module if it was previously enabled
 *
 * HARDWARE REQUIREMENT:
 *   I2C module must be disabled (CON.EN=0) when changing clock settings.
 *   This is a hardware restriction of the AM67x I2C controller.
 *
 * Input Parameters:
 *   base      - Physical base address of I2C peripheral
 *   frequency - Target I2C frequency in Hz
 *               Standard: 100000 Hz
 *               Fast: 400000 Hz
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   - Safe to call during active transfer (will temporarily pause)
 *   - Actual frequency may differ slightly from requested
 *   - Changes take effect immediately
 *
 * Side Effects:
 *   - Briefly disables I2C module (~1-2us)
 *   - Any ongoing bit transfer completes before disable takes effect
 *   - No data loss if called between messages
 *
 ****************************************************************************/
static void am67_i2c_set_frequency(uintptr_t base, uint32_t frequency)
{
    uint32_t psc, scll, sclh;
    uint32_t con;

    /* Calculate optimal timing values using brute-force search */
    am67_i2c_calc_prescaler(frequency, &psc, &scll, &sclh);

    /* Save current control register state */
    con = getreg32(AM67_I2C_REG(base, AM67_I2C_CON));
    
    /* Disable I2C module if currently enabled
     * Hardware requirement: clock settings can only change when disabled
     */
    if (con & I2C_CON_EN)
    {
        putreg32(con & ~I2C_CON_EN, AM67_I2C_REG(base, AM67_I2C_CON));
    }

    /* Program new timing values */
    putreg32(psc, AM67_I2C_REG(base, AM67_I2C_PSC));
    putreg32(scll, AM67_I2C_REG(base, AM67_I2C_SCLL));
    putreg32(sclh, AM67_I2C_REG(base, AM67_I2C_SCLH));

    /* Restore previous enable state */
    if (con & I2C_CON_EN)
    {
        putreg32(con, AM67_I2C_REG(base, AM67_I2C_CON));
    }
}

/****************************************************************************
 * Name: am67_i2c_calc_timeout
 *
 * Description:
 *   Calculate dynamic timeout based on transfer size.
 *
 *   Larger transfers need more time to complete. This function computes
 *   a reasonable timeout value based on:
 *   - Base overhead (bus arbitration, START/STOP conditions)
 *   - Per-byte transfer time (depends on I2C frequency)
 *
 * TIMEOUT FORMULA:
 *   timeout = BASE + (total_bytes * PER_BYTE)
 *   
 *   Default values:
 *   - BASE = 100ms (covers initialization and overhead)
 *   - PER_BYTE = 100us (assumes 100kHz I2C clock)
 *
 * RATIONALE:
 *   At 100kHz, each byte takes ~90us to transfer (9 bits * 10us).
 *   Adding margin (100us) accounts for clock stretching and delays.
 *
 * Input Parameters:
 *   msgc - Number of messages in transfer sequence
 *   msgs - Array of message structures
 *
 * Returned Value:
 *   Timeout value in system ticks (ready for nxsem_tickwait)
 *
 * Assumptions/Limitations:
 *   - Assumes worst-case 100kHz operation
 *   - Does not account for actual configured frequency
 *   - Conservative estimate (rarely times out on healthy bus)
 *
 * Example:
 *   64-byte transfer: 100ms + (64 * 0.1ms) = 106.4ms timeout
 *   1-byte transfer:  100ms + (1 * 0.1ms) = 100.1ms timeout
 *
 ****************************************************************************/
static clock_t am67_i2c_calc_timeout(int msgc, struct i2c_msg_s *msgs)
{
    size_t total_bytes = 0;
    int i;

    /* Accumulate total bytes across all messages */
    for (i = 0; i < msgc; i++)
    {
        total_bytes += msgs[i].length;
    }

    /* Base timeout + per-byte component
     * Convert milliseconds and microseconds to system ticks
     */
    return MSEC2TICK(I2C_TIMEOUT_BASE_MS) +
           USEC2TICK(total_bytes * I2C_TIMEOUT_PER_BYTE_US);
}

/****************************************************************************
 * Name: am67_i2c_start_transfer
 *
 * Description:
 *   Initiate a single message transfer in the message sequence.
 *
 *   This function configures the hardware for one message and starts
 *   the transfer. It handles:
 *   - Frequency changes (if msg->frequency differs from current)
 *   - Slave address programming
 *   - Byte count setup
 *   - START/STOP condition control
 *   - Transfer direction (read/write)
 *   - Address mode (7-bit/10-bit)
 *   - Interrupt enable
 *
 * TRANSFER CONTROL FLAGS:
 *   I2C_M_READ    - Receive mode (vs transmit)
 *   I2C_M_NOSTART - Suppress START condition (for repeated START)
 *   I2C_M_NOSTOP  - Suppress STOP condition (for message chaining)
 *   I2C_M_TEN     - 10-bit addressing mode
 *
 * Input Parameters:
 *   priv - Pointer to driver private data structure
 *          Must have msgs, msgidx set to current message
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   - Called with lock held
 *   - Called from task context (am67_transfer) or ISR (for chaining)
 *   - Hardware must be idle before calling
 *   - Message buffer must remain valid until transfer completes
 *
 * Context:
 *   Task or ISR (protected by critical section in caller)
 *
 ****************************************************************************/
static void am67_i2c_start_transfer(FAR struct am67_i2c_priv_s *priv)
{
    uintptr_t base = priv->base;
    struct i2c_msg_s *msg = &priv->msgs[priv->msgidx];
    uint32_t con;
    uint32_t buf_cfg;
    uint32_t ie_flags;
    irqstate_t flags;

    /* Validate message buffer */
    if (!msg->buffer)
    {
        priv->result = -EINVAL;
        priv->inprogress = false;
        nxsem_post(&priv->waitsem);
        return;
    }

    if (msg->frequency > 0 && msg->frequency != priv->frequency)
    {
        am67_i2c_set_frequency(base, msg->frequency);
        priv->frequency = msg->frequency;
    }

    flags = enter_critical_section();

    /* Clear and configure FIFO */
    putreg32(I2C_BUF_TXFIFO_CLR | I2C_BUF_RXFIFO_CLR,
             AM67_I2C_REG(base, AM67_I2C_BUF));

    /* Set FIFO thresholds */
    buf_cfg = I2C_BUF_TXTRSH(I2C_FIFO_THRESHOLD) | 
              I2C_BUF_RXTRSH(I2C_FIFO_THRESHOLD);
    putreg32(buf_cfg, AM67_I2C_REG(base, AM67_I2C_BUF));

    putreg32(msg->addr, AM67_I2C_REG(base, AM67_I2C_SA));
    putreg32(msg->length, AM67_I2C_REG(base, AM67_I2C_CNT));

    con = I2C_CON_EN | I2C_CON_MST;

    if (priv->msgidx == 0 || !(msg->flags & I2C_M_NOSTART))
    {
        con |= I2C_CON_STT;
    }

    if (priv->msgidx == priv->msgc - 1 || !(msg->flags & I2C_M_NOSTOP))
    {
        con |= I2C_CON_STP;
    }

    if (!(msg->flags & I2C_M_READ))
    {
        con |= I2C_CON_TRX;
    }

    if (msg->flags & I2C_M_TEN)
    {
        con |= I2C_CON_XSA;
    }

    /* Choose interrupt mode based on transfer size */
    if (msg->length >= I2C_USE_FIFO_MIN_BYTES)
    {
        /* Use FIFO mode for larger transfers */
        ie_flags = I2C_IE_XDR | I2C_IE_RDR | I2C_IE_ARDY | 
                   I2C_IE_NACK | I2C_IE_AL;
    }
    else
    {
        /* Use byte-by-byte mode for small transfers */
        ie_flags = I2C_IE_XRDY | I2C_IE_RRDY | I2C_IE_ARDY | 
                   I2C_IE_NACK | I2C_IE_AL;
    }

    putreg32(ie_flags, AM67_I2C_REG(base, AM67_I2C_IE));
    putreg32(con, AM67_I2C_REG(base, AM67_I2C_CON));

    leave_critical_section(flags);
}
/****************************************************************************
 * Name: am67_i2c_isr
 *
 * Description:
 *   I2C interrupt service routine.
 *
 *   This ISR handles all I2C events during a transfer:
 *   - Data transmission (XRDY)
 *   - Data reception (RRDY)
 *   - Transfer completion (ARDY)
 *   - Error conditions (NACK, AL)
 *
 * INTERRUPT FLOW:
 *   1. Read and acknowledge status register
 *   2. Check for errors (NACK, arbitration loss)
 *   3. Handle data transfer (XRDY/RRDY)
 *   4. Handle message completion (ARDY)
 *   5. Wake up waiting task if transfer complete
 *
 * ERROR HANDLING:
 *   - NACK: Slave did not acknowledge - return -EIO
 *   - AL (Arbitration Lost): Another master won bus - return -EIO
 *   - Both errors immediately abort transfer
 *
 * TRANSFER COMPLETION:
 *   - ARDY indicates current message complete
 *   - If more messages pending, start next message
 *   - If all messages done, wake up waiting task
 *
 * Input Parameters:
 *   irq     - Interrupt number (not used)
 *   context - Saved processor context (not used)
 *   arg     - Pointer to driver private data (am67_i2c_priv_s)
 *
 * Returned Value:
 *   OK (0) - Interrupt handled successfully
 *
 * Assumptions/Limitations:
 *   - Called in interrupt context
 *   - Must be fast (no blocking operations)
 *   - All register accesses are atomic
 *   - Status bits auto-clear when written with 1
 *
 * Context:
 *   Interrupt context - no blocking allowed
 *
 * Performance:
 *   - Typical execution: 2-5us per interrupt
 *   - One interrupt per byte (non-FIFO mode)
 *   - Additional interrupt for message completion
 *
 ****************************************************************************/
static int am67_i2c_isr(int irq, FAR void *context, FAR void *arg)
{
    FAR struct am67_i2c_priv_s *priv = (FAR struct am67_i2c_priv_s *)arg;
    uintptr_t base = priv->base;
    uint32_t stat;
    uint32_t bufstat;
    struct i2c_msg_s *msg;
    bool transfer_done = false;
    int fifo_level;
    int bytes_to_transfer;
    int i;

    stat = getreg32(AM67_I2C_REG(base, AM67_I2C_STAT));
    putreg32(stat, AM67_I2C_REG(base, AM67_I2C_STAT));

    /* Handle errors first */
    if (stat & (I2C_STAT_NACK | I2C_STAT_AL))
    {
        priv->result = -EIO;
        transfer_done = true;
        goto done;
    }

    if (priv->msgidx >= priv->msgc)
    {
        goto done;
    }

    msg = &priv->msgs[priv->msgidx];

    /* Validate buffer pointer */
    if (!msg->buffer)
    {
        priv->result = -EINVAL;
        transfer_done = true;
        goto done;
    }

    /* Handle TX FIFO Draining (burst write) */
    if ((stat & I2C_STAT_XDR) && !(msg->flags & I2C_M_READ))
    {
        bytes_to_transfer = msg->length - priv->byteidx;
        if (bytes_to_transfer > I2C_FIFO_THRESHOLD)
            bytes_to_transfer = I2C_FIFO_THRESHOLD;

        for (i = 0; i < bytes_to_transfer && priv->byteidx < msg->length; i++)
        {
            putreg32(msg->buffer[priv->byteidx++],
                    AM67_I2C_REG(base, AM67_I2C_DATA));
        }
    }

    /* Handle RX FIFO Threshold (burst read) */
    if ((stat & I2C_STAT_RDR) && (msg->flags & I2C_M_READ))
    {
        bufstat = getreg32(AM67_I2C_REG(base, AM67_I2C_BUFSTAT));
        fifo_level = (bufstat & I2C_BUFSTAT_RXSTAT_MASK) >> 
                     I2C_BUFSTAT_RXSTAT_SHIFT;

        bytes_to_transfer = msg->length - priv->byteidx;
        if (bytes_to_transfer > fifo_level)
            bytes_to_transfer = fifo_level;

        for (i = 0; i < bytes_to_transfer && priv->byteidx < msg->length; i++)
        {
            msg->buffer[priv->byteidx++] = 
                (uint8_t)getreg32(AM67_I2C_REG(base, AM67_I2C_DATA));
        }
    }

    /* Handle byte-by-byte TX (for small transfers) */
    if ((stat & I2C_STAT_XRDY) && !(msg->flags & I2C_M_READ))
    {
        if (priv->byteidx < msg->length)
        {
            putreg32(msg->buffer[priv->byteidx++],
                    AM67_I2C_REG(base, AM67_I2C_DATA));
        }
    }

    /* Handle byte-by-byte RX (for small transfers) */
    if ((stat & I2C_STAT_RRDY) && (msg->flags & I2C_M_READ))
    {
        if (priv->byteidx < msg->length)
        {
            msg->buffer[priv->byteidx++] = 
                (uint8_t)getreg32(AM67_I2C_REG(base, AM67_I2C_DATA));
        }
    }

    /* Handle transfer completion */
    if (stat & I2C_STAT_ARDY)
    {
        priv->msgidx++;
        priv->byteidx = 0;

        if (priv->msgidx < priv->msgc)
        {
            am67_i2c_start_transfer(priv);
        }
        else
        {
            priv->result = OK;
            transfer_done = true;
        }
    }

done:
    if (transfer_done)
    {
        putreg32(0, AM67_I2C_REG(base, AM67_I2C_IE));
        priv->inprogress = false;
        nxsem_post(&priv->waitsem);
    }

    return OK;
}
/****************************************************************************
 * Name: am67_transfer
 *
 * Description:
 *   Perform a sequence of I2C messages with dynamic timeout.
 *
 *   This is the main entry point for I2C communication. It:
 *   1. Validates input parameters
 *   2. Acquires exclusive bus access
 *   3. Calculates appropriate timeout
 *   4. Starts transfer sequence
 *   5. Waits for completion or timeout
 *   6. Handles timeout recovery
 *   7. Returns result to caller
 *
 * MESSAGE SEQUENCE:
 *   Messages are processed sequentially. Each message can have:
 *   - Different slave address
 *   - Different transfer direction (read/write)
 *   - Different length
 *   - START/STOP condition control
 *
 * TIMEOUT HANDLING:
 *   - Timeout is dynamically calculated based on transfer size
 *   - On timeout, hardware is stopped and error returned
 *   - Caller can retry or perform bus reset
 *
 * Input Parameters:
 *   dev   - Pointer to I2C master device structure
 *   msgs  - Array of message structures to process
 *   count - Number of messages in array (must be > 0)
 *
 * Returned Value:
 *   OK (0)        - All messages transferred successfully
 *   -EINVAL       - Invalid parameters (NULL pointers, count <= 0)
 *   -ETIMEDOUT    - Transfer did not complete in time
 *   -EIO          - NACK or arbitration loss
 *   -EBUSY        - Bus busy (should not happen with proper locking)
 *
 * Assumptions/Limitations:
 *   - Messages array must remain valid until function returns
 *   - Each message buffer must be >= message length
 *   - Maximum message count: INT_MAX (practical limit much lower)
 *   - Not safe to call from interrupt context (blocks on mutex)
 *
 * Context:
 *   Task context only (uses blocking mutex and semaphore)
 *
 * POSIX Compliance:
 *   Follows POSIX error code conventions
 *
 * Example:
 *   struct i2c_msg_s msgs[2];
 *   uint8_t reg = 0x10;
 *   uint8_t data[4];
 *   
 *   // Write register address
 *   msgs[0].addr = 0x50;
 *   msgs[0].flags = 0;
 *   msgs[0].buffer = &reg;
 *   msgs[0].length = 1;
 *   
 *   // Read data (repeated START)
 *   msgs[1].addr = 0x50;
 *   msgs[1].flags = I2C_M_READ;
 *   msgs[1].buffer = data;
 *   msgs[1].length = 4;
 *   
 *   ret = am67_transfer(dev, msgs, 2);
 *
 ****************************************************************************/
static int am67_transfer(FAR struct i2c_master_s *dev,
                         FAR struct i2c_msg_s *msgs, int count)
{
    FAR struct am67_i2c_priv_s *priv = (FAR struct am67_i2c_priv_s *)dev;
    int ret;
    int i;
    irqstate_t flags;
    clock_t timeout;

    /* Validate input parameters */
    if (!dev || !msgs || count <= 0 || count > INT_MAX)
    {
        return -EINVAL;
    }

    /* Validate all message buffers before starting */
    for (i = 0; i < count; i++)
    {
        if (!msgs[i].buffer && msgs[i].length > 0)
        {
            return -EINVAL;
        }
    }

    ret = nxmutex_lock(&priv->lock);
    if (ret < 0)
    {
        return ret;
    }

    timeout = am67_i2c_calc_timeout(count, msgs);

    /* Reset semaphore to ensure clean state */
    nxsem_reset(&priv->waitsem, 0);

    flags = enter_critical_section();
    
    priv->msgs = msgs;
    priv->msgc = count;
    priv->msgidx = 0;
    priv->byteidx = 0;
    priv->result = -EBUSY;
    priv->inprogress = true;

    leave_critical_section(flags);

    am67_i2c_start_transfer(priv);

    ret = nxsem_tickwait_uninterruptible(&priv->waitsem, timeout);
    
    if (ret < 0)
    {
        flags = enter_critical_section();
        
        if (priv->inprogress)
        {
            putreg32(0, AM67_I2C_REG(priv->base, AM67_I2C_IE));
            
            uint32_t con = getreg32(AM67_I2C_REG(priv->base, AM67_I2C_CON));
            putreg32(con | I2C_CON_STP, AM67_I2C_REG(priv->base, AM67_I2C_CON));
            
            priv->inprogress = false;
            priv->result = (ret == -ETIMEDOUT) ? -ETIMEDOUT : ret;
        }
        
        leave_critical_section(flags);
    }

    ret = priv->result;
    nxmutex_unlock(&priv->lock);

    return ret;
}
/****************************************************************************
 * Name: am67_reset
 *
 * Description:
 *   GPIO-based bus recovery for stuck I2C slaves.
 *
 *   This function implements I2C bus recovery per I2C specification.
 *   It manually clocks the bus to release slaves that are holding SDA low.
 *
 * RECOVERY PROCEDURE:
 *   1. Stop any ongoing transfer
 *   2. Disable I2C peripheral
 *   3. Configure SCL/SDA as GPIO outputs
 *   4. Manually clock SCL up to 9 times
 *   5. Monitor SDA - stop when it goes high
 *   6. Generate STOP condition
 *   7. Reconfigure pins for I2C function
 *   8. Re-initialize peripheral
 *
 * WHY THIS WORKS:
 *   A stuck slave is typically mid-byte in receive mode, holding SDA low
 *   waiting for more clock pulses. By providing clocks, we allow the slave
 *   to complete its byte reception and release SDA. The STOP condition
 *   then fully resets the slave's state machine.
 *
 * RECOVERY SCENARIOS:
 *   - Slave crashed mid-transfer
 *   - Master reset during transfer
 *   - Glitch caused slave to enter invalid state
 *   - EMI-induced bus lockup
 *
 * Input Parameters:
 *   dev - Pointer to I2C master device structure
 *
 * Returned Value:
 *   OK (0)   - Bus successfully recovered (SDA released)
 *   -EIO     - Recovery failed (SDA still stuck low)
 *
 * Assumptions/Limitations:
 *   - scl_pin and sda_pin must be valid GPIO configurations
 *   - GPIO driver (am67_gpio) must be initialized
 *   - May not work if slave is powered off
 *   - Takes ~1-2ms to complete
 *
 * Context:
 *   Task context (uses mutex, may block briefly)
 *
 * Side Effects:
 *   - Temporarily disables I2C peripheral
 *   - Generates up to 9 clock pulses on SCL
 *   - All I2C configuration preserved and restored
 *
 ****************************************************************************/
#ifdef CONFIG_I2C_RESET
static int am67_reset(FAR struct i2c_master_s *dev)
{
    FAR struct am67_i2c_priv_s *priv = (FAR struct am67_i2c_priv_s *)dev;
    irqstate_t flags;
    uint32_t frequency;
    int clock_count;
    int stretch_count;
    int ret;

    /* Acquire bus lock to prevent concurrent access */
    ret = nxmutex_lock(&priv->lock);
    if (ret < 0)
    {
        return ret;
    }

    flags = enter_critical_section();

    /* Save current frequency to restore later */
    frequency = priv->frequency;

    /* Stop any ongoing transfer */
    if (priv->inprogress)
    {
        putreg32(0, AM67_I2C_REG(priv->base, AM67_I2C_IE));
        priv->inprogress = false;
    }

    /* Disable I2C peripheral completely
     * This releases hardware control of SCL/SDA pins
     */
    putreg32(0, AM67_I2C_REG(priv->base, AM67_I2C_CON));

    leave_critical_section(flags);

    /* Configure SCL and SDA as GPIO outputs with initial high level
     * High level allows reading pin state via GPIO
     */
    am67_gpio_config(priv->scl_pin | GPIO_OUTPUT | GPIO_OUTPUT_ONE);
    am67_gpio_config(priv->sda_pin | GPIO_OUTPUT | GPIO_OUTPUT_ONE);

    /* Release SDA (set high, but with weak pull)
     * If slave is not stuck, SDA will stay high
     */
    am67_gpio_write(priv->sda_pin, 1);
    up_udelay(10);

    /* Clock the bus to release stuck slave
     * Standard allows up to 9 clock pulses for recovery
     */
    ret = -EIO;  /* Assume failure until SDA releases */
    
    for (clock_count = 0; clock_count < I2C_RECOVERY_CLOCK_MAX; clock_count++)
    {
        /* Check if SDA has been released (pulled high)
         * If yes, recovery successful
         */
        if (am67_gpio_read(priv->sda_pin))
        {
            ret = OK;
            break;
        }

        /* Wait for any clock stretching by slave to finish
         * Some slaves may hold SCL low between clock pulses
         */
        for (stretch_count = 0; stretch_count < I2C_RECOVERY_STRETCH_MAX; 
             stretch_count++)
        {
            if (am67_gpio_read(priv->scl_pin))
            {
                /* SCL is high - we can proceed with clock pulse */
                break;
            }
            up_udelay(10);
        }

        /* If clock stretch timeout, give up */
        if (stretch_count >= I2C_RECOVERY_STRETCH_MAX)
        {
            break;
        }

        /* Generate one clock pulse:
         * SCL high -> low -> high
         * Each pulse allows slave to shift out one bit
         */
        am67_gpio_write(priv->scl_pin, 0);
        up_udelay(10);
        am67_gpio_write(priv->scl_pin, 1);
        up_udelay(10);
    }

    /* Generate STOP condition to reset slave state machine
     * STOP condition: SDA low-to-high while SCL is high
     * 
     * Sequence:
     * 1. SDA low (START-like condition)
     * 2. SCL low (prepare for transition)
     * 3. SCL high (set up for STOP)
     * 4. SDA high (STOP condition - rising edge while SCL high)
     */
    am67_gpio_write(priv->sda_pin, 0);
    up_udelay(10);
    am67_gpio_write(priv->scl_pin, 0);
    up_udelay(10);
    am67_gpio_write(priv->scl_pin, 1);
    up_udelay(10);
    am67_gpio_write(priv->sda_pin, 1);
    up_udelay(10);

    /* Reconfigure pins back to I2C peripheral function
     * This restores hardware control of the pins
     */
    am67_gpio_config(priv->scl_pin);
    am67_gpio_config(priv->sda_pin);

    /* Re-initialize I2C peripheral to clean state */
    am67_i2c_hw_reset(priv->base);
    am67_i2c_set_frequency(priv->base, frequency);
    priv->frequency = frequency;

    /* Enable I2C module */
    putreg32(I2C_CON_EN, AM67_I2C_REG(priv->base, AM67_I2C_CON));

    nxmutex_unlock(&priv->lock);
    return ret;
}
#endif

/****************************************************************************
 * Name: am67_setup
 *
 * Description:
 *   Setup I2C peripheral for use.
 *
 *   This function is called by the I2C framework when a client first
 *   accesses the bus. For this driver, all initialization is done in
 *   am67_i2c_initialize(), so this is a no-op.
 *
 * Input Parameters:
 *   dev - Pointer to I2C master device structure
 *
 * Returned Value:
 *   OK (0) - Always succeeds
 *
 ****************************************************************************/
static int am67_setup(FAR struct i2c_master_s *dev)
{
    /* No additional setup needed - already initialized */
    return OK;
}

/****************************************************************************
 * Name: am67_shutdown
 *
 * Description:
 *   Shutdown I2C peripheral for power saving.
 *
 *   This function disables the I2C peripheral to save power when not in
 *   use. It's typically called when all clients have closed the device.
 *
 * POWER SAVINGS:
 *   - Disables peripheral clock
 *   - Disables all interrupts
 *   - Puts module in reset-like state
 *
 * Input Parameters:
 *   dev - Pointer to I2C master device structure
 *
 * Returned Value:
 *   OK (0) - Always succeeds
 *
 * Side Effects:
 *   - Any ongoing transfer is aborted
 *   - Configuration is preserved (will be restored on next setup)
 *
 ****************************************************************************/
static int am67_shutdown(FAR struct i2c_master_s *dev)
{
    FAR struct am67_i2c_priv_s *priv = (FAR struct am67_i2c_priv_s *)dev;
    
    /* Disable all interrupts */
    putreg32(0, AM67_I2C_REG(priv->base, AM67_I2C_IE));
    
    /* Disable I2C module */
    putreg32(0, AM67_I2C_REG(priv->base, AM67_I2C_CON));
    
    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2c_initialize
 *
 * Description:
 *   Initialize AM67 I2C peripheral with reference counting support.
 *
 *   This function sets up a new I2C controller instance:
 *   1. Allocates and initializes driver state structure
 *   2. Initializes synchronization objects (mutex, semaphore)
 *   3. Performs hardware reset
 *   4. Configures default frequency (100kHz standard mode)
 *   5. Attaches and enables interrupt handler
 *   6. Registers with NuttX I2C framework
 *
 * REFERENCE COUNTING:
 *   Multiple clients can use the same I2C bus. The driver maintains
 *   a reference count and only deallocates resources when the last
 *   client calls am67_i2c_uninitialize().
 *
 * INITIALIZATION SEQUENCE:
 *   1. Memory allocation
 *   2. Structure initialization
 *   3. Mutex/semaphore creation
 *   4. Hardware reset
 *   5. Clock configuration
 *   6. Interrupt setup
 *   7. Framework registration
 *
 * Input Parameters:
 *   port    - I2C port number (0-based, for device naming)
 *   base    - Physical base address of I2C registers
 *   irq     - Hardware interrupt number
 *   scl_pin - GPIO configuration for SCL pin (for bus recovery)
 *   sda_pin - GPIO configuration for SDA pin (for bus recovery)
 *
 * Returned Value:
 *   Non-NULL - Pointer to I2C master device structure (success)
 *   NULL     - Initialization failed (memory allocation or IRQ attach failed)
 *
 * Assumptions/Limitations:
 *   - Called during system initialization (before scheduler starts)
 *   - Parameters must be valid (not validated)
 *   - GPIO pins must be already muxed for I2C function
 *   - I2C peripheral clock must be enabled by platform code
 *
 * Context:
 *   Initialization context (typically before tasks start)
 *
 * Example:
 *   struct i2c_master_s *i2c0 = am67_i2c_initialize(
 *       0,                      // Port 0
 *       AM67_I2C0_BASE,        // Register base
 *       AM67_IRQ_I2C0,         // IRQ number
 *       GPIO_I2C0_SCL,         // SCL pin config
 *       GPIO_I2C0_SDA          // SDA pin config
 *   );
 *
 ****************************************************************************/
FAR struct i2c_master_s *am67_i2c_initialize(int port, uintptr_t base, 
                                              int irq, uint32_t scl_pin,
                                              uint32_t sda_pin)
{
    FAR struct am67_i2c_priv_s *priv;
    int ret;

    /* Validate critical parameters */
    if (port < 0 || base == 0 || irq <= 0)
    {
        return NULL;
    }

    /* Allocate driver state structure
     * Using kmm_zalloc ensures all fields are zero-initialized
     */
    priv = kmm_zalloc(sizeof(struct am67_i2c_priv_s));
    if (!priv)
    {
        return NULL;
    }

    /* Initialize driver state structure */
    priv->dev.ops = &g_am67_i2c_ops;  /* Set operations table */
    priv->base = base;
    priv->irq = irq;
    priv->scl_pin = scl_pin;
    priv->sda_pin = sda_pin;
    priv->refs = 0;                   /* No clients yet */
    priv->inprogress = false;
    priv->frequency = 0;              /* Will be set below */

    /* Initialize mutex for exclusive bus access
     * Uses default priority inheritance protocol
     */
    nxmutex_init(&priv->lock);
    
    /* Initialize semaphore for transfer completion signaling
     * Initial count: 0 (task will block on first wait)
     * This is a binary semaphore (counting not needed)
     */
    nxsem_init(&priv->waitsem, 0, 0);
    nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

    /* Reset and configure hardware */
    am67_i2c_hw_reset(base);
    
    /* Set default frequency to 100kHz (I2C standard mode)
     * Can be changed per-transfer via i2c_msg_s.frequency field
     */
    am67_i2c_set_frequency(base, I2C_SPEED_STANDARD);
    priv->frequency = I2C_SPEED_STANDARD;

    /* Enable I2C module
     * Must be done before attaching interrupt to prevent spurious IRQs
     */
    putreg32(I2C_CON_EN, AM67_I2C_REG(base, AM67_I2C_CON));

    /* Attach interrupt handler
     * Pass priv as argument so ISR can access driver state
     */
    ret = irq_attach(irq, am67_i2c_isr, priv);
    if (ret < 0)
    {
        goto err_with_sem;
    }

    /* Enable interrupt at CPU level */
    up_enable_irq(irq);

    /* Register with I2C framework
     * Creates /dev/i2c<port> device node
     * Allows applications to open and use the bus
     */
    ret = i2c_register(&priv->dev, port);
    if (ret < 0)
    {
        goto err_with_irq;
    }

    /* Increment reference count (first client) */
    priv->refs++;

    return &priv->dev;

err_with_irq:
    /* Cleanup on interrupt attach or registration failure */
    up_disable_irq(irq);
    irq_detach(irq);

err_with_sem:
    /* Cleanup on allocation or semaphore failure */
    nxsem_destroy(&priv->waitsem);
    nxmutex_destroy(&priv->lock);
    kmm_free(priv);
    return NULL;
}

/****************************************************************************
 * Name: am67_i2c_uninitialize
 *
 * Description:
 *   Uninitialize I2C peripheral with reference counting.
 *
 *   This function decrements the reference count and only fully
 *   deinitializes the hardware when the count reaches zero.
 *
 * SHUTDOWN SEQUENCE (when refs == 0):
 *   1. Disable and detach interrupt
 *   2. Reset hardware to clean state
 *   3. Destroy synchronization objects
 *   4. Free driver state memory
 *
 * REFERENCE COUNTING:
 *   - Each am67_i2c_initialize() increments refs
 *   - Each am67_i2c_uninitialize() decrements refs
 *   - Hardware only disabled when refs reaches 0
 *   - This allows multiple clients to safely share the bus
 *
 * Input Parameters:
 *   dev - Pointer to I2C master device structure
 *
 * Returned Value:
 *   OK (0)   - Success (reference decremented or fully uninitialized)
 *   -EINVAL  - Invalid parameter or reference count already zero
 *
 * Assumptions/Limitations:
 *   - Must be called same number of times as am67_i2c_initialize()
 *   - Safe to call even if transfers are pending (they will be aborted)
 *   - Not safe to call from interrupt context
 *
 * Context:
 *   Task context (uses mutex, may block)
 *
 ****************************************************************************/
int am67_i2c_uninitialize(FAR struct i2c_master_s *dev)
{
    FAR struct am67_i2c_priv_s *priv = (FAR struct am67_i2c_priv_s *)dev;
    int ret;

    /* Validate parameter */
    if (!priv)
    {
        return -EINVAL;
    }

    /* Acquire lock to safely manipulate reference count */
    ret = nxmutex_lock(&priv->lock);
    if (ret < 0)
    {
        return ret;
    }

    /* Validate reference count */
    if (priv->refs <= 0)
    {
        nxmutex_unlock(&priv->lock);
        return -EINVAL;
    }

    /* Decrement reference count */
    priv->refs--;

    /* Only fully deinitialize when last client releases */
    if (priv->refs == 0)
    {
        /* Disable interrupts at CPU level */
        up_disable_irq(priv->irq);
        
        /* Detach interrupt handler */
        irq_detach(priv->irq);

        /* Reset hardware to clean state */
        am67_i2c_hw_reset(priv->base);

        /* Release mutex before destroying it */
        nxmutex_unlock(&priv->lock);

        /* Destroy synchronization objects */
        nxsem_destroy(&priv->waitsem);
        nxmutex_destroy(&priv->lock);

        /* Free driver state memory */
        kmm_free(priv);
        
        return OK;
    }

    /* Other clients still using the bus - just decrement and return */
    nxmutex_unlock(&priv->lock);
    return OK;
}

