/**
 * Copyright (c) 2016  Zubax Robotics OU  <info@zubax.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#define CH_CFG_ST_RESOLUTION            32

/*
 * ChibiOS is one of the very few open source RTOS that are fully preemptible and support tickless mode.
 *
 * Fully preemptible means that the OS never disables IRQ whose priority is higher than a certain
 * pre-configured level; therefore these IRQ can be used to perform hard real time tasks with zero latency/jitter
 * imposed by the RTOS. In this application we make heavy use of this feature.
 *
 * A tickless mode is another neat feature that enables the OS to use a free-running counter with programmable
 * alarms which trigger IRQ only when needed, rather than triggering a system tick IRQ at a fixed rate even when
 * it's not needed. I won't go into detail here, this feature is well described in the documentation.
 *
 * So, we have two neat features that have a major caveat: when used together, they turn the RTOS into a fucking
 * minefield. When the RTOS configures a new deadline on the system timer, it basically performs the following
 * steps:
 *
 *  1. Read the current system timer value.
 *  2. Add the time remaining to the deadline.
 *  3. Write the result into the compare register.
 *
 * See, the timer keeps running the whole time the sequence is executed, so it is possible that by the time we
 * reach the step 3, the value obtained at the step 1 is obsolete. Obviously, the designer of the RTOS has taken
 * that into account, and put in place two safeguards:
 *
 *  - The minimum configurable delay is limited, typically to just 2 timer ticks. So if the timer increments in the
 *    process, it's fine. The minimum is defined by the configuration parameter CH_CFG_ST_TIMEDELTA.
 *
 *  - The process is protected by the critical section, which makes it quasiatomic.
 *
 * Now remember that we have hard real time IRQ that can interrupt the RTOS at any moment.
 * See the problem? Took me 2 days...
 *
 * If the OS is interrupted between the steps 1 and 3, it may configure a deadline that is already in the past.
 * The IRQ will never fire until the system timer overflows, which may take days. Until that time, the RTOS will
 * not receive any tick interrupts, which will halt a lot of its functions. For example, if a thread goes to sleep,
 * it will never wake up (until the system timer overflows). Worse yet, if a thread is locked on a non-time related
 * event, such as arrival of data via some communication interface or whatever, it will continue to function
 * correctly, blowing the developer's mind even further.
 *
 * In order to work around the problem, we use the traditional ticked mode.
 *
 * Read the discussion here: http://www.chibios.com/forum/viewtopic.php?f=3&t=3651
 */
#define CH_CFG_ST_FREQUENCY             1000
#define CH_CFG_ST_TIMEDELTA             0

#define PORT_INT_REQUIRED_STACK         4096

#if !defined(__ASSEMBLER__) && !defined(__cplusplus)
extern void fastLowLevelSystemIntegrityCheckHook(void);
#endif
#define CH_CFG_CONTEXT_SWITCH_HOOK(...)     fastLowLevelSystemIntegrityCheckHook()
#define CH_CFG_SYSTEM_TICK_HOOK(...)        fastLowLevelSystemIntegrityCheckHook()

#include <zubax_chibios/sys/chconf_tail.h>
