/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file prerotator_params.c
 * Parameters prerotation utility.
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 * 2020
 */

/**
 * Minimal rotor speed for eneable prerotation.
 *
 * Set to 0 (or smaller) if it can spin-up itself from zero RPM.
 * If you are helping to spin the rotor manually set minimal RPM
 * when autopilot should increase prerotator power.
 *
 * @min 0
 * @max 1000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROTAT_MIN_RPM, 20.0f);

/**
 * Prerotation speed
 *
 * Set to 0 (or smaller) if it can spin-up itself from zero RPM.
 * If you are helping to spin the rotor manually set minimal RPM
 * when autopilot should increase prerotator power.
 *
 * @min 0
 * @max 1000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROTAT_RAMP_UP, 5.0f);

/**
 * Prerotator target speed
 *
 * Prerotator target RPM
 *
 * @min 0
 * @max 1000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROTAT_TRG_RPM, 100.0f);
/**
 * Prerotator target speed
 *
 * Prerotator target RPM
 *
 * @min 1000
 * @max 2000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROTAT_IDLE, 1400.0f);
