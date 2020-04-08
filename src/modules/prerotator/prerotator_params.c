/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o. All rights reserved.
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
 * Parameters of rotor prerotator controller.
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 * @url https://www.thunderfly.cz/
 */

/**
 * Minimal rotor speed for start prerotation.
 *
 * Set to 0 (or smaller) if rotor can spin-up itself from zero RPM.
 * It will start prerotation immediately after arm.
 * If you must to pre-spin the rotor manually set minimal RPM
 * when autopilot should increase prerotator power.
 *
 * @min 0
 * @max 1000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_WAIT_RPM, 20.0f);

/**
 * Time to reach full maximal PWM output
 *
 * This value should ensure that you dont overloading prerotator motor/esc.
 *
 * @min 0
 * @max 1000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_RAMP_T, 10.0f);

/**
 * Minimal PWM output for prerotation start
 *
 *
 * @min 1000
 * @max 2000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_PWM_MIN, 1100.0f);

/**
 * Maximal PWM output of
 *
 * Set to 0 (or smaller) if it can spin-up itself from zero RPM.
 * If you are helping to spin the rotor manually set minimal RPM
 * when autopilot should increase prerotator power.
 *
 * @min 1000
 * @max 2000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_PWM_MAX, 1800.0f);

/**
 * Prerotation speed
 *
 * Set to 0 (or smaller) if it can spin-up itself from zero RPM.
 * If you are helping to spin the rotor manually set minimal RPM
 * when autopilot should increase prerotator power.
 *
 * @min 1500
 * @max 2000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_PWM_IDLE, 1600.0f);

/**
 * Prerotator target speed
 *
 * Prerotator target RPM
 *
 * @min 0
 * @max 1000
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_TRG_RPM, 950.0f);

/**
 * Prerotator target speed
 *
 * Prerotator target RPM
 *
 * @min 5
 * @max 50
 * @group Prerotator
 */
PARAM_DEFINE_FLOAT(PREROT_END_ASPD, 5.0f);
