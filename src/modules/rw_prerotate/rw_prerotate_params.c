/****************************************************************************
 *
 * Copyright (c) 2019 ThunderFly s.r.o., Roman Dvorak. All rights reserved.
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
 * @file rw_prerotate_params.c
 * Prerotator parameters.
 *
 * @author Roman Dvorak <romandvorak@mlab.cz>
 */

/**
 * Start-up prerotator speed in percent of PPM output
 *
 * @group Prerotator
 * @unit %
 * @min 0
 * @max 100
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(PREROT_START_UP, 10.0f);

/**
 * Prerotator target speed
 *
 * @group Prerotator
 * @unit RPM
 * @min 50
 * @max 1000
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PREROT_TRG_RPM, 200.0f);


/**
 * Prerotator ramp-up acceleration.
 *
 * @group Prerotator
 * @unit RPM/S
 * @min 0
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(PREROT_ACCEL, 0.5f);
