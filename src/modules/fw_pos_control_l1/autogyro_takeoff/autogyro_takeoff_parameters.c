/****************************************************************************
 *
 *   Copyright (c) 2021, 2022  PX4 Development Team. All rights reserved.
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
 * @file autogyro_takeoff_params.c
 *
 * Parameters for autogyro takeoff
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 */




/**
 * Autogyro takeoff
 *
 * @boolean
 * @group Autogyro
 */
PARAM_DEFINE_INT32(AG_TKOFF, 0);

/**
 * Minimal RPM of rotor for beginning prerotation
 *
 * //@unit rpm
 * @min 0
 * @max 5000.0
 * @decimal 1
 * @increment 1
 * @group Autogyro
 */
PARAM_DEFINE_FLOAT(AG_PROT_MIN_RPM, 100.0);


/**
 * Target RPM of rotor prerotator
 *
 * //@unit rpm
 * @min 0
 * @max 5000.0
 * @decimal 1
 * @increment 1
 * @group Autogyro
 */
PARAM_DEFINE_FLOAT(AG_PROT_TRG_RPM, 1000.0);

/**
 * Flight rotor RPM
 *
 * //@unit rpm
 * @min 0
 * @max 5000.0
 * @decimal 1
 * @increment 1
 * @group Autogyro
 */
PARAM_DEFINE_FLOAT(AG_ROTOR_RPM, 1000.0);


/**
 * Type of prerotator
 *
 * @group Autogyro
 * @value 0 Without prerotator; Prerotation is done with external forward movement (moving platform, winch)
 * @value 1 Without prerotator; On runway
 * @value 2 Not implemented: With electronic prerotator controlled from autopilot, takeoff from moving platform
 * @value 3 Not implemented: With electtronic prerotator controlled from autopilot, takoff from runway
 * @value 10 Not implemented: SITL in flightgear
 */
PARAM_DEFINE_INT32(AG_PROT_TYPE, 0);
