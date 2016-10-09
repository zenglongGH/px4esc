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

#include <board/board.hpp>
#include <math/math.hpp>


namespace led_indicator
{
/**
 * LED colored blinking pattern.
 */
struct Pattern
{
    board::RGB color_rgb = board::RGB::Zero();
    std::uint8_t num_blinks = 0;
};

/**
 * Displays colored blinking patterns on the LED.
 */
class Indicator
{
    static constexpr int NumEmptySlotsBetweenPatterns = 3;

    Pattern pattern_;
    int current_slot_ = 0;

    void on()
    {
        board::setRGBLED(pattern_.color_rgb);
    }

    void off()
    {
        board::setRGBLED(board::RGB::Zero());
    }

    void restartPattern()
    {
        current_slot_ = pattern_.num_blinks * 2;
    }

public:
    void setPattern(const Pattern& p)
    {
        const bool needs_restart = p.num_blinks != pattern_.num_blinks;
        pattern_ = p;
        if (needs_restart)
        {
            restartPattern();
        }
    }

    /**
     * This function needs to be invoked at a constant rate.
     * The rate defines the base time frame, i.e. the pause between blinks.
     */
    void onNextTimeFrame()
    {
        if (pattern_.num_blinks > 0)
        {
            current_slot_--;
            if (current_slot_ < 0)
            {
                off();
                if (current_slot_ < -NumEmptySlotsBetweenPatterns)
                {
                    restartPattern();
                }
            }
            else
            {
                (this->*((current_slot_ % 2 == 0) ? &Indicator::on : &Indicator::off))();
            }
        }
        else
        {
            current_slot_ = 0;
            on();
        }
    }
};

}
