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

#include <cstdint>
#include <board/board.hpp>
#include <math/math.hpp>
#include <zubax_chibios/util/heapless.hpp>


namespace board
{
/**
 * This namespace contains API to the motor control hardware - PWM, ADC, Driver IC, etc.
 * All functions except init() may be invoked from IRQ context.
 */
namespace motor
{
/**
 * After initialization the driver will enter the inactive mode.
 */
void init();

/**
 * This handle allows the application to command PWM values.
 * The handle will be automatically released as soon as the object is destroyed.
 * When no handle is active, the driver hardware will be automatically deactivated.
 * In inactive mode, all current measurements will be reported as zero.
 * The driver may create its own handles, e.g. during calibration.
 * This semantics is very similar of that of the Android Wake Lock.
 */
class PWMHandle
{
    static unsigned total_number_of_active_handles_;

    bool active_ = false;

public:
    ~PWMHandle() { release(); }

    /**
     * Activates the current handle, activates the driver if not active yet, and sets the PWM outputs.
     * @param abc           PWM values per channel in the range [0, 1].
     */
    void setPWM(const math::Vector<3>& abc);

    /**
     * Deactivates the current handle, deactivates the driver if it was the last active handle,
     * clears PWM outputs to zero.
     * Does nothing if the handle is not active.
     */
    void release();

    /**
     * Returns true if the current handle is the only one active,
     * or if the driver is not active (and therefore the current handle will be unique once activated).
     */
    bool isUnique() const;

    bool isActive() const { return active_; }

    static unsigned getTotalNumberOfActiveHandles() { return total_number_of_active_handles_; }
};

/**
 * This function can be invoked to perform zero offset calibration.
 * It must be guaranteed that during such calibration the motor is NOT spinning,
 * and that no other component will be using the driver while the calibration is in progress.
 * See also @ref isCalibrationInProgress().
 */
void beginCalibration();

/**
 * Always returns false unless @ref beginCalibration() was invoked recently.
 */
bool isCalibrationInProgress();

/**
 * Static PWM parameters; guaranteed to stay constant as long as the firmware is running.
 */
struct PWMParameters
{
    float period = 0;       ///< Second
    float dead_time = 0;    ///< Second
    float upper_limit = 0;  ///< Unitless, (0, 1]
};

/**
 * Meaningful results guaranteed only after initialization.
 */
PWMParameters getPWMParameters();

/**
 * Returns the most recent phase currents sample.
 * May return zeros if there's no active PWM handles.
 */
math::Vector<2> getPhaseCurrentsAB();

/**
 * Returns the power stage voltage, aka VBAT, in volts.
 */
float getInverterVoltage();

/**
 * Immediately deactivates the PWM outputs (shuts down the carrier).
 * Further use of the driver may not be possible.
 * This function can be called from ANY context, e.g. from Hard Fault handler.
 */
void emergency();

/**
 * Temporarily disables IRQ.
 * This function must NEVER be called if any processing is underway, including calibration.
 *
 * This function is implemented in order to permit modification of the embedded flash memory.
 * Flash modification takes at least 0.5 seconds, which is eternity compared to the time scales we work here with.
 *
 * @return True if suspended, False if the current state does not permit suspension.
 */
bool suspend();
void unsuspend();

/**
 * Prints extended status information into stdout.
 */
void printStatus();

/**
 * @ref getStatus().
 */
struct Status
{
    float inverter_temperature = 0.0F;          ///< Kelvin
    float inverter_voltage = 0.0F;              ///< Volt

    float current_sensor_gain = 0.0F;           ///< Volt/Volt

    bool power_ok = false;                      ///< PWRGD
    bool overload = false;                      ///< OCTW
    bool fault = false;                         ///< FAULT

    bool isOkay() const
    {
        const bool bad = (!power_ok) || overload || fault;
        return !bad;
    }

    auto toString() const
    {
        return os::heapless::format("Inverter Temperature: %.0f C\n"
                                    "Inverter Voltage    : %.1f\n"
                                    "Current Sensor Gain : %.1f\n"
                                    "Power OK            : %u\n"
                                    "Overload            : %u\n"
                                    "Fault               : %u",
                                    double(math::convertKelvinToCelsius(inverter_temperature)),
                                    double(inverter_voltage),
                                    double(current_sensor_gain),
                                    power_ok,
                                    overload,
                                    fault);
    }
};

/**
 * Returns immediate status information. @ref Status.
 */
Status getStatus();

/**
 * Board-specific parameter limits.
 */
struct Limits
{
    using Range = math::Range<>;

    struct Set
    {
        Range inverter_temperature;
        Range inverter_voltage;

        auto toString() const
        {
            return os::heapless::format("Inverter Temperature: [%.1f, %.1f]\n"
                                        "Inverter Voltage    : %s",
                                        double(math::convertKelvinToCelsius(inverter_temperature.min)),
                                        double(math::convertKelvinToCelsius(inverter_temperature.max)),
                                        inverter_voltage.toString().c_str());
        }
    };

    Set measurement_range;
    Set safe_operating_area;

    auto toString() const
    {
        return os::heapless::format("Measurement Range:\n%s\n"
                                    "Safe Operating Area:\n%s",
                                    measurement_range.toString().c_str(),
                                    safe_operating_area.toString().c_str());
    }
};

/**
 * @ref Limits
 */
const Limits& getLimits();

/**
 * This external handler is invoked from the HIGHEST PRIORITY IRQ context shortly after the middle of every PWM
 * period, as soon as the corresponding ADC measurements are processed.
 * Steps were taken to minimize the measurement latency, so the application code can rely on that.
 * The PWM update latency is also minimized: the freshly computed PWM values will be applied on the very next period.
 * This IRQ preempts every other process and maskable IRQ handler in the system.
 *
 * @param phase_currents_ab             Instant currents of phases A and B, in Amperes.
 * @param inverter_voltage              Low-pass filtered VBUS voltage of the inverter, in Volts.
 */
extern void handleFastIRQ(const math::Vector<2>& phase_currents_ab,
                          const float inverter_voltage);

/**
 * This external handler is invoked from the SECOND HIGHEST PRIORITY IRQ context shortly after the middle of every
 * N-th PWM period. The N is defined by the driver. This handler is preemptible by the fast IRQ only.
 * When the fast IRQ and the main IRQ are triggered at the same PWM period, the fast IRQ is executed shortly
 * after the middle of the period (see the fast IRQ docs for explanation), and the main IRQ is invoked
 * IMMEDIATELY AFTER THE FAST IRQ IS FINISHED. The processor core does not return to the main context between
 * these two IRQs.
 *
 * Graphically, F - fast IRQ, M - main IRQ:
 *
 *      F       F       F       F       F       F...
 *       MMMMMMMMMMMMMMMMMMM             MMMMMMMM...
 *
 * @param period                        Equals N * @ref getPWMPeriod(), in seconds.
 * @param inverter_voltage              See fast IRQ
 */
extern void handleMainIRQ(const float period);

/**
 * This critical section disables ALL maskable IRQ, including the motor control ones.
 * This is unlike os::CriticalSectionLocker, which keeps them enabled.
 * RTFM: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0553a/CHDBIBGJ.html.
 * The longest use is tracked.
 * It is possible to configure the class to crash the firmware if the critical section is taken for an overly long
 * period time - this is very useful for debugging.
 */
typedef class AbsoluteCriticalSectionLockerImpl_
{
#if defined(DEBUG_BUILD) && DEBUG_BUILD
    static constexpr std::uint32_t MaximumDurationAbsoluteUSec      = 15;
    static constexpr std::uint32_t MaximumDurationWhenActiveUSec    = 3;

    static constexpr std::uint32_t TimerFrequency = STM32_SYSCLK;

    static constexpr std::uint32_t usec2cyc(unsigned usec)
    {
        return std::uint32_t(std::uint64_t(usec) * std::uint64_t(TimerFrequency) / 1000000ULL);
    }

    static constexpr float cyc2sec(std::uint32_t cyc)
    {
        return float(cyc) / float(TimerFrequency);
    }

    static std::uint32_t worst_duration_cyc_;
    static std::uint32_t worst_duration_since_reset_cyc_;

    std::uint32_t entered_at_;      // NOT INITIALIZED, SEE CONSTRUCTOR
#endif

    const volatile bool irq_was_enabled_ = (__get_PRIMASK() & 1) == 0;

public:
    AbsoluteCriticalSectionLockerImpl_()
    {
        __disable_irq();
#if defined(DEBUG_BUILD) && DEBUG_BUILD
        entered_at_ = DWT->CYCCNT;  // Initializing AFTER the critical section is taken, this is important
#endif
        assertLocked();
    }

    ~AbsoluteCriticalSectionLockerImpl_()
    {
        assertLocked();

        if (irq_was_enabled_)
        {
#if defined(DEBUG_BUILD) && DEBUG_BUILD
            /*
             * Absolute worst duration is not updated here because we want to minimize latency.
             * Note that the update is performed BEFORE we exit the critical section, this is very important.
             */
            const std::uint32_t new_duration = DWT->CYCCNT - entered_at_;
            worst_duration_since_reset_cyc_ = std::max(worst_duration_since_reset_cyc_, new_duration);
            worst_duration_cyc_ = std::max(worst_duration_cyc_, new_duration);

            // Driver activation is checked before we exit the critical section - race conditions afoot!
            const bool driver_active = (PWMHandle::getTotalNumberOfActiveHandles() != 0) && !isCalibrationInProgress();
#endif

            __enable_irq();
            assertNotLocked();

#if defined(DEBUG_BUILD) && DEBUG_BUILD
            /*
             * This shim is used only during debugging; when a debugger is attached, it allows to track down all
             * blocks of code that acquire very long critical sections. Note that it is very important to actually
             * check the new value, and not the max, since the max could be updated concurrently from another critical
             * section, which would lead us down a wrong stack trace in the debugger.
             */
            static constexpr auto active_max    = usec2cyc(MaximumDurationWhenActiveUSec);
            static constexpr auto absolute_max  = usec2cyc(MaximumDurationAbsoluteUSec);

            const auto limit = driver_active ? active_max : absolute_max;
            if (new_duration > limit)
            {
                chibios_rt::System::halt(os::heapless::concatenate(
                    "ABS CRITSECT TOO LONG: ",
                    cyc2sec(new_duration) * 1e6F, "us > ",
                    cyc2sec(limit)        * 1e6F, "us").c_str());
            }
#endif
        }
    }

    inline static bool isLocked()
    {
        return (__get_PRIMASK() & 1) == 1;
    }

    inline static void assertNotLocked()
    {
        assert(!isLocked());
    }

    inline static void assertLocked()
    {
        assert(isLocked());
    }

#if defined(DEBUG_BUILD) && DEBUG_BUILD
    static float getWorstDuration()
    {
        return cyc2sec(worst_duration_cyc_);
    }

    static float getWorstDurationSinceReset()
    {
        return cyc2sec(worst_duration_since_reset_cyc_);
    }

    static void resetWorstDuration()
    {
        worst_duration_since_reset_cyc_ = 0;
    }
#endif
} volatile AbsoluteCriticalSectionLocker;

}
}
