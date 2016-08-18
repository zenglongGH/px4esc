/****************************************************************************
*
*   Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
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

#include "foc.hpp"
#include "svm.hpp"
#include "pid.hpp"
#include "transforms.hpp"

#include <zubax_chibios/config/config.hpp>


namespace foc
{
namespace
{
// Configuration parameters.                             Name                Default     Min       Max

/*
 * Motor profile parameters.
 * All of these values should be provided by the motor manufacturer.
 * If not, automatic identification can be used (see below).
 * Note that most of the parameters are by default assigned invalid values.
 */
os::config::Param<float> g_config_motor_start_current   ("mot.start_curr",      2.0F,    0.1F,    50.0F);  // Ampere
os::config::Param<float> g_config_motor_max_current     ("mot.max_curr",      100.0F,   10.0F,   200.0F);  // Ampere
os::config::Param<float> g_config_motor_max_voltage     ("mot.max_volt",      100.0F,   10.0F,   200.0F);  // Volt
os::config::Param<float> g_config_motor_field_flux      ("mot.phi",             0.0F,    0.0F,    10.0F);  // Weber
os::config::Param<float> g_config_motor_resistance_ab   ("mot.r_ab",            0.0F,    0.0F,   100.0F);  // Ohm
os::config::Param<float> g_config_motor_inductance_ab   ("mot.l_ab",            0.0F,    0.0F,     1.0F);  // Henry
os::config::Param<unsigned> g_config_motor_num_poles    ("mot.num_poles",          0,       0,      200);

/*
 * Auto identification settings.
 * If automatic identification is selected, it will be performed once, and the identified parameters will be stored
 * as the appropriate configuration parameters. Once Auto ID is complete, the auto ID level configuration parameter
 * will be reset to zero automatically.
 */
enum class MotorAutoIDLevel
{
    None,               ///< All motor parameters are user-defined. Auto ID will not be performed.
    Identify_R_L,       ///< Identify and save R_ab and L_ab.
    Identify_R_L_Phi    ///< Identify and save R_ab, L_ab, and Phi. The motor will spin, and it MUST be unloaded!
};

os::config::Param<unsigned> g_config_motor_auto_id_level("mot.auto_id_lvl",        0,       0,        2);

} // namespace



}
