/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

extern "C" {
#include <ch.h>
#include <hal.h>
void __late_init();
}

#include "Beep_Controller.h"
#include "LED_Controller.h"
#include "Pushbutton_Controller.h"
#include "drivers/LED_RGB_PWM1.h"
#include "drivers/Beeper_PWM16.h"
#include "drivers/Battery_CAN.h"
#include "drivers/BQ34Z100_I2C.h"
#include "drivers/Flash_STM32L4.h"
#include "drivers/power_control.h"
#include "drivers/COMP_STM32L4.h"
#include "drivers/ADC_control.h"

#include "battery_node.h"

#include "Driver.h"
#include <array>
#include <vector>

using namespace hebi::firmware;


const static I2CConfig I2C_BATTERY_CONFIG = {
    0x00000E14, //this is generated using STM32CubeMX
    0,
    0
};

hardware::Beeper_PWM16 beeper_driver(4000 /*4kHz*/);
modules::Beep_Controller beeper (beeper_driver);
hardware::LED_RGB_PWM1 rgb_led_driver;
modules::LED_Controller status_led (rgb_led_driver);
hardware::COMP_STM32L4 comp;
hardware::ADC_control adc;

modules::Pushbutton_Controller button (400 /*ms*/, 600 /*ms*/, 200 /*ms*/);

hardware::Flash_STM32L4 database;

hardware::Battery_CAN can;
hardware::BQ34Z100_I2C battery_i2c(&I2CD1, I2C_BATTERY_CONFIG);

std::vector<hardware::Driver *> drivers{&beeper_driver, &rgb_led_driver, &can, &battery_i2c, &comp, &adc};
hardware::Power_Control power_ctrl(drivers);

Battery_Node battery_node(database, status_led, button, beeper, can, battery_i2c, power_ctrl);

/**
 * @brief Initializes hal and ChibiOS
 *
 * @note This function overrides the definition in crt0.c
 *
 * @note This function is called before any static constructors
 *
 * @return Void
 */
void __late_init() {
    /*
    * System initializations.
    * - HAL (Hardware Abstraction Layer) initialization, this initializes 
    *   the configured device drivers and performs the board-specific 
    *   initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();
}

/*
 * Application entry point.
 */
int main(void) {

    // if(!power_ctrl.wakeFromStandby()){
    //     power_ctrl.enterStop2();
    // } else {
    //     power_ctrl.clearStandby();
    // }

    rgb_led_driver.setColor(255,0,0);

    while (true) {

        button.update(palReadLine(LINE_PB_WKUP));

        battery_node.update(comp.output_comp1(), comp.output_comp2(), adc.v_bat(), adc.v_ext());

        status_led.update();
        beeper.update();

        palWriteLine(LINE_DSG_EN, battery_node.dsgEnable());
        palWriteLine(LINE_CHG_EN, battery_node.chgEnable());

        chThdSleepMilliseconds(1);
    }
}
