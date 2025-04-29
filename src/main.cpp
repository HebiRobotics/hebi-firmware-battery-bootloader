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

#include "LED_Controller.h"
#include "Pushbutton_Controller.h"
#include "drivers/LED_RGB_PWM1.h"
#include "drivers/Battery_CAN.h"
#include "drivers/Flash_STM32L4.h"

#include "bootloader_node.h"

using namespace hebi::firmware;

hardware::LED_RGB_PWM1 rgb_led_driver;
modules::LED_Controller status_led (rgb_led_driver);
modules::Pushbutton_Controller button (400 /*ms*/, 600 /*ms*/, 200 /*ms*/);
hardware::Flash_STM32L4 database;
hardware::Battery_CAN can;

Bootloader_Node bootloader_node(database, status_led, button, can);

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

        bootloader_node.update();

        status_led.update();

        chThdSleepMilliseconds(1);
    }
}
