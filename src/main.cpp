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
#include <cstring>

void __late_init();

#include "boot_ctrl.h"
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
hardware::Flash_STM32L4 flash;
hardware::Battery_CAN can;

Bootloader_Node bootloader_node(flash, status_led, button, can);

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

extern "C" {

/* Reset_Handler
    This function is used to initialize the application in a "near-reset" state
    to minimize undesired interactions between the bootloader and application
    code. We check whether or not to jump to the application before ChibiOS
    initialization so that all clocks and peripherals are in their reset state.

    To jump to application, the bootloader should set the "jumptoApplication"
    flag to true and trigger a reset with NVIC_SystemReset(). 
*/
void Reset_Handler(void){
    if(shouldJumpToApplication()){
        uint32_t sp, pc;
        uint32_t reg = 0;
        
        //Read the stack pointer and reset handler addresses
        uint32_t addr = appDataStartAddress();
        memcpy(&sp, (uint32_t*)addr, 4);
        memcpy(&pc, (uint32_t*)(addr + 4), 4);
    
        // Do the actual jump to application!
        // Set main stack pointer, program counter
        asm __volatile__(
            "msr     MSP, %0 \r\n"
            // Switch to the main stack, turn off FPU state storing
            "msr     CONTROL, %1 \r\n"
            "isb \r\n"
            // Go!
            "bx      %2 \r\n" : : "r" (sp), "r" (reg), "r" (pc)
        );
    } else {
        //Run normal bootloader reset handler
        asm __volatile__(
            "b    _crt0_entry"
        );
    }
}
}

/*
 * Application entry point.
 */
int main(void) {
    rgb_led_driver.setColor(255,0,0);
    status_led.teal().fade();

    bool application_valid = false;
    flash.get(hardware::FlashDatabaseKey::APPLICATION_VALID, application_valid);

    //Stay in the bootloader if the application tells us to or if the button is held down
    bool stay_in_bootloader = shouldStayInBootloader();
    if(palReadLine(LINE_PB_WKUP)){
        stay_in_bootloader = true;
    }

    //temporary
    palWriteLine(LINE_DSG_EN, true);
    palWriteLine(LINE_CHG_EN, true);

    while (true) {

        button.update(palReadLine(LINE_PB_WKUP));

        bootloader_node.update();

        status_led.update();

        chThdSleepMilliseconds(1);
        
        if(!stay_in_bootloader && application_valid){
            //Signal to the bootloader to jump to app on reset, and reset
            setJumpToApplication(true);
            NVIC_SystemReset();
        }
    }
}
