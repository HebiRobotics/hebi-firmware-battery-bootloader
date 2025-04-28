/* Battery_Node.h
    
*/

#pragma once

#include "drivers/Battery_CAN.h"
#include "drivers/power_control.h"
#include "Flash_Database.h"
#include "drivers/BQ34Z100_I2C.h"

#include "LED_Controller.h"
#include "Pushbutton_Controller.h"
#include "Beep_Controller.h"

#include "nodes/base_node.h"

namespace hebi::firmware {

class Battery_Node : public protocol::Base_Node {
    enum class NodeState {
        INIT = 0,                   //Startup Value
        LOW_POWER_TIMEOUT = 1,      //Waiting to shutdown
        LOW_POWER_SHUTDOWN = 2,     //Shutdown, waiting for wakeup
        LOW_POWER_DEEP_SLEEP = 3,   //Lower power shutdown (TODO)
        FAULT_SILENT = 4,           //Fault detected, but no user feedback (low power mode)
        FAULT = 5,                  //Fault detected with user feedback
        OUTPUT_ENABLED = 6,         //Normal operation, DSG / CHG enabled
        CHARGE_LOCKOUT = 7,         //Charger detected, DSG / CHG disabled. Lets every battery initialize before CC mode is enabled
        CHARGE_ENABLED = 8,         //Charger detected, DSG disabled, CHG enabled
        CHARGE_PRESENT = 9,         //Charger present but not actively charging, DSG disabled, CHG enabled
        CHARGE_FINISHED = 10,       //Charge finished, DSG disabled, CHG enabled
        NO_BATTERY_DETECTED = 11,   //Bus power on, but no battery comms detected

        //CAN Special states - Outputs off
        ID_ACQUISITION_WAIT = 20,   //Waiting for CAN node id assignment.
        ID_ACQUISITION_TAKE = 21,   //ID acquisition triggered, take next available ID.
        ID_ACQUISITION_DONE = 22,   //ID valid. Wait for acquisition to end.
    };



public:
    Battery_Node(hardware::Flash_Database& database, 
        modules::LED_Controller& led, 
        modules::Pushbutton_Controller& button_ctrl,
        modules::Beep_Controller& beeper,
        protocol::CAN_driver& can_driver,
        hardware::BQ34Z100_I2C& bat_i2c,
        hardware::Power_Control& power_ctrl);

    void update(bool chg_detect, bool polarity_ok, uint16_t v_bat, uint16_t v_ext);

    bool chgEnable() { return chg_enable_; }
    bool dsgEnable() { return dsg_enable_; }
    bool isFaulted() { return state_ == NodeState::FAULT; }
    
    void addTxMessage(protocol::base_msg msg) {
        if(!node_id_valid_) return;

        can_driver_.sendMessage(msg);
    }

    bool shouldSendBatteryData(){
        return send_battery_data_ && node_id_ != protocol::DEFAULT_NODE_ID;
    }

    uint8_t nodeID(){
        return node_id_;
    }

protected:
    static const uint64_t BATTERY_DATA_TIMEOUT_MS = 1100;

    static const uint64_t LOW_POWER_TIMEOUT_MS = 2000;
    static const uint64_t CHARGE_LOCKOUT_MS = 50; /* 50ms */
    static const uint64_t CHARGE_PRES_TIMEOUT_MS = 1 * 1000; /* 1s */
    static const uint64_t CHARGE_FIN_TIMEOUT_MS = 4 * 60 * 60 * 1000; /* 4 hr */

    static const uint16_t CHARGE_VOLTAGE_LOW_THR = 20000; /* mV */
    static const uint16_t CHARGE_FIN_VOLTAGE_THR = 41500; /* mV */
    static const int16_t CHARGE_CURRENT_OFF_THR = 20; /* mA */
    static const int16_t CHARGE_CURRENT_START_THR = 100; /* mA */

    static constexpr float FAULT_BATTERY_UNDERVOLT_THR = 27.; /* V */
    static constexpr float FAULT_BATTERY_DISCONNECT_THR = 24.; /* V */

    static const uint16_t FAULT_CODE_REVERSE_POLARITY = 0x01;
    static const uint16_t FAULT_CODE_UNDERVOLTAGE = 0x02;

    void initNodeID();
    void changeNodeState(NodeState state);
    void changeNodeStateFromISR(NodeState state);
    void changeNodeStateUnsafe(NodeState state);
    void enterLowPowerMode();
    void enterFaultState(uint16_t fault_code);

    void recvd_ctrl_poll_node_id(protocol::ctrl_poll_node_id_msg msg) override;
    void recvd_ctrl_set_node_id(protocol::ctrl_set_node_id_msg msg) override;
    void recvd_ctrl_start_acquisition(protocol::ctrl_start_acquisition_msg msg) override; 
    void recvd_ctrl_stop_acquisition(protocol::ctrl_stop_acquisition_msg msg) override; 
    void recvd_ctrl_read_info(protocol::ctrl_read_info_msg msg) override;

    void recvd_cmd_start_data(protocol::cmd_start_data_msg msg) override;
    void recvd_cmd_set_led(protocol::cmd_set_led_msg msg) override;
    void recvd_cmd_disable_output(protocol::cmd_disable_output_msg msg) override;
    void recvd_cmd_enable_output(protocol::cmd_enable_output_msg msg) override;
    
    bool dsg_enable_ { false };
    bool chg_enable_ { false };

    bool battery_connected_ { false };
    bool send_battery_data_ { false };
    bool node_id_valid_ { false };
    uint8_t node_id_ { protocol::DEFAULT_NODE_ID };
    NodeState state_ { NodeState::INIT };
    uint64_t state_counter_ {0};
    uint16_t fault_code_ {0};

    hardware::battery_data last_battery_data_ {};
    uint64_t last_battery_data_counter_ {0};

    hardware::Flash_Database& database_;
    modules::LED_Controller& led_;
    modules::Beep_Controller& beeper_;
    modules::Pushbutton_Controller& button_;
    protocol::CAN_driver& can_driver_;
    hardware::BQ34Z100_I2C& bat_i2c_;
    hardware::Power_Control& power_ctrl_;
    
    // uint8_t counter_ {0};
};

};