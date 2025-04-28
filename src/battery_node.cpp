#include "battery_node.h"
#include "project_info.h"
#include <string.h>

extern "C" {
    #include <ch.h>
    #include <hal.h>
}

using namespace hebi::firmware;

Battery_Node::Battery_Node(hardware::Flash_Database& database, 
    modules::LED_Controller& led, 
    modules::Pushbutton_Controller& button_ctrl,
    modules::Beep_Controller& beeper,
    protocol::CAN_driver& can_driver,
    hardware::BQ34Z100_I2C& bat_i2c,
    hardware::Power_Control& power_ctrl) :
    database_(database), led_(led), button_(button_ctrl), beeper_(beeper),
    can_driver_(can_driver), bat_i2c_(bat_i2c), power_ctrl_(power_ctrl) {
    
    initNodeID();
    changeNodeStateUnsafe(NodeState::LOW_POWER_TIMEOUT);
}

void Battery_Node::initNodeID(){
    node_id_valid_ = database_.get(hardware::FlashDatabaseKey::NODE_ID, node_id_);
    if(!node_id_valid_)
        node_id_ = protocol::DEFAULT_NODE_ID;
}

void Battery_Node::update(bool chg_detect, bool polarity_ok, uint16_t v_bat, uint16_t v_ext) {
    bool has_data = true;
    bool msg_recvd = false;

    while(has_data){
        auto msg_opt = can_driver_.getMessage();
        if(msg_opt.has_value()){
            //Msg_recvd should only be set to true if we receive a valid message from the base node
            msg_recvd |= (tryParseMsg(msg_opt.value()) && msg_opt.value().EID.node_id == 0x00);
        } else {
            has_data = false;
        }

    }

    last_battery_data_counter_++;
    battery_connected_ = bat_i2c_.batteryPresent();

    if(!battery_connected_ && state_ != NodeState::NO_BATTERY_DETECTED)
        changeNodeState(NodeState::NO_BATTERY_DETECTED);

    //Update with latest fuel gauge data
    if(bat_i2c_.hasData()){
        last_battery_data_ = bat_i2c_.getData();
        last_battery_data_counter_ = 0;

        //If we should be sending data, broadcast it
        if(send_battery_data_ && node_id_ != protocol::DEFAULT_NODE_ID){
            can_driver_.sendMessage( protocol::battery_state_msg(
                node_id_, 
                protocol::battery_state_msg::BATTERY_CONNECTED_FLAG,
                last_battery_data_.soc, 
                last_battery_data_.voltage, 
                last_battery_data_.current, 
                last_battery_data_.temperature
            ));
            can_driver_.sendMessage( protocol::battery_state_ext_1_msg(
                node_id_, 
                last_battery_data_.avg_current, 
                last_battery_data_.standby_current, 
                last_battery_data_.status_flags, 
                last_battery_data_.avg_power
            ));
            can_driver_.sendMessage( protocol::battery_state_ext_2_msg(
                node_id_, 
                last_battery_data_.time_to_empty, 
                last_battery_data_.time_to_full, 
                last_battery_data_.capacity_remaining, 
                last_battery_data_.capacity_full
            ));
        }
    } else if (last_battery_data_counter_ > BATTERY_DATA_TIMEOUT_MS){
        last_battery_data_ = {};
        last_battery_data_counter_ = 0;

        if(send_battery_data_ && node_id_ != protocol::DEFAULT_NODE_ID)
            can_driver_.sendMessage( protocol::battery_state_msg(
                node_id_, 
                0, //Battery not connected
                0, 0, 0, 0 //Data invalid
            ));
    }

    //TODO: Battery undervoltage detection

    switch(state_){
    case NodeState::LOW_POWER_TIMEOUT:
        state_counter_++;
        if(msg_recvd) //Reset timeout when we receive a CAN message
            state_counter_ = 0;

        if(!polarity_ok) { //Reverse polarity fault
            enterFaultState(FAULT_CODE_REVERSE_POLARITY);
        } else if (!battery_connected_) {
            changeNodeState(NodeState::NO_BATTERY_DETECTED);
        } else if(button_.enabled()){
            changeNodeState(NodeState::OUTPUT_ENABLED);
        } else if(v_ext > CHARGE_VOLTAGE_LOW_THR) {
            //TODO: Check voltage with ADC, charge state, etc.
            changeNodeState(NodeState::CHARGE_LOCKOUT);
        } else if(state_counter_ == LOW_POWER_TIMEOUT_MS){
            enterLowPowerMode();
        }
        break;
    case NodeState::NO_BATTERY_DETECTED:
        if(battery_connected_)
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        break;
    case NodeState::FAULT_SILENT:
        //TODO: Check battery state, Reset behavior
        if(button_.enabled()){ 
            changeNodeState(NodeState::FAULT);
        }
        break;
    case NodeState::FAULT:
        if(!beeper_.active())
            beeper_.beepFault(200);
        if(!button_.enabled()){ 
            changeNodeState(NodeState::FAULT_SILENT);
        }
        break;
    case NodeState::OUTPUT_ENABLED:
        if(!button_.enabled()){ 
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        }
        break;
    case NodeState::CHARGE_LOCKOUT:
        state_counter_++;
        if(state_counter_ > CHARGE_LOCKOUT_MS)
            changeNodeState(NodeState::CHARGE_ENABLED);
        break;
    case NodeState::CHARGE_ENABLED:
        //TODO: use SOC for charge end detection?
        if(last_battery_data_.current < CHARGE_CURRENT_OFF_THR || last_battery_data_.fullyCharged())
            state_counter_++;
        else
            state_counter_ = 0;

        if(button_.enabled()){ 
            changeNodeState(NodeState::OUTPUT_ENABLED);
        } else if(v_ext < CHARGE_VOLTAGE_LOW_THR) {
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        } else if (state_counter_ >= CHARGE_PRES_TIMEOUT_MS) {
            /* If current has dropped low for at least CHARGE_PRES_TIMEOUT_MS, 
                try to figure out if the battery is fully charged*/
            if(last_battery_data_.fullyCharged())
                changeNodeState(NodeState::CHARGE_FINISHED);
            else
                changeNodeState(NodeState::CHARGE_PRESENT);
        }
        break;
    case NodeState::CHARGE_PRESENT:
        if(last_battery_data_.current < CHARGE_CURRENT_OFF_THR)
            state_counter_++;
        else
            state_counter_ = 0;
        
        if(button_.enabled()){ 
            changeNodeState(NodeState::OUTPUT_ENABLED);
        } else if(v_ext < CHARGE_VOLTAGE_LOW_THR) {
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        } else if (last_battery_data_.current >= CHARGE_CURRENT_START_THR) {
            changeNodeState(NodeState::CHARGE_ENABLED);
        } else if (state_counter_ >= CHARGE_FIN_TIMEOUT_MS) {
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        }
        break;
    case NodeState::CHARGE_FINISHED:
        if(last_battery_data_.current < CHARGE_CURRENT_OFF_THR)
            state_counter_++;
        else
            state_counter_ = 0;
        
        if(button_.enabled()){ 
            changeNodeState(NodeState::OUTPUT_ENABLED);
        } else if(v_ext < CHARGE_VOLTAGE_LOW_THR) {
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        } else if (last_battery_data_.current >= CHARGE_CURRENT_START_THR) {
            changeNodeState(NodeState::CHARGE_ENABLED);
        } else if (state_counter_ >= CHARGE_FIN_TIMEOUT_MS) {
            changeNodeState(NodeState::LOW_POWER_TIMEOUT);
        }
        break;
    case NodeState::ID_ACQUISITION_WAIT:
        if(button_.stateChanged())
            changeNodeState(NodeState::ID_ACQUISITION_TAKE);
    default:
        //Do nothing!
        break;
    }
}

void Battery_Node::changeNodeState(NodeState state){
    chSysLock();
    changeNodeStateUnsafe(state);
    chSysUnlock();
}

void Battery_Node::changeNodeStateFromISR(NodeState state){
    chSysLockFromISR();
    changeNodeStateUnsafe(state);
    chSysUnlockFromISR();
}

void Battery_Node::enterLowPowerMode(){
    changeNodeState(NodeState::LOW_POWER_SHUTDOWN);

    power_ctrl_.enterStop2();
    led_.red();
    led_.update();

    //We will resume from here upon wakeup from stop2
    changeNodeState(NodeState::LOW_POWER_TIMEOUT);
}

void Battery_Node::enterFaultState(uint16_t fault_code){
    changeNodeState(NodeState::FAULT_SILENT);
    fault_code_ = fault_code;
}

void Battery_Node::changeNodeStateUnsafe(NodeState state){
    switch (state){
    case NodeState::LOW_POWER_TIMEOUT: {
        //Can enter from any state
        dsg_enable_ = false;
        chg_enable_ = false;
        led_.off();
        button_.setMode(modules::Pushbutton_Controller::TOGGLE_MODE);
        if(state_ != NodeState::LOW_POWER_SHUTDOWN)
            beeper_.beepOnce(400);

        state_counter_ = 0;
        state_ = NodeState::LOW_POWER_TIMEOUT;
        break;
    }
    case NodeState::LOW_POWER_SHUTDOWN: {
        if(state_ != NodeState::LOW_POWER_TIMEOUT) return;

        dsg_enable_ = false;
        chg_enable_ = false;
        led_.off();

        state_ = NodeState::LOW_POWER_SHUTDOWN;
        break;
    }
    /*case NodeState::LOW_POWER_DEEP_SLEEP: {
        //TODO: Implement this state
        break;
    }*/
    case NodeState::FAULT_SILENT: {
        
        dsg_enable_ = false;
        chg_enable_ = false;
        led_.off();

        state_ = NodeState::FAULT_SILENT;
        break;
    }
    case NodeState::FAULT: {

        dsg_enable_ = false;
        chg_enable_ = false;
        led_.red().blinkFast();

        state_ = NodeState::FAULT;
        break;
    }
    case NodeState::OUTPUT_ENABLED: {
        dsg_enable_ = true;
        chg_enable_ = true;
        led_.green().fade();
        beeper_.beepTwice();

        state_ = NodeState::OUTPUT_ENABLED;
        break;
    }
    case NodeState::CHARGE_LOCKOUT: {
        dsg_enable_ = false;
        chg_enable_ = false;
        led_.orange().blinkFast();
        beeper_.beepTwice();

        state_counter_ = 0;
        state_ = NodeState::CHARGE_LOCKOUT;
        break;
    }
    case NodeState::CHARGE_ENABLED: {
        dsg_enable_ = false;
        chg_enable_ = true;
        led_.orange().blinkFast();
        // beeper_.beepTwice();

        state_counter_ = 0;
        state_ = NodeState::CHARGE_ENABLED;
        break;
    }
    case NodeState::CHARGE_PRESENT: {
        dsg_enable_ = false;
        chg_enable_ = true;
        led_.orange().blink();

        state_counter_ = 0;
        state_ = NodeState::CHARGE_PRESENT;
        break;
    }
    case NodeState::CHARGE_FINISHED: {
        dsg_enable_ = false;
        chg_enable_ = true;
        led_.green().blink();
        beeper_.beepThrice();

        state_counter_ = 0;
        state_ = NodeState::CHARGE_FINISHED;
        break;
    }
    case NodeState::NO_BATTERY_DETECTED: {
        dsg_enable_ = false;
        chg_enable_ = false;
        led_.red().blink();

        state_counter_ = 0;
        state_ = NodeState::NO_BATTERY_DETECTED;
        break;
    }
    /* CAN Special States */
    case NodeState::ID_ACQUISITION_WAIT: {
        if(state_ != NodeState::LOW_POWER_TIMEOUT) return;

        dsg_enable_ = false;
        chg_enable_ = false;
        led_.blue().orange(modules::LED_Controller::BACKGROUND).blink();
        button_.setMode(modules::Pushbutton_Controller::EDGE_DETECT_MODE);
        button_.stateChanged(); //Clear edge detector

        state_ = NodeState::ID_ACQUISITION_WAIT;
        break;
    }
    case NodeState::ID_ACQUISITION_TAKE: {
        if(state_ != NodeState::ID_ACQUISITION_WAIT) return;

        dsg_enable_ = false;
        chg_enable_ = false;
        led_.blue().orange(modules::LED_Controller::BACKGROUND).blinkFast();
        button_.setMode(modules::Pushbutton_Controller::TOGGLE_MODE);

        state_ = NodeState::ID_ACQUISITION_TAKE;
        break;
    }
    case NodeState::ID_ACQUISITION_DONE: {
        if(state_ != NodeState::ID_ACQUISITION_TAKE &&
            state_ != NodeState::LOW_POWER_TIMEOUT) return;

        dsg_enable_ = false;
        chg_enable_ = false;
        led_.blue().fade();
        button_.setMode(modules::Pushbutton_Controller::TOGGLE_MODE);

        state_ = NodeState::ID_ACQUISITION_DONE;
        break;
    }
    default:
        //Invalid, do nothing
        break;
    }
}

void Battery_Node::recvd_ctrl_start_acquisition(protocol::ctrl_start_acquisition_msg msg){
    //Request must be coming from master node
    //Must not already be in acquisition state
    if(msg.EID.node_id != 0x00 || 
        state_ == NodeState::ID_ACQUISITION_WAIT || 
        state_ == NodeState::ID_ACQUISITION_DONE) return; 

    //Go to off state first to ensure proper transition
    changeNodeState(NodeState::LOW_POWER_TIMEOUT);

    /*  If "should_clear_id" is true, every node receiving this message should 
        reset its ID and start the acquisition process.
    */
    if(msg.should_clear_id() && node_id_ != protocol::DEFAULT_NODE_ID) {
        node_id_ = protocol::DEFAULT_NODE_ID;
        database_.put(hardware::FlashDatabaseKey::NODE_ID, node_id_);

        initNodeID();
    }

    /* If we have a valid ID already, skip the acquisition process */
    if(node_id_ == protocol::DEFAULT_NODE_ID)
        changeNodeState(NodeState::ID_ACQUISITION_WAIT);
    else
        changeNodeState(NodeState::ID_ACQUISITION_DONE);
}

void Battery_Node::recvd_ctrl_stop_acquisition(protocol::ctrl_stop_acquisition_msg msg){
    //Request must be coming from master node
    if(msg.EID.node_id != 0x00 || 
        (state_ != NodeState::ID_ACQUISITION_WAIT && 
        state_ != NodeState::ID_ACQUISITION_DONE)) return; 

    //Return to off state
    changeNodeState(NodeState::LOW_POWER_TIMEOUT);
}

void Battery_Node::recvd_cmd_start_data(protocol::cmd_start_data_msg msg){
    //Request must be coming from master node
    if(msg.EID.node_id != 0x00 || node_id_ == protocol::DEFAULT_NODE_ID) return; 

    send_battery_data_ = true; 
}

void Battery_Node::recvd_ctrl_poll_node_id(protocol::ctrl_poll_node_id_msg msg){
    //Request must be coming from master node
    if(msg.EID.node_id != 0x00 || node_id_ == protocol::DEFAULT_NODE_ID) return; 

    can_driver_.sendMessage(protocol::ctrl_poll_node_id_msg(node_id_));
}

void Battery_Node::recvd_ctrl_set_node_id(protocol::ctrl_set_node_id_msg msg){
    if(msg.EID.node_id != node_id_ || state_ != NodeState::ID_ACQUISITION_TAKE) return;

    uint8_t new_node_id = msg.new_node_id();
    if(new_node_id != protocol::DEFAULT_NODE_ID){
        if(new_node_id != node_id_)
            database_.put(hardware::FlashDatabaseKey::NODE_ID, new_node_id);

        initNodeID();

        changeNodeState(NodeState::ID_ACQUISITION_DONE);
    }
}

void Battery_Node::recvd_ctrl_read_info(protocol::ctrl_read_info_msg msg) { 
    if(msg.EID.node_id != node_id_) return;
    
    uint8_t guid[8] = {}; //TODO - fill this in

    if(msg.read_GUID()) can_driver_.sendMessage(protocol::ctrl_guid_msg(node_id_, 0, *(uint64_t*)UID_BASE));
    if(msg.read_elec_type()) can_driver_.sendMessage(protocol::ctrl_elec_type_msg(node_id_, ELECTRICAL_TYPE));
    if(msg.read_HW_type()) can_driver_.sendMessage(protocol::ctrl_hw_type_msg(node_id_, BOARD_TYPE));
    if(msg.read_FW_version()){
        size_t size = strlen(FIRMWARE_REVISION);
        size_t msg_len = protocol::ctrl_fw_version_msg::MSG_LEN_BYTES;
        size_t ind = (size / msg_len) + 1;
        for(size_t i = 0; i < ind; i++)
            can_driver_.sendMessage(protocol::ctrl_fw_version_msg(node_id_, i, FIRMWARE_REVISION + (i*msg_len)));
    }
}

void Battery_Node::recvd_cmd_set_led(protocol::cmd_set_led_msg msg){
    if(msg.EID.node_id != node_id_) return;

    if(msg.enabled())
        led_.enableOverride(msg.R(), msg.G(), msg.B());
    else
        led_.disableOverride();

}

void Battery_Node::recvd_cmd_disable_output(protocol::cmd_disable_output_msg msg){
    if(msg.EID.node_id != node_id_) return;
    
    button_.forceDisabled();
}

void Battery_Node::recvd_cmd_enable_output(protocol::cmd_enable_output_msg msg){
    if(msg.EID.node_id != node_id_) return;
    
    button_.forceEnabled();

}