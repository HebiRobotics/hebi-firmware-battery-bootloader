/* Bootloader_Node.h
    
*/

#pragma once

#include "drivers/Battery_CAN.h"
#include "Flash_Database.h"
#include "Flash_STM32L4.h"
#include "LED_Controller.h"
#include "Pushbutton_Controller.h"
#include "Hash.h"

#include "nodes/base_node.h"

extern "C" {
#include <mbedtls/build_info.h>
#include "mbedtls/aes.h"
#include "mbedtls/md5.h"
}

namespace hebi::firmware {

class Bootloader_Node : public protocol::Base_Node {
public:
    Bootloader_Node(
        hardware::Flash_STM32L4& flash,
        modules::LED_Controller& led, 
        modules::Pushbutton_Controller& button_ctrl,
        protocol::CAN_driver& can_driver);

    void update();

    void addTxMessage(protocol::base_msg& msg) {
        if(!node_id_valid_) return;

        can_driver_.sendMessage(msg);
    }

    uint8_t nodeID(){
        return node_id_;
    }

protected:
    void initNodeID();

    void recvd_ctrl_poll_node_id(protocol::ctrl_poll_node_id_msg& msg) override;
    void recvd_ctrl_read_info(protocol::ctrl_read_info_msg& msg) override;

    void recvd_boot_set_key(protocol::boot_set_key_msg& msg) override;
    void recvd_boot_partition_length(protocol::boot_partition_length_msg& msg) override;
    void recvd_boot_read(protocol::boot_read_msg& msg) override;
    void recvd_boot_write(protocol::boot_write_msg& msg) override;
    void recvd_boot_write_data(protocol::boot_write_data_msg& msg) override;
    void recvd_boot_write_end(protocol::boot_write_end_msg& msg) override;
    void recvd_boot_erase(protocol::boot_erase_msg& msg) override;


    bool node_id_valid_ {false};
    uint8_t node_id_ { protocol::DEFAULT_NODE_ID };

    static const uint16_t MAX_TRANSACTION_LEN = 2048;
    bool read_active_ {false};
    uint8_t read_buffer_[MAX_TRANSACTION_LEN];
    bool write_active_ {false};
    uint8_t write_buffer_[MAX_TRANSACTION_LEN];
    
    uint16_t write_buffer_index_ {0};
    uint16_t write_length_ {0};
    uint16_t write_current_offset_ {0};
    uint16_t write_sequence_num_ {0};
    protocol::partition_t write_partition_ {protocol::partition_t::ALL};

    mbedtls_aes_context aes_ctx;
    mbedtls_md5_context md5_ctx;

    // TLS
    // 32 bytes = 256 bits.
    /* @brief The size of a block, all buffers passed into MbedTLS should be a
     * multiple of this length.
     */
    static const uint8_t AES_KEY_LENGTH = 32;
    
    static const uint8_t AES_KEY_N_PACKETS = AES_KEY_LENGTH / protocol::boot_set_key_msg::MSG_LEN_BYTES;
    static const uint8_t AES_BLOCK_SIZE = 16;
    uint8_t aes_key_[AES_KEY_LENGTH] {};
    bool has_key_ {false};

    /* @brief Varables loaded in from the message being sent over, they need to
     * persist between sessions.
     */
    uint8_t iv[AES_BLOCK_SIZE];
    uint8_t reference_md5[Hash::MD5_SUM_LEN];

    bool iv_is_set;
    bool md5_is_set;

    hardware::Flash_STM32L4& flash_;
    modules::LED_Controller& led_;
    modules::Pushbutton_Controller& button_;
    protocol::CAN_driver& can_driver_;
};

}