#include "bootloader_node.h"
#include "project_info.h"
#include <string.h>

extern "C" {
#include <mbedtls/build_info.h>
}

using namespace hebi::firmware;
using namespace hebi::firmware::protocol;

Bootloader_Node::Bootloader_Node(hardware::Flash_STM32L4& flash, 
    modules::LED_Controller& led, 
    modules::Pushbutton_Controller& button_ctrl,
    protocol::CAN_driver& can_driver) :
    flash_(flash), led_(led), button_(button_ctrl),
    can_driver_(can_driver) {
    
    initNodeID();
    has_key_ = flash_.get(hardware::FlashDatabaseKey::AES_KEY, aes_key_);
}

void Bootloader_Node::initNodeID(){
    node_id_valid_ = flash_.get(hardware::FlashDatabaseKey::NODE_ID, node_id_);

    //temporary...
    if(!node_id_valid_){
        node_id_ = 0x01;
        flash_.put(hardware::FlashDatabaseKey::NODE_ID, node_id_);
    }
}

void Bootloader_Node::update() {
    bool has_data = true;
    bool msg_recvd = false;

    while(has_data){
        chSysLock();
        auto msg_opt = can_driver_.getMessage();
        chSysUnlock();
        if(msg_opt.has_value()){
            //Msg_recvd should only be set to true if we receive a valid message from the base node
            msg_recvd |= (tryParseMsg(msg_opt.value()) && msg_opt.value().EID.node_id == 0x00);
        } else {
            has_data = false;
        }

    }

    // can_driver_.sendMessage(protocol::ctrl_poll_node_id_msg(node_id_));
    // can_driver_.sendMessage(protocol::ctrl_poll_node_id_msg(node_id_));
    // can_driver_.sendMessage(protocol::ctrl_poll_node_id_msg(node_id_));
    // can_driver_.sendMessage(protocol::ctrl_poll_node_id_msg(node_id_));
}

void Bootloader_Node::recvd_ctrl_poll_node_id(protocol::ctrl_poll_node_id_msg& msg){
    //Request must be coming from master node
    if(msg.EID.node_id != 0x00 || node_id_ == protocol::DEFAULT_NODE_ID) return; 

    can_driver_.sendMessage(protocol::ctrl_poll_node_id_msg(node_id_));
}

void Bootloader_Node::recvd_ctrl_read_info(protocol::ctrl_read_info_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;

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
    if(msg.read_FW_mode()) can_driver_.sendMessage(protocol::ctrl_fw_mode_msg(node_id_, true));
}

void Bootloader_Node::recvd_boot_set_key(protocol::boot_set_key_msg& msg) { 
    if(msg.EID.node_id != node_id_ || msg.index() >= AES_KEY_N_PACKETS) return;
    
    uint8_t offset = msg.index() * protocol::boot_set_key_msg::MSG_LEN_BYTES;
    for (uint8_t ind = 0; ind < AES_KEY_LENGTH; ind++)
        aes_key_[ind + offset] = msg.data8[ind];
    
    if(msg.index() == 0){
        has_key_ = false;
        //TODO: Clear old database entry?
    } else if(msg.index() == (AES_KEY_N_PACKETS - 1)){
        flash_.put(hardware::FlashDatabaseKey::AES_KEY, aes_key_);
        has_key_ = true;
    }
}

void Bootloader_Node::recvd_boot_partition_length(protocol::boot_partition_length_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;

    //Get partition size based on ID
    auto partition = static_cast<hardware::Flash_STM32L4::Partition>(msg.partition());
    auto size = flash_.getSize(partition);

    //Return partition size
    can_driver_.sendMessage(
        protocol::boot_partition_length_msg(
            node_id_, size, 
            static_cast<protocol::partition_t>(partition)
    ));
}

void Bootloader_Node::recvd_boot_read(protocol::boot_read_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;

    //Error out if a read is active!
    if(read_active_){
        can_driver_.sendMessage(boot_read_end_msg(
                node_id_, msg.offset(), 
                msg.length(), msg.partition(), 
                status_t::ERROR
        ));
        return;
    }
    read_active_ = true;

    //Read flash data
    auto partition = static_cast<hardware::Flash_STM32L4::Partition>(msg.partition());
    auto read_length = (msg.length() < MAX_TRANSACTION_LEN) ? msg.length() : MAX_TRANSACTION_LEN;
    
    auto result = flash_.read(partition, msg.offset(), read_buffer_, read_length);
    
    if(result != hardware::Flash_STM32L4::Status::COMPLETE){
        can_driver_.sendMessage(boot_read_end_msg(
                node_id_, msg.offset(), 
                msg.length(), msg.partition(), 
                status_t::ERROR
        ));
        read_active_ = false;
        return;
    }

    //If the read was successful, split the data into packets
    uint8_t n_packets = (read_length / boot_read_data_msg::MSG_LEN_BYTES) + 1;
    for(uint8_t index = 0; index < n_packets; index++){
        //Return partition size
        can_driver_.sendMessage(boot_read_data_msg(
                node_id_, index,
                read_buffer_ + (index * boot_read_data_msg::MSG_LEN_BYTES)
        ));
    }

    //Send end of transaction message
    can_driver_.sendMessage(boot_read_end_msg(
        node_id_, msg.offset(), 
        msg.length(), msg.partition(), 
        status_t::OK
    ));

    read_active_ = false;
}

void Bootloader_Node::recvd_boot_write(protocol::boot_write_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;

    //Check for error conditions
    if(!has_key_ || write_active_ || 
       (msg.sequence_number() != 0 && msg.sequence_number() != (write_sequence_num_ + 1)) ||
        msg.partition() != partition_t::APPLICATION ){

        can_driver_.sendMessage(boot_write_end_msg(
                node_id_, msg.length(), 
                msg.sequence_number(), msg.partition(), 
                status_t::ERROR
        ));
        return;
    }


    write_length_ = (msg.length() < MAX_TRANSACTION_LEN) ? msg.length() : MAX_TRANSACTION_LEN;;
    write_sequence_num_ = msg.sequence_number();
    write_partition_ = msg.partition();
    write_buffer_index_ = 0;

    if(write_sequence_num_ == 0){
        //Reset state
        write_current_offset_ = 0;

        // //Erase application on first message
        // auto result = flash_.erase(static_cast<hardware::Flash_STM32L4::Partition>(msg.partition()));
        
        // //Check for erase error
        // if(result != hardware::Flash_STM32L4::Status::COMPLETE){
        //     can_driver_.sendMessage(boot_write_end_msg(
        //         node_id_, msg.length(), 
        //         msg.sequence_number(), msg.partition(), 
        //         status_t::ERROR
        //     ));
        //     return;
        // }

        // Re-initialize contexts.
        iv_is_set = false;
        md5_is_set = false;

        mbedtls_aes_free(&aes_ctx);
        mbedtls_aes_init(&aes_ctx);

        mbedtls_md5_free(&md5_ctx);
        mbedtls_md5_init(&md5_ctx);
        mbedtls_md5_starts(&md5_ctx);

        //Initialize mbedtls
        mbedtls_aes_setkey_dec(
                        &aes_ctx,
                        aes_key_,
                        AES_KEY_LENGTH * 8 /* bits per byte */);
    }

    write_active_ = true;
}

void Bootloader_Node::recvd_boot_write_data(protocol::boot_write_data_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;
    
    //Error out if a write isn't active!
    if(!write_active_){
        can_driver_.sendMessage(boot_write_end_msg(
            node_id_, write_length_, 
            write_sequence_num_, write_partition_, 
            status_t::ERROR
        ));
        return;
    }

    for(uint8_t ind = 0; ind < boot_write_data_msg::MSG_LEN_BYTES; ind++){
        write_buffer_[write_buffer_index_ + ind] = msg.data8[ind];
    }

    write_buffer_index_ += boot_write_data_msg::MSG_LEN_BYTES;
}

void Bootloader_Node::recvd_boot_write_end(protocol::boot_write_end_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;

    //Error out if a write isn't active!
    if(!write_active_ || 
        msg.length() != write_length_ ||
        msg.partition() != write_partition_ ||
        msg.sequence_number() != write_sequence_num_ ||
        write_buffer_index_ < msg.length()){
        can_driver_.sendMessage(boot_write_end_msg(
            node_id_, write_buffer_index_, 
            write_sequence_num_, write_partition_, 
            status_t::ERROR
        ));
        write_active_ = false;
        return;
    }

    auto partition = static_cast<hardware::Flash_STM32L4::Partition>(msg.partition());
    bool success = true;

    if (msg.length() % AES_BLOCK_SIZE != 0) {
        success = false;
        //TODO Error
    } else {
        for (unsigned int i = 0; i < msg.length(); i += AES_BLOCK_SIZE) {
            if (!iv_is_set) {
                // Copy the IV
                for (unsigned int j = 0; j < AES_BLOCK_SIZE; ++j) {
                    iv[j] = write_buffer_[i + j];
                }
                iv_is_set = true;
            } else if (!md5_is_set) {
                // Copy the MD5 Sum
                // At the moment this relies on the MD5 sum being
                // equivalent to the AES_BLOCK_SIZE.  Whenever that
                // changes, you need to encode the sum as a multiple of
                // AES_BLOCK_SIZE, since that's the padding guarantee
                // we've put into place (always padded to
                // AES_BLOCK_SIZE).
                for (unsigned int j = 0; j < Hash::MD5_SUM_LEN; ++j) {
                    reference_md5[j] = write_buffer_[i + j];
                }
                md5_is_set = true;
            } else {
                // Decrypt the data, then write it out.
                unsigned char decrypted[AES_BLOCK_SIZE];

                mbedtls_aes_crypt_cbc(
                        &aes_ctx,
                        MBEDTLS_AES_DECRYPT,
                        AES_BLOCK_SIZE,
                        iv,
                        write_buffer_ + i,
                        decrypted);

                mbedtls_md5_update(&md5_ctx, decrypted, AES_BLOCK_SIZE);
                // success &= loader.writeCode(decrypted, AES_BLOCK_SIZE);
                success &= (flash_.program(partition, write_current_offset_, decrypted, AES_BLOCK_SIZE) 
                    == hardware::Flash_STM32L4::Status::COMPLETE);
                
                write_current_offset_ += AES_BLOCK_SIZE;
            }
        }
    }

    //Valid write at this point
    // auto result = flash_.program(partition, write_current_offset_, write_buffer_, msg.length());

    //Flash program was successful, increment offset
    // write_current_offset_ += msg.length();
    // write_active_ = false;

    //If this is the final packet, finalize everything
    if(msg.status() == status_t::FINAL_PACKET){
        flash_.put(hardware::FlashDatabaseKey::APPLICATION_VALID, true);

        unsigned char computed_md5[Hash::MD5_SUM_LEN];
        mbedtls_md5_finish(&md5_ctx, computed_md5);

        //Save Hash
        flash_.put(hardware::FlashDatabaseKey::APPLICATION_HASH, computed_md5);

        // Check the output md5s.
        bool is_equal = true;

        for (size_t i = 0; i < Hash::MD5_SUM_LEN; ++i) {
            is_equal &= computed_md5[i] == reference_md5[i];
        }

        if (!is_equal) {
            // This will happen when the decrypted data doesn't match the
            // hash.  99% of the time, this is because the AES keys don't
            // match, so we've decrypted garbage.  This also checks overall
            // message integrity.
            success = false;

            //Erase the application
            flash_.erase(hardware::Flash_STM32L4::Partition::APPLICATION);
            flash_.put(hardware::FlashDatabaseKey::APPLICATION_VALID, false);
        }
    }

    if(!success){
        //Send error response
        can_driver_.sendMessage(boot_write_end_msg(
            node_id_, write_buffer_index_, 
            write_sequence_num_, write_partition_, 
            status_t::ERROR
        ));
    } else {
        //Send success response
        can_driver_.sendMessage(boot_write_end_msg(
            node_id_, write_buffer_index_, 
            write_sequence_num_, write_partition_, 
            status_t::OK
        ));
    }

    write_active_ = false;
    return;
}

void Bootloader_Node::recvd_boot_erase(protocol::boot_erase_msg& msg) { 
    if(msg.EID.node_id != node_id_) return;

    bool success = true;
    switch (msg.partition()) {
        // Do nothing.
    case partition_t::ALL:
        break;

        // Erase application code
    case partition_t::APPLICATION:
        flash_.erase(hardware::Flash_STM32L4::Partition::APPLICATION);
        flash_.put(hardware::FlashDatabaseKey::APPLICATION_VALID, false);
        break;

        // Erase database
    case partition_t::DATABASE: {

        //Backup important values
        bool app_valid = false;
        flash_.get(hardware::FlashDatabaseKey::APPLICATION_VALID, app_valid);
        bool has_hash = flash_.get(hardware::FlashDatabaseKey::APPLICATION_HASH, reference_md5);

        success = (flash_.erase(hardware::Flash_STM32L4::Partition::DATABASE)
                    == hardware::Flash_STM32L4::Status::COMPLETE);

        if (success && has_key_) {
            success &= flash_.put(hardware::FlashDatabaseKey::AES_KEY, aes_key_);
        }

        if (success && has_hash) {
            success &= flash_.put(hardware::FlashDatabaseKey::APPLICATION_HASH, reference_md5);
        }

        if (success) {
            success &= flash_.put(hardware::FlashDatabaseKey::APPLICATION_VALID, app_valid);
        }
        break;
    }

    default:
        success = false;
        break;

    }

    if(!success){
        //Send error response
        can_driver_.sendMessage(boot_erase_msg(
            node_id_, msg.partition(), 
            status_t::ERROR
        ));
    } else {
        //Send success response
        can_driver_.sendMessage(boot_erase_msg(
            node_id_, msg.partition(), 
            status_t::OK
        ));
    }
}

