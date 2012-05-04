#pragma once

#include "types.hpp"
#include "universe.hpp"
#include "generic_protocol.hpp"

namespace generic_protocol {

    #pragma pack(push, 1)
    template <typename message_id_type, typename message_len_type>
    struct generic_message_header
    {
        message_id_type msg_id;
        message_len_type msg_len;
        void init_msg(message_id_type id, message_len_type len) {msg_id = id; msg_len = len;}
    };

    template <typename message_id_type, message_id_type message_id, typename message_len_type>
    struct variable_size_message_header : public generic_message_header<message_id_type, message_len_type>
    {
        void init_msg(message_len_type payload_len)
        {
            generic_message_header<message_id_type, message_len_type>::init_msg(message_id, payload_len + sizeof(generic_message_header<message_id_type, message_len_type>));
        }
    };

    template <typename message_type, typename message_id_type, message_id_type message_id, typename message_len_type>
    struct specialized_message_header : public generic_message_header<message_id_type, message_len_type>
    {
        void init_msg() {generic_message_header<message_id_type, message_len_type>::init_msg(message_id, sizeof(message_type));}
    };
    #pragma pack(pop)

    // handles multiple messages per packet, uses ID + Len
    template <typename protocol_type, typename message_id_type, typename message_len_type>
    class generic_packet_handler
    {
    public:
        generic_packet_handler() : current_message(0), payload_len(0), messages_len(0) {}

        void init()
        {
            protocol.init();
            payload_start = protocol.get_payload();
            current_message = reinterpret_cast<message_header_type*>(protocol.get_payload());
            payload_len = protocol.max_payload_len();
        }

        void clear()
        {
            protocol.clear();
            current_message = reinterpret_cast<message_header_type*>(protocol.get_payload());
        }
        
        bool add_byte(u8 byte)
        {
            bool ready = protocol.add_byte(byte);
            if (ready)
            {
                current_message = reinterpret_cast<message_header_type*>(protocol.get_payload());
                payload_len = protocol.get_payload_len();
                messages_len = 0;
            }
            return ready;
        }

        bool has_message() { return current_message != 0; }

        universe::en get_message_id()
        {
            assert(current_message);
            return static_cast<universe::en>(current_message->msg_id);
        }

        template <typename message_type>
        message_type* get_message()
        {
            if (!current_message)
                return 0;
            return static_cast<message_type*>(current_message);
        }

        message_len_type get_message_payload_len()
        {
            return current_message->msg_len - sizeof(message_header_type);
        }

        message_len_type get_max_message_payload_len()
        {
            return max_size<message_len_type>::size - sizeof(message_header_type);
        }

        void next_message()
        {
            if (!current_message)
                return;

            if (current_message->msg_len == 0)
            {
                current_message = 0;
                return;
            }

            messages_len += current_message->msg_len;
            
            if (messages_len >= payload_len)
                current_message = 0;
            else
                current_message = reinterpret_cast<message_header_type*>(payload_start + messages_len);
        }

        typename protocol_type::len_t get_messages_len()
        {
            return messages_len;
        }

        void prepare_packet()
        {
            protocol.prepare_packet(messages_len);
            current_message = reinterpret_cast<message_header_type*>(payload_start);
            messages_len = 0;
        }

        protocol_type& get_protocol() { return protocol; }

    private:
        typedef generic_message_header<message_id_type, message_len_type> message_header_type;
        protocol_type protocol;
        u8* payload_start;
        message_header_type* current_message;
        typename protocol_type::len_t payload_len;
        typename protocol_type::len_t messages_len;
    };
}