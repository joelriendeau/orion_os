#pragma once

#include "types.hpp"
#include "assert.h"

namespace generic_protocol {

// a template to extract the largest size a given unsigned integer can get
template <typename type> struct max_size {static const u32 size = 0x0;};
template <> struct max_size<u8>          {static const u32 size = 0xFF;};
template <> struct max_size<u16>         {static const u32 size = 0xFFFF;};
template <> struct max_size<u32>         {static const u32 size = 0xFFFFFFFF;};

// CRC verifier
template <typename crc_bits, u32 polynomial>
class crc
{
public:
    typedef crc_bits verifier_result_t;
    typedef u8 alignment_req_t;

    void init()
    {
        u32 remainder;
        u32 dividend;
        u8 bit;

        // Compute the remainder of each possible dividend.
        for (dividend = 0; dividend < 256; ++dividend)
        {
            // Start with the dividend followed by zeros.
            remainder = dividend << (width - 8);

            // Perform modulo-2 division, a bit at a time.
            for (bit = 8; bit > 0; --bit)
            {
                // Try to divide the current data bit.
                if (remainder & topbit)
                    remainder = (remainder << 1) ^ polynomial;
                else
                    remainder = (remainder << 1);
            }

            // Store the result into the table.
            crc_table[dividend] = remainder;
        }
    }

    crc_bits compute(u8 const message[], u32 bytes)
    {
        u8 data;
        crc_bits remainder = 0;
        u32 byte;

        // Divide the message by the polynomial, a byte at a time.
        for (byte = 0; byte < bytes; ++byte)
        {
            data = message[byte] ^ (remainder >> (width - 8));
            remainder = crc_table[data] ^ (remainder << 8);
        }

        // The final remainder is the CRC.
        return remainder;
    }

private:
    static const u32 width = 8 * sizeof(crc_bits);
    static const u32 topbit = (1 << (width - 1));
    crc_bits crc_table[256];
};

template <typename Type> struct fletcher_helper {typedef u8 work_type;  static const u16 mask = 0xff;   static const u8 shift = 8;  static const u16 over = 21;};
template <> struct fletcher_helper<u32>         {typedef u16 work_type; static const u16 mask = 0xffff; static const u8 shift = 16; static const u16 over = 360;};

// Fletcher's checksum verifier
template <typename checksum_bits>
class fletcher
{
public:
    typedef checksum_bits verifier_result_t;
    typedef typename fletcher_helper<checksum_bits>::work_type alignment_req_t;

    void init() {}

    checksum_bits compute(u8 const message[], u32 bytes) // optimized algorithm from Wikipedia (http://en.wikipedia.org/wiki/Fletcher's_checksum)
    {
        checksum_bits sum1 = fletcher_helper<checksum_bits>::mask, sum2 = fletcher_helper<checksum_bits>::mask;
        const typename fletcher_helper<checksum_bits>::work_type* data = reinterpret_cast<const typename fletcher_helper<checksum_bits>::work_type*>(message);
        u32 steps = bytes / sizeof(typename fletcher_helper<checksum_bits>::work_type);
        if (bytes % sizeof(typename fletcher_helper<checksum_bits>::work_type))
            ++steps;

        while (steps)
        {
            u32 overflow_steps = steps > fletcher_helper<checksum_bits>::over ? fletcher_helper<checksum_bits>::over : steps; // max amount of adds without overflow
            steps -= overflow_steps;
            do
            {
                sum1 += *data++;
                sum2 += sum1;
            } while (--overflow_steps);
            sum1 = (sum1 & fletcher_helper<checksum_bits>::mask) + (sum1 >> fletcher_helper<checksum_bits>::shift); // will be in the range 1..0x1fffe
            sum2 = (sum2 & fletcher_helper<checksum_bits>::mask) + (sum2 >> fletcher_helper<checksum_bits>::shift);
        }

        sum1 = (sum1 & fletcher_helper<checksum_bits>::mask) + (sum1 >> fletcher_helper<checksum_bits>::shift); // second reduction step to reduce range to 1..0xffff
        sum2 = (sum2 & fletcher_helper<checksum_bits>::mask) + (sum2 >> fletcher_helper<checksum_bits>::shift);
        return sum2 << fletcher_helper<checksum_bits>::shift | sum1;
    }
};

// stub class when no verifier is used
class noop_verifier
{
public:
    typedef u8 verifier_result_t;
    typedef u8 alignment_req_t;
    void init() {}
    verifier_result_t compute(u8 const [], u32) {return 0;}
};

#if defined(_MSC_VER)
    #define DATA_ALIGN(declaration, alignment) __declspec(align(alignment)) declaration
#elif defined(__GNUC__)
    #define DATA_ALIGN(declaration, alignment) declaration __attribute__ ((aligned (alignment)))
#endif

// simple protocol format. start_marker and stop_marker are optional,
//   len can have variable amounts of bits,
//   crc is of variable len and optional as well
// [start_marker][len][payload : [padding][seq_id][user_payload][padding]][crc][stop_marker]
// A NOTE ON OPTIMIZATION :
//  Many of the settings specified when instantiating the template are booleans checked inside the methods at runtime.
//  Alternative implementations could be written with boost's enable_if templates, so correct code path would be chosen
//  at compile time. However, an optimizing compiler should remove all dead code anyway, and the complexity and portability
//  issues of enable_if should make this approach preferable in my opinion.
template <bool use_start_marker = false, bool use_stop_marker = false, bool use_seq_id = false, bool use_verification = false,
          typename len_type = u8, len_type max_len = max_size<len_type>::size,
          typename seq_id_type = u8,
          typename verifier_type = noop_verifier,
          u8 start_marker_val = 0, u8 stop_marker_val = start_marker_val>
class state_machine
{
public:
    typedef len_type len_t;

    void init()
    {
        sequence_id = 0;
        orphan_bytes = 0;
        missed_messages = 0;
        aborted_messages = 0;
        failed_verifications = 0;
        state_receive_start_marker();
        if (use_verification)
            verifier.init();
    }

    void clear()
    {
        state_receive_start_marker();
    }

    len_type max_payload_len()
    {
        len_type size = user_payload_pos;
        if (use_verification)
        {
            size += sizeof(typename verifier_type::verifier_result_t);
            size += worst_payload_padding();
        }
        if (use_stop_marker)
            size++;
        return max_len - size;
    }

    bool add_byte(u8 byte)
    {
        switch (state)
        {
        case receive_start_marker:
            if (start_marker_val != byte)
            {
                ++orphan_bytes;
                return state_receive_start_marker();
            }
            save_byte(byte);
            return state_receive_len();
        case receive_len:
            save_byte(byte);
            return state_receive_len();
        case receive_seq_id:
            save_byte(byte);
            return state_receive_seq_id();
        case receive_user_payload:
            save_byte(byte);
            return state_receive_payload();
        case receive_verification:
            save_byte(byte);
            return state_receive_verification();
        case receive_stop_marker:
            if (stop_marker_val != byte)
            {
                ++aborted_messages;
                return state_receive_start_marker();
            }
            save_byte(byte);
            state_receive_start_marker();
            return true;
        }
        return false;
    }

    u8* get_linear_buffer()
    {
        return linear_buffer;
    }

    u8* get_payload()
    {
        return linear_buffer + user_payload_pos;
    }

    len_type get_payload_len()
    {
        return current_len;
    }

    len_type get_packet_len()
    {
        len_type size = user_payload_pos;
        len_type len = reinterpret<len_type>(linear_buffer + len_pos);
        size += len;
        if (use_verification)
        {
            size += sizeof(typename verifier_type::verifier_result_t);
            if (may_need_realignment)
                size += payload_padding(len);
        }
        if (use_stop_marker)
            size++;
        return size;
    }

    void prepare_packet(len_type len)
    {
        if (use_start_marker)
            linear_buffer[0] = start_marker_val;
        reinterpret<len_type>(linear_buffer + len_pos, len);
        if (use_seq_id)
            reinterpret<seq_id_type>(linear_buffer + payload_pos, sequence_id++);
        u32 post_payload_pos = user_payload_pos + len;
        if (may_need_realignment)
        {
            u32 padding = payload_padding(len);
            for (u32 j = 0; j < padding; j++)
                linear_buffer[post_payload_pos++] = 0;
        }
        if (use_verification)
            reinterpret<typename verifier_type::verifier_result_t>(linear_buffer + post_payload_pos, verifier.compute(linear_buffer + payload_pos, len + seq_id_size));
        if (use_stop_marker)
        {
            if (use_verification)
                linear_buffer[post_payload_pos + sizeof(typename verifier_type::verifier_result_t)] = stop_marker_val;
            else
                linear_buffer[post_payload_pos] = stop_marker_val;
        }
        
        assert(get_packet_len() <= max_len);
    }

    void get_stats(u32& orphan, u32& missed, u32& aborts, u32& failed_ver)
    {
        orphan = orphan_bytes;
        missed = missed_messages;
        aborts = aborted_messages;
        failed_ver = failed_verifications;
    }

private:
    enum state_en
    {
        receive_start_marker,
        receive_len,
        receive_seq_id,
        receive_user_payload,
        receive_verification,
        receive_stop_marker,
    };

    len_type payload_padding(u32 len)
    {
        if (may_need_realignment)
        {
            len_type offset = (user_payload_pos + len) % sizeof(typename verifier_type::alignment_req_t);
            if (offset != 0)
                return sizeof(typename verifier_type::alignment_req_t) - offset;
        }
        return 0;
    }

    len_type worst_payload_padding()
    {
        if (may_need_realignment)
            return sizeof(typename verifier_type::alignment_req_t);
        return 0;
    }

    template <typename t>
    t reinterpret(u8* pos)
    {
        t res;
        for (u8 i = 0; i < sizeof(t); i++)
            *(reinterpret_cast<u8*>(&res)+i) = *(pos+i);
        return res;
    }

    template <typename t>
    void reinterpret(u8* pos, t set)
    {
        for (u8 i = 0; i < sizeof(t); i++)
            *(pos+i) = *(reinterpret_cast<u8*>(&set)+i);
    }

    void save_byte(u8 byte)
    {
        linear_buffer[current_pos] = byte;
    }

    bool state_receive_start_marker()
    {
        current_pos = 0;
        if (use_start_marker)
            state = receive_start_marker;
        else
            state = receive_len;
        return false;
    }

    bool state_receive_len()
    {
        ++current_pos;
        if (current_pos < payload_pos)
            state = receive_len;
        else
        {
            current_len = reinterpret<len_type>(linear_buffer + len_pos);
            if (current_len > max_len)
            {
                ++aborted_messages;
                return state_receive_start_marker();
            }
            cached_post_payload_pos = user_payload_pos + current_len + payload_padding(current_len);
            if (use_seq_id)
                state = receive_seq_id;
            else
                state = receive_user_payload;
        }
        return false;
    }

    bool state_receive_seq_id()
    {
        ++current_pos;
        if (current_pos < user_payload_pos)
            state = receive_seq_id;
        else
        {
            seq_id_type current_sequence_id = reinterpret<seq_id_type>(linear_buffer + payload_pos);
            if (current_sequence_id != sequence_id && sequence_id != 0)
                missed_messages++;
            sequence_id = current_sequence_id + 1;
            state = receive_user_payload;
        }
        return false;
    }

    bool state_receive_payload()
    {
        ++current_pos;
        if (current_pos < cached_post_payload_pos)
            return false;

        if (use_verification)
        {
            cached_post_verification_pos = cached_post_payload_pos + sizeof(typename verifier_type::verifier_result_t);
            state = receive_verification;
        }
        else if (use_stop_marker)
            state = receive_stop_marker;
        else
        {
            state_receive_start_marker();
            return true;
        }
        return false;
    }

    bool state_receive_verification()
    {
        ++current_pos;
        if (current_pos < cached_post_verification_pos)
            state = receive_verification;
        else
        {
            typename verifier_type::verifier_result_t ver_result = verifier.compute(linear_buffer + payload_pos, current_len + seq_id_size);
            typename verifier_type::verifier_result_t ver_expected = reinterpret<typename verifier_type::verifier_result_t>(linear_buffer + cached_post_payload_pos);
            if (ver_result != ver_expected)
            {
                ++aborted_messages;
                ++failed_verifications;
                return state_receive_start_marker();
            }
            if (use_stop_marker)
                state = receive_stop_marker;
            else
            {
                state_receive_start_marker();
                return true;
            }
        }
        return false;
    }

    static const u32 len_pos = (use_start_marker) ? sizeof(u8) : 0;
    static const u32 unaligned_payload_pos = len_pos + sizeof(len_type);
    static const bool may_need_realignment = use_verification && (sizeof(typename verifier_type::alignment_req_t) != 1);
    static const u32 payload_pos = (may_need_realignment)
        ? ( (unaligned_payload_pos != (unaligned_payload_pos % sizeof(typename verifier_type::alignment_req_t)))
            ? unaligned_payload_pos - (unaligned_payload_pos % sizeof(typename verifier_type::alignment_req_t)) + sizeof(typename verifier_type::alignment_req_t)
            : unaligned_payload_pos )
        : unaligned_payload_pos;
    static const u32 seq_id_size = (use_seq_id) ? sizeof(seq_id_type) : 0;
    static const u32 user_payload_pos = payload_pos + seq_id_size;

    state_en state;
    seq_id_type sequence_id;
    u32 current_pos;
    len_type current_len;
    u32 cached_post_payload_pos;
    u32 cached_post_verification_pos;

    // all the state could be defined in a helper templated struct to include only the data we need depending on configuration
    u32 orphan_bytes;
    u32 missed_messages;
    u32 aborted_messages;
    u32 failed_verifications;

    verifier_type verifier;
    DATA_ALIGN(u8 linear_buffer[max_len], 4); // this needs to be aligned for the worst case
};

}