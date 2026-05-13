/*
 * This file is part of GB-Drone project (https://github.com/GBeetle/GB-Drone).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "lora_state.h"
#include "log_sys.h"
#include "error_handle.h"
#include "gb_timer.h"

extern struct rf24 radio;

// ===========================================================
// CRC16 Implementation (MODBUS CRC16)
// ===========================================================

uint16_t GB_CRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }

    return crc;
}

// ===========================================================
// PID Table CRC Calculation and Validation
// ===========================================================

uint16_t GB_PidTableCalculateCRC(const GB_PID_TABLE_T *table)
{
    if (!table)
        return 0;

    // Calculate CRC over PID parameters only (exclude CRC field itself)
    return GB_CRC16((const uint8_t *)table->params, sizeof(table->params));
}

GB_RESULT GB_PidTableValidate(const GB_PID_TABLE_T *table)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(table, GB_NULL_PTR_ERROR);

    uint16_t calculated_crc = GB_PidTableCalculateCRC(table);

    if (calculated_crc != table->crc16)
    {
        GB_DEBUGI(LORA_TAG, "PID table CRC mismatch: expected 0x%04X, got 0x%04X",
            table->crc16, calculated_crc);
        res = GB_CRC_ERROR;
        goto error_exit;
    }

    GB_DEBUGI(LORA_TAG, "PID table validation passed (CRC: 0x%04X)", calculated_crc);

error_exit:
    return res;
}

// ===========================================================
// Reassembly Context Management
// ===========================================================

void GB_ReassemblyInit(GB_REASSEMBLY_CTX_T *ctx)
{
    if (!ctx)
        return;

    memset(ctx, 0, sizeof(GB_REASSEMBLY_CTX_T));
    ctx->active = false;
}

GB_RESULT GB_ReassemblyCheckTimeout(GB_REASSEMBLY_CTX_T *ctx)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(ctx, GB_NULL_PTR_ERROR);

    if (!ctx->active)
        goto error_exit;

    uint64_t current_time = 0;

    GB_GetTimerMs(&current_time);
    uint64_t elapsed = current_time - ctx->timestamp_ms;

    if (elapsed > GB_REASSEMBLY_TIMEOUT_MS)
    {
        GB_DEBUGI(LORA_TAG, "Reassembly timeout: msg_id=%d, received %d/%d fragments",
            ctx->msg_id, ctx->frag_received, ctx->frag_total);
        GB_ReassemblyInit(ctx); // Reset context
        res = GB_TIMEOUT_ERROR;
        goto error_exit;
    }

error_exit:
    return res;
}

// ===========================================================
// Fragment Send Function
// ===========================================================

GB_RESULT GB_LoraFragmentSend(
    const uint8_t *data,
    size_t len,
    uint8_t msg_id,
    GB_LORA_STATE *state)
{
    GB_RESULT res = GB_OK;

    CHK_NULL(data, GB_NULL_PTR_ERROR);
    CHK_NULL(state, GB_NULL_PTR_ERROR);

    if (len == 0 || len > GB_MAX_FRAGMENTS * GB_FRAGMENT_PAYLOAD_SIZE)
    {
        GB_DEBUGI(LORA_TAG, "Invalid data length for fragmentation: %d bytes", len);
        res = GB_INVALID_PARAM;
        goto error_exit;
    }

    // Calculate number of fragments needed
    uint8_t frag_total = (len + GB_FRAGMENT_PAYLOAD_SIZE - 1) / GB_FRAGMENT_PAYLOAD_SIZE;

    GB_DEBUGI(LORA_TAG, "Fragmenting %d bytes into %d fragments (msg_id=%d)",
        len, frag_total, msg_id);

    // Send each fragment
    for (uint8_t frag_index = 0; frag_index < frag_total; frag_index++)
    {
        GB_LORA_FRAGMENT_T frag;
        memset(&frag, 0, sizeof(frag));

        // Fill fragment header
        frag.type = GB_PID_FRAGMENT;
        frag.sync = msg_id; // Use msg_id as sync for this message
        frag.msg_id = msg_id;
        frag.frag_index = frag_index;
        frag.frag_total = frag_total;

        // Calculate payload size for this fragment
        size_t offset = frag_index * GB_FRAGMENT_PAYLOAD_SIZE;
        size_t payload_size = (offset + GB_FRAGMENT_PAYLOAD_SIZE <= len) ?
            GB_FRAGMENT_PAYLOAD_SIZE :
            (len - offset);

        memcpy(frag.payload, data + offset, payload_size);

        GB_DEBUGI(LORA_TAG, "Sending fragment %d/%d (payload: %d bytes)",
            frag_index + 1, frag_total, payload_size);

        // Send fragment via NRF24
        if (radio.write(&radio, &frag, sizeof(frag)) != GB_OK)
        {
            GB_DEBUGI(LORA_TAG, "Failed to send fragment %d/%d", frag_index + 1, frag_total);
            res = GB_SEND_ERROR;
            goto error_exit;
        }

        radio.flush_rx(&radio);
        radio.write_register(&radio, NRF_STATUS, _BV(RX_DR), false);
        radio.startListening(&radio);

        // Wait for ACK with timeout
        uint64_t start_time = 0;
        uint64_t wait_time = 0;

        GB_GetTimerMs(&start_time);
        GB_GetTimerMs(&wait_time);
        bool ack_received = false;

        while (wait_time - start_time < 500) // 500ms timeout per fragment
        {
            if (radio.isAvailable(&radio))
            {
                GB_LORA_FRAGMENT_T ack_frag;
                radio.read(&radio, &ack_frag, sizeof(ack_frag));

                // Check if this is an ACK for our fragment
                if (ack_frag.type == GB_PID_FRAGMENT &&
                    ack_frag.msg_id == msg_id &&
                    ack_frag.frag_index == frag_index &&
                    ack_frag.sync == (msg_id + 1)) // ACK has sync + 1
                {
                    ack_received = true;
                    GB_DEBUGI(LORA_TAG, "Fragment %d/%d ACK received", frag_index + 1, frag_total);
                    break;
                }
            }

            GB_SleepMs(10);
            GB_GetTimerMs(&wait_time);
        }

        radio.stopListening(&radio);

        if (!ack_received)
        {
            GB_DEBUGI(LORA_TAG, "ACK timeout for fragment %d/%d", frag_index + 1, frag_total);
            res = GB_TIMEOUT_ERROR;
            goto error_exit;
        }

        // Small delay between fragments
        GB_SleepMs(50);
    }

    GB_DEBUGI(LORA_TAG, "All %d fragments sent successfully", frag_total);
    *state = LORA_SEND;

error_exit:
    return res;
}

// ===========================================================
// Fragment Receive Function
// ===========================================================

GB_FRAG_RESULT GB_LoraFragmentReceive(
    const GB_LORA_FRAGMENT_T *frag,
    GB_REASSEMBLY_CTX_T *ctx,
    uint8_t *complete_msg,
    size_t *msg_len)
{
    GB_FRAG_RESULT result = GB_FRAG_OK;

    if (!frag || !ctx || !complete_msg || !msg_len)
    {
        GB_DEBUGI(LORA_TAG, "Fragment receive: null pointer");
        return GB_FRAG_ERROR;
    }

    // Check for timeout on existing reassembly
    if (ctx->active)
    {
        if (GB_ReassemblyCheckTimeout(ctx) != GB_OK)
        {
            return GB_FRAG_TIMEOUT;
        }
    }

    // If no active reassembly, start new one
    if (!ctx->active)
    {
        GB_DEBUGI(LORA_TAG, "Starting new reassembly: msg_id=%d, %d fragments",
            frag->msg_id, frag->frag_total);

        GB_ReassemblyInit(ctx);
        ctx->active = true;
        ctx->msg_id = frag->msg_id;
        ctx->frag_total = frag->frag_total;
        GB_GetTimerMs(&(ctx->timestamp_ms));
    }

    // Verify fragment belongs to current message
    if (frag->msg_id != ctx->msg_id)
    {
        GB_DEBUGI(LORA_TAG, "Fragment msg_id mismatch: expected %d, got %d",
            ctx->msg_id, frag->msg_id);
        return GB_FRAG_ERROR;
    }

    // Check fragment index bounds
    if (frag->frag_index >= frag->frag_total || frag->frag_index >= GB_MAX_FRAGMENTS)
    {
        GB_DEBUGI(LORA_TAG, "Invalid fragment index: %d (total: %d)",
            frag->frag_index, frag->frag_total);
        return GB_FRAG_ERROR;
    }

    // Check if fragment already received
    if (ctx->fragments_received[frag->frag_index])
    {
        GB_DEBUGI(LORA_TAG, "Duplicate fragment %d received", frag->frag_index);
        return GB_FRAG_DUPLICATE;
    }

    // Copy fragment payload to buffer
    size_t offset = frag->frag_index * GB_FRAGMENT_PAYLOAD_SIZE;
    size_t copy_size = GB_FRAGMENT_PAYLOAD_SIZE;

    // For last fragment, calculate actual size
    if (frag->frag_index == frag->frag_total - 1)
    {
        // Last fragment might be partial
        copy_size = GB_FRAGMENT_PAYLOAD_SIZE; // We'll copy full size and trim later
    }

    memcpy(ctx->buffer + offset, frag->payload, copy_size);
    ctx->fragments_received[frag->frag_index] = true;
    ctx->frag_received++;

    GB_DEBUGI(LORA_TAG, "Fragment %d/%d received and buffered (%d bytes)",
        frag->frag_index + 1, frag->frag_total, copy_size);

    // Check if all fragments received
    if (ctx->frag_received == ctx->frag_total)
    {
        GB_DEBUGI(LORA_TAG, "All fragments received! Reassembling message...");

        // Calculate total message size
        size_t total_size = (ctx->frag_total - 1) * GB_FRAGMENT_PAYLOAD_SIZE;

        // For the last fragment, we need to find actual size
        // We'll copy the maximum possible and let the caller validate
        total_size += GB_FRAGMENT_PAYLOAD_SIZE;

        // Copy complete message to output
        memcpy(complete_msg, ctx->buffer, total_size);
        *msg_len = total_size;

        GB_DEBUGI(LORA_TAG, "Message reassembly complete: %d bytes", total_size);

        // Reset context
        GB_ReassemblyInit(ctx);

        result = GB_FRAG_COMPLETE;
    }

    return result;
}
