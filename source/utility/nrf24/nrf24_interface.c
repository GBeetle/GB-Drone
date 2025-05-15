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

#include "nrf24_interface.h"
#include "lora_state.h"
#include "gpio_setting.h"
#include "gb_timer.h"

struct rf24 radio;

static GB_RESULT begin(struct rf24 *nrf24);
static GB_RESULT _init_pins(struct rf24 *nrf24);
static GB_RESULT _init_radio(struct rf24 *nrf24);
static GB_RESULT csn(struct rf24 *nrf24, bool mode);
static GB_RESULT ce(struct rf24 *nrf24, bool level);
static inline void beginTransaction(struct rf24 *nrf24);
static inline void endTransaction(struct rf24 *nrf24);
static uint8_t read_register(struct rf24 *nrf24, uint8_t reg);
static GB_RESULT read_registers(struct rf24 *nrf24, uint8_t reg, uint8_t *buf, uint8_t len);
static GB_RESULT write_register(struct rf24 *nrf24, uint8_t reg, uint8_t value, bool is_cmd_only);
static GB_RESULT write_registers(struct rf24 *nrf24, uint8_t reg, const uint8_t *buf, uint8_t len);
static GB_RESULT write_payload(struct rf24 *nrf24, const void *buf, uint8_t data_len, const uint8_t writeType);
static GB_RESULT read_payload(struct rf24 *nrf24, void *buf, uint8_t data_len);
static GB_RESULT flush_rx(struct rf24 *nrf24);
static GB_RESULT flush_tx(struct rf24 *nrf24);
static uint8_t get_status(struct rf24 *nrf24);
static void print_status(struct rf24 *nrf24, uint8_t _status);
static void print_observe_tx(struct rf24 *nrf24, uint8_t value);
static void print_byte_register(struct rf24 *nrf24, const char *name, uint8_t reg, uint8_t qty);
static void sprintf_byte_register(struct rf24 *nrf24, char *out_buffer, uint8_t reg, uint8_t qty);
static void print_address_register(struct rf24 *nrf24, const char *name, uint8_t reg, uint8_t qty);
static void sprintf_address_register(struct rf24 *nrf24, char *out_buffer, uint8_t reg, uint8_t qty);
static GB_RESULT setChannel(struct rf24 *nrf24, uint8_t channel);
static uint8_t getChannel(struct rf24 *nrf24);
static GB_RESULT setPayloadSize(struct rf24 *nrf24, uint8_t size);
static uint8_t getPayloadSize(struct rf24 *nrf24);
static void printDetails(struct rf24 *nrf24);
static void printPrettyDetails(struct rf24 *nrf24);
static void sprintfPrettyDetails(struct rf24 *nrf24, char *debugging_information);
static bool isChipConnected(struct rf24 *nrf24);
static GB_RESULT isValid(struct rf24 *nrf24);
static GB_RESULT startListening(struct rf24 *nrf24);
static GB_RESULT stopListening(struct rf24 *nrf24);
static GB_RESULT powerDown(struct rf24 *nrf24);
static GB_RESULT powerUp(struct rf24 *nrf24);
static GB_RESULT write_data(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast);
static GB_RESULT write(struct rf24 *nrf24, const void *buf, uint8_t len);
static GB_RESULT writeBlocking(struct rf24 *nrf24, const void *buf, uint8_t len, uint32_t timeout);
static GB_RESULT reUseTX(struct rf24 *nrf24);
static GB_RESULT writeFast_data(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast);
static GB_RESULT writeFast(struct rf24 *nrf24, const void *buf, uint8_t len);
static GB_RESULT startFastWrite(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast, bool startTx);
static GB_RESULT startWrite(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast);
static bool rxFifoFull(struct rf24 *nrf24);
static bool txStandBy(struct rf24 *nrf24);
static bool txStandByAfter(struct rf24 *nrf24, uint32_t timeout, bool startTx);
static GB_RESULT maskIRQ(struct rf24 *nrf24, bool tx, bool fail, bool rx);
static uint8_t getDynamicPayloadSize(struct rf24 *nrf24);
static bool isAvailable(struct rf24 *nrf24);
static bool available(struct rf24 *nrf24, uint8_t *pipe_num);
static GB_RESULT read(struct rf24 *nrf24, void *buf, uint8_t len);
static void whatHappened(struct rf24 *nrf24, bool *tx_ok, bool *tx_fail, bool *rx_ready);
static GB_RESULT openWritingPipe(struct rf24 *nrf24, uint64_t value);
static GB_RESULT openWritingPipeAddr(struct rf24 *nrf24, const uint8_t *address);
static GB_RESULT openReadingPipe(struct rf24 *nrf24, uint8_t child, uint64_t address);
static GB_RESULT setAddressWidth(struct rf24 *nrf24, uint8_t a_width);
static GB_RESULT openReadingPipeAddr(struct rf24 *nrf24, uint8_t child, const uint8_t *address);
static GB_RESULT closeReadingPipe(struct rf24 *nrf24, uint8_t pipe);
static GB_RESULT toggle_features(struct rf24 *nrf24);
static void enableDynamicPayloads(struct rf24 *nrf24);
static void disableDynamicPayloads(struct rf24 *nrf24);
static void enableAckPayload(struct rf24 *nrf24);
static void disableAckPayload(struct rf24 *nrf24);
static void enableDynamicAck(struct rf24 *nrf24);
static bool writeAckPayload(struct rf24 *nrf24, uint8_t pipe, const void *buf, uint8_t len);
static bool isAckPayloadAvailable(struct rf24 *nrf24);
static bool isPVariant(struct rf24 *nrf24);
static void setAutoAck(struct rf24 *nrf24, bool enable);
static void setAutoAckPipe(struct rf24 *nrf24, uint8_t pipe, bool enable);
static bool testCarrier(struct rf24 *nrf24);
static bool testRPD(struct rf24 *nrf24);
static GB_RESULT setPALevel(struct rf24 *nrf24, uint8_t level, bool lnaEnable);
static uint8_t getPALevel(struct rf24 *nrf24);
static uint8_t getARC(struct rf24 *nrf24);
static bool setDataRate(struct rf24 *nrf24, rf24_datarate_e speed);
static rf24_datarate_e getDataRate(struct rf24 *nrf24);
static void setCRCLength(struct rf24 *nrf24, rf24_crclength_e length);
static rf24_crclength_e getCRCLength(struct rf24 *nrf24);
static void disableCRC(struct rf24 *nrf24);
static GB_RESULT setRetries(struct rf24 *nrf24, uint8_t delay, uint8_t count);
static void startConstCarrier(struct rf24 *nrf24, rf24_pa_dbm_e level, uint8_t channel);
static void stopConstCarrier(struct rf24 *nrf24);
static void toggleAllPipes(struct rf24 *nrf24, bool isEnabled);
static uint8_t _data_rate_reg_value(struct rf24 *nrf24, rf24_datarate_e speed);
static uint8_t _pa_level_reg_value(struct rf24 *nrf24, uint8_t level, bool lnaEnable);
static GB_RESULT setRadiation(struct rf24 *nrf24, uint8_t level, rf24_datarate_e speed, bool lnaEnable);

/****************************************************************************/

GB_RESULT GB_NRF24_Init(struct rf24 *nrf24);

static const uint8_t address[][6] = {"1Node", "2Node"};
static GB_LORA_STATE lora_state = LORA_IDLE;

// radioNumber 0 uses address[0] to transmit (RECEIVE), 1 uses address[1] to transmit (SEND)
GB_RESULT GB_LoraSystemInit(GB_LORA_STATE state, bool radioNumber)
{
    GB_RESULT res = GB_OK;

    GB_NRF24_Init(&radio);
    // radio.printDetails(&radio);
    CHK_RES(radio.begin(&radio));
    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    CHK_RES(radio.setPALevel(&radio, RF24_PA_LOW, true)); // RF24_PA_MAX is default.
    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    CHK_BOOL(sizeof(GB_LORA_PACKAGE_T) <= 32);
    CHK_RES(radio.setPayloadSize(&radio, sizeof(GB_LORA_PACKAGE_T)));
    CHK_RES(radio.maskIRQ(&radio, true, true, false));
    // set the TX address of the RX node into the TX pipe
    CHK_RES(radio.openWritingPipeAddr(&radio, address[radioNumber])); // always uses pipe 0
    // set the RX address of the TX node into a RX pipe
    CHK_RES(radio.openReadingPipeAddr(&radio, 1, address[!radioNumber])); // using pipe 1

    switch (state)
    {
    case LORA_RECEIVE:
        // additional setup specific to the node's role
        CHK_RES(radio.startListening(&radio)); // put radio in RX mode
        lora_state = LORA_RECEIVE;
        break;
    case LORA_SEND:
        CHK_RES(radio.stopListening(&radio)); // put radio in TX mode
        lora_state = LORA_SEND;
        break;
    default:
        CHK_RES(GB_RF24_UNKNOWN_STAT);
    }
    GB_DEBUGI(GB_INFO, "NRF24 initialization DONE, %p, %p, payload size: %d", &radio, &(radio.get_status), sizeof(GB_LORA_PACKAGE_T));

error_exit:
    return res;
}

GB_RESULT GB_NRF24_Init(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
#if defined CONFIG_NRF24_SPI
#if defined CONFIG_NRF24_FSPI
    CHK_RES(fspi.begin(&fspi, NRF24_SPI_MOSI, NRF24_SPI_MISO, NRF24_SPI_SCLK, 0));
    CHK_RES(fspi.addDevice(&fspi, GB_SPI_DEV_1, 0, 0, SPI_DEVICE_NO_DUMMY, NRF24_SPI_CLOCK_SPEED, -1));
    nrf24->bus = &fspi;
#elif defined CONFIG_NRF24_HSPI
    CHK_RES(hspi.begin(&hspi, NRF24_SPI_MOSI, NRF24_SPI_MISO, NRF24_SPI_SCLK, 0));
    CHK_RES(hspi.addDevice(&hspi, GB_SPI_DEV_1, 0, 0, SPI_DEVICE_NO_DUMMY, NRF24_SPI_CLOCK_SPEED, -1));
    nrf24->bus = &hspi;
#endif
#else
#error "NRF24 SPI MUST BE DEFINED"
#endif
    nrf24->addr                     = GB_SPI_DEV_1;
    nrf24->ce_pin                   = NRF24_CE;
    nrf24->csn_pin                  = NRF24_SPI_CS;
    nrf24->spi_speed                = NRF24_SPI_CLOCK_SPEED;
    nrf24->payload_size             = 32;
    nrf24->_is_p_variant            = false;
    nrf24->_is_p0_rx                = false;
    nrf24->addr_width               = 5;
    nrf24->dynamic_payloads_enabled = true;
    nrf24->csDelay                  = 5;
    nrf24->pipe0_reading_address[0] = 0;

    // init function
    nrf24->begin                    = &begin;
    nrf24->_init_pins               = &_init_pins;
    nrf24->_init_radio              = &_init_radio;
    nrf24->csn                      = &csn;
    nrf24->ce                       = &ce;
    nrf24->beginTransaction         = &beginTransaction;
    nrf24->endTransaction           = &endTransaction;
    nrf24->read_register            = &read_register;
    nrf24->read_registers           = &read_registers;
    nrf24->write_register           = &write_register;
    nrf24->write_registers          = &write_registers;
    nrf24->write_payload            = &write_payload;
    nrf24->read_payload             = &read_payload;
    nrf24->flush_rx                 = &flush_rx;
    nrf24->flush_tx                 = &flush_tx;
    nrf24->get_status               = &get_status;
    nrf24->print_status             = &print_status;
    nrf24->print_observe_tx         = &print_observe_tx;
    nrf24->print_byte_register      = &print_byte_register;
    nrf24->sprintf_byte_register    = &sprintf_byte_register;
    nrf24->print_address_register   = &print_address_register;
    nrf24->sprintf_address_register = &sprintf_address_register;
    nrf24->setChannel               = &setChannel;
    nrf24->getChannel               = &getChannel;
    nrf24->setPayloadSize           = &setPayloadSize;
    nrf24->getPayloadSize           = &getPayloadSize;
    nrf24->printDetails             = &printDetails;
    nrf24->printPrettyDetails       = &printPrettyDetails;
    nrf24->sprintfPrettyDetails     = &sprintfPrettyDetails;
    nrf24->isChipConnected          = &isChipConnected;
    nrf24->isValid                  = &isValid;
    nrf24->startListening           = &startListening;
    nrf24->stopListening            = &stopListening;
    nrf24->powerDown                = &powerDown;
    nrf24->powerUp                  = &powerUp;
    nrf24->write_data               = &write_data;
    nrf24->write                    = &write;
    nrf24->writeBlocking            = &writeBlocking;
    nrf24->reUseTX                  = &reUseTX;
    nrf24->writeFast_data           = &writeFast_data;
    nrf24->writeFast                = &writeFast;
    nrf24->startFastWrite           = &startFastWrite;
    nrf24->startWrite               = &startWrite;
    nrf24->rxFifoFull               = &rxFifoFull;
    nrf24->txStandBy                = &txStandBy;
    nrf24->txStandByAfter           = &txStandByAfter;
    nrf24->maskIRQ                  = &maskIRQ;
    nrf24->getDynamicPayloadSize    = &getDynamicPayloadSize;
    nrf24->isAvailable              = &isAvailable;
    nrf24->available                = &available;
    nrf24->read                     = &read;
    nrf24->whatHappened             = &whatHappened;
    nrf24->openWritingPipe          = &openWritingPipe;
    nrf24->openWritingPipeAddr      = &openWritingPipeAddr;
    nrf24->openReadingPipe          = &openReadingPipe;
    nrf24->setAddressWidth          = &setAddressWidth;
    nrf24->openReadingPipeAddr      = &openReadingPipeAddr;
    nrf24->closeReadingPipe         = &closeReadingPipe;
    nrf24->toggle_features          = &toggle_features;
    nrf24->enableDynamicPayloads    = &enableDynamicPayloads;
    nrf24->disableDynamicPayloads   = &disableDynamicPayloads;
    nrf24->enableAckPayload         = &enableAckPayload;
    nrf24->disableAckPayload        = &disableAckPayload;
    nrf24->enableDynamicAck         = &enableDynamicAck;
    nrf24->writeAckPayload          = &writeAckPayload;
    nrf24->isAckPayloadAvailable    = &isAckPayloadAvailable;
    nrf24->isPVariant               = &isPVariant;
    nrf24->setAutoAck               = &setAutoAck;
    nrf24->setAutoAckPipe           = &setAutoAckPipe;
    nrf24->testCarrier              = &testCarrier;
    nrf24->testRPD                  = &testRPD;
    nrf24->setPALevel               = &setPALevel;
    nrf24->getPALevel               = &getPALevel;
    nrf24->getARC                   = &getARC;
    nrf24->setDataRate              = &setDataRate;
    nrf24->getDataRate              = &getDataRate;
    nrf24->setCRCLength             = &setCRCLength;
    nrf24->getCRCLength             = &getCRCLength;
    nrf24->disableCRC               = &disableCRC;
    nrf24->setRetries               = &setRetries;
    nrf24->startConstCarrier        = &startConstCarrier;
    nrf24->stopConstCarrier         = &stopConstCarrier;
    nrf24->toggleAllPipes           = &toggleAllPipes;
    nrf24->_data_rate_reg_value     = &_data_rate_reg_value;
    nrf24->_pa_level_reg_value      = &_pa_level_reg_value;
    nrf24->setRadiation             = &setRadiation;
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT begin(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;

    CHK_RES(nrf24->_init_pins(nrf24));
    CHK_RES(nrf24->_init_radio(nrf24));
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT _init_pins(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;

    CHK_RES(nrf24->isValid(nrf24));
    // Initialize pins
    if (nrf24->ce_pin != nrf24->csn_pin)
    {
        //gpio_pad_select_gpio( ce_pin );
        GB_GPIO_Reset(nrf24->ce_pin);
        GB_GPIO_SetDirection(nrf24->ce_pin, GB_GPIO_OUTPUT);
        GB_GPIO_Set(nrf24->ce_pin, 0);

        //gpio_pad_select_gpio( csn_pin );
        GB_GPIO_Reset(nrf24->csn_pin);
        GB_GPIO_SetDirection(nrf24->csn_pin, GB_GPIO_OUTPUT);
        GB_GPIO_Set(nrf24->csn_pin, 1);
    }

    CHK_RES(nrf24->ce(nrf24, RF_LOW));
    CHK_RES(nrf24->csn(nrf24, RF_HIGH));
error_exit:
    return res; // assuming pins are connected properly
}

/****************************************************************************/

static GB_RESULT _init_radio(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See datasheet for a more complete explanation.
    CHK_RES(nrf24->setRetries(nrf24, 5, 15));

#if 0
    uint8_t ret = nrf24->read_register(nrf24, SETUP_RETR);
    int i = 0;
    while (ret != 0x5f) {
        GB_DEBUGI(RF24_TAG, "retry: %02x, count: %d\n", ret, i);
        CHK_RES(nrf24->setRetries(nrf24, 5, 15));
        ret = nrf24->read_register(nrf24, SETUP_RETR);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        i++;
    }
#endif

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware. Since this value occupies the same register as the PA level value, set
    // the PA level to MAX
    CHK_RES(nrf24->setRadiation(nrf24, RF24_PA_MAX, RF24_1MBPS, true)); // LNA enabled by default

    // detect if is a plus variant & use old toggle features command accordingly
    uint8_t before_toggle = nrf24->read_register(nrf24, FEATURE);
    CHK_RES(nrf24->toggle_features(nrf24));
    uint8_t after_toggle = nrf24->read_register(nrf24, FEATURE);
    nrf24->_is_p_variant = (before_toggle == after_toggle);
    if (after_toggle)
    {
        if (nrf24->_is_p_variant)
        {
            // module did not experience power-on-reset (#401)
            CHK_RES(nrf24->toggle_features(nrf24));
        }
        // allow use of multicast parameter and dynamic payloads by default
        CHK_RES(nrf24->write_register(nrf24, FEATURE, 0x00, false));
    }
    nrf24->ack_payloads_enabled = false; // ack payloads disabled by default
    CHK_RES(nrf24->write_register(nrf24, DYNPD, 0x00, false));     // disable dynamic payloads by default (for all pipes)
    nrf24->dynamic_payloads_enabled = false;
    CHK_RES(nrf24->write_register(nrf24, EN_AA, 0x3f, false));  // enable auto-ack on all pipes
    CHK_RES(nrf24->write_register(nrf24, EN_RXADDR, 3, false)); // only open RX pipes 0 & 1
    CHK_RES(nrf24->setPayloadSize(nrf24, 32));           // set static payload size to 32 (max) bytes by default
    CHK_RES(nrf24->setAddressWidth(nrf24, 5));           // set default address length to (max) 5 bytes

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    CHK_RES(nrf24->setChannel(nrf24, 76));

    // Reset current status
    // Notice reset and flush is the last thing we do
    CHK_RES(nrf24->write_register(nrf24, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), false));

    // Flush buffers
    CHK_RES(flush_rx(nrf24));
    CHK_RES(flush_tx(nrf24));

    // Clear CONFIG register:
    //      Reflect all IRQ events on IRQ pin
    //      Enable PTX
    //      Power Up
    //      16-bit CRC (CRC required by auto-ack)
    // Do not write CE high so radio will remain in standby I mode
    // PTX should use only 22uA of power
    CHK_RES(nrf24->write_register(nrf24, NRF_CONFIG, (_BV(EN_CRC) | _BV(CRCO)), false));
    CHK_RES(nrf24->read_registers(nrf24, NRF_CONFIG, &(nrf24->config_reg), 1));

    CHK_RES(nrf24->powerUp(nrf24));

    // if config is not set correctly then there was a bad response from module
    CHK_BOOL(nrf24->config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)));
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT csn(struct rf24 *nrf24, bool mode)
{
    if (nrf24->ce_pin != nrf24->csn_pin)
    {
        if (ESP_OK != GB_GPIO_Set(nrf24->csn_pin, mode))
            return GB_RF24_IO_SET_FAIL;
    }
    return GB_OK;
}

/****************************************************************************/

static GB_RESULT ce(struct rf24 *nrf24, bool level)
{
    //Allow for 3-pin use on ATTiny
    if (nrf24->ce_pin != nrf24->csn_pin)
    {
        if (ESP_OK != GB_GPIO_Set(nrf24->ce_pin, level))
            return GB_RF24_IO_SET_FAIL;
    }
    return GB_OK;
}

/****************************************************************************/

static inline void beginTransaction(struct rf24 *nrf24)
{
    nrf24->csn(nrf24, RF_LOW);
}

/****************************************************************************/

static inline void endTransaction(struct rf24 *nrf24)
{
    nrf24->csn(nrf24, RF_HIGH);
}

/****************************************************************************/
static uint8_t read_register(struct rf24 *nrf24, uint8_t reg)
{
    uint8_t result;
    GB_RESULT res = nrf24->read_registers(nrf24, reg, &result, 1);
    CHK_LOGE(res, "NRF24 read_register failed\n");

    return result;
}

/****************************************************************************/

static GB_RESULT read_registers(struct rf24 *nrf24, uint8_t reg, uint8_t *buf, uint8_t len)
{
    GB_RESULT res = GB_OK;

    nrf24->beginTransaction(nrf24); //configures the spi settings for RPi, locks mutex and setting csn low
    CHK_BOOL(len > 0);
    nrf24->spi_txbuff[0] = R_REGISTER | (REGISTER_MASK & reg);
    CHK_RES(nrf24->bus->readWriteBytes(nrf24->bus, nrf24->addr, 0x00, len + 1, nrf24->spi_rxbuff, nrf24->spi_txbuff));
    nrf24->status = nrf24->spi_rxbuff[0];
    memcpy(buf, nrf24->spi_rxbuff + 1, len);

    //GB_DUMPI(RF24_TAG, nrf24->spi_rxbuff, len + 1);
error_exit:
    nrf24->endTransaction(nrf24); // unlocks mutex and setting csn high
    return res;
}

/****************************************************************************/

static GB_RESULT write_register(struct rf24 *nrf24, uint8_t reg, uint8_t value, bool is_cmd_only)
{
    GB_RESULT res = GB_OK;

    nrf24->beginTransaction(nrf24);
    if (is_cmd_only) {
        CHK_RES(nrf24->bus->writeByte(nrf24->bus, nrf24->addr, 0x00, W_REGISTER | reg));
    }
    else {
        nrf24->spi_txbuff[0] = (W_REGISTER | reg);
        nrf24->spi_txbuff[1] = value;
        //GB_DUMPI(RF24_TAG, nrf24->spi_txbuff, 2);
        CHK_RES(nrf24->bus->readWriteBytes(nrf24->bus, nrf24->addr, 0x00, 2, nrf24->spi_rxbuff, nrf24->spi_txbuff));
        nrf24->status = nrf24->spi_rxbuff[0];
        //GB_DEBUGI(RF24_TAG, "NRF24 status: %02x\n", nrf24->status);
    }
error_exit:
    nrf24->endTransaction(nrf24);

    return res;
}

/****************************************************************************/

static GB_RESULT write_registers(struct rf24 *nrf24, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    GB_RESULT res = GB_OK;

    nrf24->beginTransaction(nrf24);
    CHK_BOOL(len > 0);
    nrf24->spi_txbuff[0] = W_REGISTER | (REGISTER_MASK & reg);
    memcpy(nrf24->spi_txbuff + 1, buf, len);
    CHK_RES(nrf24->bus->readWriteBytes(nrf24->bus, nrf24->addr, 0x00, len + 1, nrf24->spi_rxbuff, nrf24->spi_txbuff));
    nrf24->status = nrf24->spi_rxbuff[0];
error_exit:
    nrf24->endTransaction(nrf24);
    return res;
}

/****************************************************************************/

static GB_RESULT write_payload(struct rf24 *nrf24, const void *buf, uint8_t data_len, const uint8_t writeType)
{
    GB_RESULT res = GB_OK;

    CHK_BOOL(data_len > 0);
    uint8_t blank_len = !data_len ? 1 : 0;
    if (!nrf24->dynamic_payloads_enabled) {
        data_len = rf24_min(data_len, nrf24->payload_size);
        blank_len = nrf24->payload_size - data_len;
    }
    else {
        data_len = rf24_min(data_len, 32);
    }

    GB_DEBUGD(RF24_TAG, "[Writing %u bytes %u blanks]\n", data_len, blank_len);

    nrf24->beginTransaction(nrf24);
    uint8_t size = data_len + blank_len + 1; // Add register value to transmit buffer

    nrf24->spi_txbuff[0] = writeType;
    memcpy(nrf24->spi_txbuff + 1, buf, data_len);
    memset(nrf24->spi_txbuff + 1 + data_len, 0x00, blank_len);
    CHK_RES(nrf24->bus->readWriteBytes(nrf24->bus, nrf24->addr, 0x00, size, nrf24->spi_rxbuff, nrf24->spi_txbuff));
    nrf24->status = nrf24->spi_rxbuff[0]; // status is 1st byte of receive buffer
error_exit:
    nrf24->endTransaction(nrf24);
    return res;
}

/****************************************************************************/

static GB_RESULT read_payload(struct rf24 *nrf24, void *buf, uint8_t data_len)
{
    GB_RESULT res = GB_OK;
    uint8_t blank_len = 0;

    CHK_BOOL(data_len > 0);
    if (!nrf24->dynamic_payloads_enabled) {
        data_len = rf24_min(data_len, nrf24->payload_size);
        blank_len = nrf24->payload_size - data_len;
    }
    else {
        data_len = rf24_min(data_len, 32);
    }

    GB_DEBUGD(RF24_TAG, "[Reading %u bytes %u blanks]\n", data_len, blank_len);

    nrf24->beginTransaction(nrf24);
    uint8_t size = data_len + blank_len + 1; // Add register value to transmit buffer

    nrf24->spi_txbuff[0] = R_RX_PAYLOAD;
    memset(nrf24->spi_txbuff + 1, RF24_NOP, size - 1);
    CHK_RES(nrf24->bus->readWriteBytes(nrf24->bus, nrf24->addr, 0x00, size, nrf24->spi_rxbuff, nrf24->spi_txbuff));
    nrf24->status = nrf24->spi_rxbuff[0]; // 1st byte is status
    memcpy(buf, nrf24->spi_rxbuff + 1, data_len);
error_exit:
    nrf24->endTransaction(nrf24);
    return res;
}

/****************************************************************************/

static GB_RESULT flush_rx(struct rf24 *nrf24)
{
    return nrf24->write_register(nrf24, FLUSH_RX, RF24_NOP, true);
}

/****************************************************************************/

static GB_RESULT flush_tx(struct rf24 *nrf24)
{
    return nrf24->write_register(nrf24, FLUSH_TX, RF24_NOP, true);
}

/****************************************************************************/

static uint8_t get_status(struct rf24 *nrf24)
{
    nrf24->write_register(nrf24, RF24_NOP, RF24_NOP, false);
    return nrf24->status;
}

/****************************************************************************/

static void print_status(struct rf24 *nrf24, uint8_t _status)
{
    GB_DEBUGD(RF24_TAG, PSTR("STATUS\t\t= 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),  _status, (_status & _BV(RX_DR)) ? 1 : 0,
             (_status & _BV(TX_DS)) ? 1 : 0, (_status & _BV(MAX_RT)) ? 1 : 0, ((_status >> RX_P_NO) & 0x07), (_status & _BV(TX_FULL)) ? 1 : 0);
}

/****************************************************************************/

static void print_observe_tx(struct rf24 *nrf24, uint8_t value)
{
    GB_DEBUGD(RF24_TAG, PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"), value, (value >> PLOS_CNT) & 0x0F, (value >> ARC_CNT) & 0x0F);
}

/****************************************************************************/

static void print_byte_register(struct rf24 *nrf24, const char *name, uint8_t reg, uint8_t qty)
{
//char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
//GB_DEBUGD(RF24_TAG, PSTR(PRIPSTR"\t%c ="),name,extra_tab);
    GB_DEBUGD(RF24_TAG, PSTR(PRIPSTR "\t="), name);
    while (qty--)
    {
        uint8_t val = nrf24->read_register(nrf24, reg++);
        GB_DEBUGD(RF24_TAG, PSTR(" 0x%02x"), val);
    }
    GB_DEBUGD(RF24_TAG, PSTR("\r\n"));
}

/****************************************************************************/

static void sprintf_byte_register(struct rf24 *nrf24, char *out_buffer, uint8_t reg, uint8_t qty)
{
    uint8_t i = 0;
    char sprintf_buffer[4] = {'\0'};
    while (qty--)
    {
        uint8_t val = nrf24->read_register(nrf24, reg++);
        sprintf(sprintf_buffer, PSTR("%02x"), val);
        out_buffer[i + 0] = sprintf_buffer[0];
        out_buffer[i + 1] = sprintf_buffer[1];
        i = i + 2;
    }
}

/****************************************************************************/

static void print_address_register(struct rf24 *nrf24, const char *name, uint8_t reg, uint8_t qty)
{
    GB_DEBUGD(RF24_TAG, PSTR(PRIPSTR "\t="), name);
    uint8_t buffer[nrf24->addr_width];
    while (qty--)
    {
        nrf24->read_registers(nrf24, reg++ & REGISTER_MASK, buffer, nrf24->addr_width);

        GB_DEBUGD(RF24_TAG, PSTR(" 0x"));
        uint8_t *bufptr = buffer + nrf24->addr_width;
        while (--bufptr >= buffer)
        {
            GB_DEBUGD(RF24_TAG, PSTR("%02x"), *bufptr);
        }
    }
    GB_DEBUGD(RF24_TAG, PSTR("\r\n"));
}

/****************************************************************************/

static void sprintf_address_register(struct rf24 *nrf24, char *out_buffer, uint8_t reg, uint8_t qty)
{
    uint8_t i = 0;
    char sprintf_buffer[4] = {'\0'};
    uint8_t read_buffer[nrf24->addr_width];
    while (qty--)
    {
        nrf24->read_registers(nrf24, reg++ & REGISTER_MASK, read_buffer, nrf24->addr_width);

        uint8_t *bufptr = read_buffer + nrf24->addr_width;
        while (--bufptr >= read_buffer)
        {
            sprintf(sprintf_buffer, PSTR("%02x"), *bufptr);
            out_buffer[i] = sprintf_buffer[0];
            out_buffer[i + 1] = sprintf_buffer[1];
            i = i + 2;
        }
    }
}

/****************************************************************************/

static GB_RESULT setChannel(struct rf24 *nrf24, uint8_t channel)
{
    const uint8_t max_channel = 125;
    return nrf24->write_register(nrf24, RF_CH, rf24_min(channel, max_channel), false);
}

static uint8_t getChannel(struct rf24 *nrf24)
{
    return nrf24->read_register(nrf24, RF_CH);
}

/****************************************************************************/

static GB_RESULT setPayloadSize(struct rf24 *nrf24, uint8_t size)
{
    GB_RESULT res = GB_OK;
    // payload size must be in range [1, 32]
    nrf24->payload_size = rf24_max(1, rf24_min(32, size));

    // write static payload size setting for all pipes
    for (uint8_t i = 0; i < 6; ++i) {
        CHK_RES(nrf24->write_register(nrf24, RX_PW_P0 + i, nrf24->payload_size, false));
    }
error_exit:
    return res;
}

/****************************************************************************/

static uint8_t getPayloadSize(struct rf24 *nrf24)
{
    return nrf24->payload_size;
}

/****************************************************************************/

static const  char rf24_datarate_e_str_0[] = "= 1 MBPS";
static const  char rf24_datarate_e_str_1[] = "= 2 MBPS";
static const  char rf24_datarate_e_str_2[] = "= 250 KBPS";
static const  char *const rf24_datarate_e_str_P[] = {
    rf24_datarate_e_str_0,
    rf24_datarate_e_str_1,
    rf24_datarate_e_str_2,
};
static const  char rf24_model_e_str_0[] = "nRF24L01";
static const  char rf24_model_e_str_1[] = "nRF24L01+";
static const  char *const rf24_model_e_str_P[] = {
    rf24_model_e_str_0,
    rf24_model_e_str_1,
};
static const  char rf24_crclength_e_str_0[] = "= Disabled";
static const  char rf24_crclength_e_str_1[] = "= 8 bits";
static const  char rf24_crclength_e_str_2[] = "= 16 bits";
static const  char *const rf24_crclength_e_str_P[] = {
    rf24_crclength_e_str_0,
    rf24_crclength_e_str_1,
    rf24_crclength_e_str_2,
};
static const  char rf24_pa_dbm_e_str_0[] = "= PA_MIN";
static const  char rf24_pa_dbm_e_str_1[] = "= PA_LOW";
static const  char rf24_pa_dbm_e_str_2[] = "= PA_HIGH";
static const  char rf24_pa_dbm_e_str_3[] = "= PA_MAX";
static const  char *const rf24_pa_dbm_e_str_P[] = {
    rf24_pa_dbm_e_str_0,
    rf24_pa_dbm_e_str_1,
    rf24_pa_dbm_e_str_2,
    rf24_pa_dbm_e_str_3,
};

static const  char rf24_feature_e_str_on[] = "= Enabled";
static const  char rf24_feature_e_str_allowed[] = "= Allowed";
static const  char rf24_feature_e_str_open[] = " open ";
static const  char rf24_feature_e_str_closed[] = "closed";
static const  char *const rf24_feature_e_str_P[] = {
    rf24_crclength_e_str_0,
    rf24_feature_e_str_on,
    rf24_feature_e_str_allowed,
    rf24_feature_e_str_closed,
    rf24_feature_e_str_open};

static void printDetails(struct rf24 *nrf24)
{
    GB_DEBUGD(RF24_TAG, PSTR("SPI Speedz\t= %d Mhz\n"), (nrf24->spi_speed / 1000000)); //Print the SPI speed on non-Linux devices

    nrf24->print_status(nrf24, nrf24->get_status(nrf24));

    nrf24->print_address_register(nrf24, PSTR("RX_ADDR_P0-1"), RX_ADDR_P0, 2);
    nrf24->print_byte_register(nrf24, PSTR("RX_ADDR_P2-5"), RX_ADDR_P2, 4);
    nrf24->print_address_register(nrf24, PSTR("TX_ADDR\t"), TX_ADDR, 1);

    nrf24->print_byte_register(nrf24, PSTR("RX_PW_P0-6"), RX_PW_P0, 6);
    nrf24->print_byte_register(nrf24, PSTR("EN_AA\t"), EN_AA, 1);
    nrf24->print_byte_register(nrf24, PSTR("EN_RXADDR"), EN_RXADDR, 1);
    nrf24->print_byte_register(nrf24, PSTR("RF_CH\t"), RF_CH, 1);
    nrf24->print_byte_register(nrf24, PSTR("RF_SETUP"), RF_SETUP, 1);
    nrf24->print_byte_register(nrf24, PSTR("CONFIG\t"), NRF_CONFIG, 1);
    nrf24->print_byte_register(nrf24, PSTR("DYNPD/FEATURE"), DYNPD, 2);

    GB_DEBUGD(RF24_TAG, PSTR("Data Rate\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_datarate_e_str_P[nrf24->getDataRate(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("Model\t\t= " PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_model_e_str_P[nrf24->isPVariant(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("CRC Length\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_crclength_e_str_P[nrf24->getCRCLength(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("PA Power\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_pa_dbm_e_str_P[nrf24->getPALevel(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("ARC\t\t= %d\r\n"), nrf24->getARC(nrf24));
}

static void printPrettyDetails(struct rf24 *nrf24)
{
    GB_DEBUGD(RF24_TAG, PSTR("SPI Frequency\t\t= %d Mhz\n"), nrf24->spi_speed / 1000000); //Print the SPI speed on non-Linux devices

    uint8_t channel = nrf24->getChannel(nrf24);
    uint16_t frequency = (channel + 2400);
    GB_DEBUGD(RF24_TAG, PSTR("Channel\t\t\t= %u (~ %u MHz)\r\n"), channel, frequency);

    GB_DEBUGD(RF24_TAG, PSTR("RF Data Rate\t\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_datarate_e_str_P[nrf24->getDataRate(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("RF Power Amplifier\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_pa_dbm_e_str_P[nrf24->getPALevel(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("RF Low Noise Amplifier\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(nrf24->read_register(nrf24, RF_SETUP) & 1) * 1])));
    GB_DEBUGD(RF24_TAG, PSTR("CRC Length\t\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_crclength_e_str_P[nrf24->getCRCLength(nrf24)])));
    GB_DEBUGD(RF24_TAG, PSTR("Address Length\t\t= %d bytes\r\n"), (nrf24->read_register(nrf24, SETUP_AW) & 3) + 2);
    GB_DEBUGD(RF24_TAG, PSTR("Static Payload Length\t= %d bytes\r\n"), nrf24->getPayloadSize(nrf24));

    uint8_t setupRetry = nrf24->read_register(nrf24, SETUP_RETR);
    GB_DEBUGD(RF24_TAG, PSTR("Auto Retry Delay\t= %d microseconds\r\n"), (setupRetry >> ARD) * 250 + 250);
    GB_DEBUGD(RF24_TAG, PSTR("Auto Retry Attempts\t= %d maximum\r\n"), setupRetry & 0x0F);

    uint8_t observeTx = nrf24->read_register(nrf24, OBSERVE_TX);
    GB_DEBUGD(RF24_TAG, PSTR("Packets lost on\n    current channel\t= %d\r\n"), observeTx >> 4);
    GB_DEBUGD(RF24_TAG, PSTR("Retry attempts made for\n    last transmission\t= %d\r\n"), observeTx & 0x0F);

    uint8_t features = nrf24->read_register(nrf24, FEATURE);
    GB_DEBUGD(RF24_TAG, PSTR("Multicast\t\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(features & _BV(EN_DYN_ACK)) * 2])));
    GB_DEBUGD(RF24_TAG, PSTR("Custom ACK Payload\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(features & _BV(EN_ACK_PAY)) * 1])));

    uint8_t dynPl = nrf24->read_register(nrf24, DYNPD);
    GB_DEBUGD(RF24_TAG, PSTR("Dynamic Payloads\t" PRIPSTR
                  "\r\n"),
             (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(dynPl && (features & _BV(EN_DPL))) * 1])));

    uint8_t autoAck = nrf24->read_register(nrf24, EN_AA);
    if (autoAck == 0x3F || autoAck == 0)
    {
        // all pipes have the same configuration about auto-ack feature
        GB_DEBUGD(RF24_TAG, PSTR("Auto Acknowledgment\t" PRIPSTR
                      "\r\n"),
                 (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(autoAck) * 1])));
    }
    else
    {
        // representation per pipe
        GB_DEBUGD(RF24_TAG, PSTR("Auto Acknowledgment\t= 0b%c%c%c%c%c%c\r\n"),
                 ((autoAck & _BV(ENAA_P5)) + 48),
                 ((autoAck & _BV(ENAA_P4)) + 48),
                 ((autoAck & _BV(ENAA_P3)) + 48),
                 ((autoAck & _BV(ENAA_P2)) + 48),
                 ((autoAck & _BV(ENAA_P1)) + 48),
                 ((autoAck & _BV(ENAA_P0)) + 48));
    }

    nrf24->config_reg = nrf24->read_register(nrf24, NRF_CONFIG);
    GB_DEBUGD(RF24_TAG, PSTR("Primary Mode\t\t= %cX\r\n"), nrf24->config_reg & _BV(PRIM_RX) ? 'R' : 'T');
    nrf24->print_address_register(nrf24, PSTR("TX address\t"), TX_ADDR, 1);

    uint8_t openPipes = nrf24->read_register(nrf24, EN_RXADDR);
    for (uint8_t i = 0; i < 6; ++i)
    {
        bool isOpen = openPipes & _BV(i);
        GB_DEBUGD(RF24_TAG, PSTR("pipe %u (" PRIPSTR
                      ") bound"),
                 i, (char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen + 3])));
        if (i < 2)
        {
            nrf24->print_address_register(nrf24, PSTR(""), (RX_ADDR_P0 + i), 1);
        }
        else
        {
            nrf24->print_byte_register(nrf24, PSTR(""), (RX_ADDR_P0 + i), 1);
        }
    }
}

/****************************************************************************/

static void sprintfPrettyDetails(struct rf24 *nrf24, char *debugging_information)
{
    const char *format_string = PSTR("================ SPI Configuration ================\n"
                                     "CSN Pin\t\t\t= %d\n"
                                     "CE Pin\t\t\t= %d\n"
                                     "SPI Frequency\t\t= %d Mhz\n"
                                     "================ NRF Configuration ================\n"
                                     "Channel\t\t\t= %u (~ %u MHz)\n"
                                     "RF Data Rate\t\t" PRIPSTR
                                     "\n"
                                     "RF Power Amplifier\t" PRIPSTR
                                     "\n"
                                     "RF Low Noise Amplifier\t" PRIPSTR
                                     "\n"
                                     "CRC Length\t\t" PRIPSTR
                                     "\n"
                                     "Address Length\t\t= %d bytes\n"
                                     "Static Payload Length\t= %d bytes\n"
                                     "Auto Retry Delay\t= %d microseconds\n"
                                     "Auto Retry Attempts\t= %d maximum\n"
                                     "Packets lost on\n    current channel\t= %d\r\n"
                                     "Retry attempts made for\n    last transmission\t= %d\r\n"
                                     "Multicast\t\t" PRIPSTR
                                     "\n"
                                     "Custom ACK Payload\t" PRIPSTR
                                     "\n"
                                     "Dynamic Payloads\t" PRIPSTR
                                     "\n"
                                     "Auto Acknowledgment\t" PRIPSTR
                                     "\n"
                                     "Primary Mode\t\t= %cX\n"
                                     "TX address\t\t= 0x" PRIPSTR
                                     "\n"
                                     "pipe 0 (" PRIPSTR
                                     ") bound\t= 0x" PRIPSTR
                                     "\n"
                                     "pipe 1 (" PRIPSTR
                                     ") bound\t= 0x" PRIPSTR
                                     "\n"
                                     "pipe 2 (" PRIPSTR
                                     ") bound\t= 0x" PRIPSTR
                                     "\n"
                                     "pipe 3 (" PRIPSTR
                                     ") bound\t= 0x" PRIPSTR
                                     "\n"
                                     "pipe 4 (" PRIPSTR
                                     ") bound\t= 0x" PRIPSTR
                                     "\n"
                                     "pipe 5 (" PRIPSTR
                                     ") bound\t= 0x" PRIPSTR);

    char tx_address_char_array[16] = {'\0'};
    nrf24->sprintf_address_register(nrf24, tx_address_char_array, TX_ADDR, 1);

    char pipe_address_char_2d_array[6][16] = {'\0'};
    char pipe_address_char_array[16] = {'\0'};
    bool isOpen_array[6] = {false};

    uint8_t openPipes = nrf24->read_register(nrf24, EN_RXADDR);
    for (uint8_t i = 0; i < 6; ++i)
    {
        bool isOpen = openPipes & _BV(i);
        isOpen_array[i] = isOpen;
        if (i < 2)
        {
            nrf24->sprintf_address_register(nrf24, pipe_address_char_array, (RX_ADDR_P0 + i), 1);
            for (uint8_t j = 0; j < 16; j++)
            {
                pipe_address_char_2d_array[i][j] = pipe_address_char_array[j];
            }
            for (uint8_t j = 0; j < 16; j++)
            {
                pipe_address_char_array[j] = '\0';
            }
        }
        else
        {
            nrf24->sprintf_byte_register(nrf24, pipe_address_char_array, (RX_ADDR_P0 + i), 1);
            for (uint8_t j = 0; j < 16; j++)
            {
                pipe_address_char_2d_array[i][j] = pipe_address_char_array[j];
            }
            for (uint8_t j = 0; j < 16; j++)
            {
                pipe_address_char_array[j] = '\0';
            }
        }
    }

    char autoack_status_char_array[11] = {'\0'};
    uint8_t autoAck = nrf24->read_register(nrf24, EN_AA);
    if (autoAck == 0x3F || autoAck == 0)
    {
        // all pipes have the same configuration about auto-ack feature
        sprintf(autoack_status_char_array,
                  PSTR("" PRIPSTR
                       ""),
                  (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(autoAck) * 1])));
    }
    else
    {
        // representation per pipe
        sprintf(autoack_status_char_array,
                  PSTR("= 0b%c%c%c%c%c%c"),
                  ((autoAck & _BV(ENAA_P5)) + 48),
                  ((autoAck & _BV(ENAA_P4)) + 48),
                  ((autoAck & _BV(ENAA_P3)) + 48),
                  ((autoAck & _BV(ENAA_P2)) + 48),
                  ((autoAck & _BV(ENAA_P1)) + 48),
                  ((autoAck & _BV(ENAA_P0)) + 48));
    }

    sprintf(debugging_information,
              format_string,
              nrf24->csn_pin,
              nrf24->ce_pin,
              (nrf24->spi_speed / 1000000),
              nrf24->getChannel(nrf24),
              (nrf24->getChannel(nrf24) + 2400),
              (char *)(pgm_read_ptr(&rf24_datarate_e_str_P[nrf24->getDataRate(nrf24)])),
              (char *)(pgm_read_ptr(&rf24_pa_dbm_e_str_P[nrf24->getPALevel(nrf24)])),
              (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(nrf24->read_register(nrf24, RF_SETUP) & 1) * 1])),
              (char *)(pgm_read_ptr(&rf24_crclength_e_str_P[nrf24->getCRCLength(nrf24)])),
              ((nrf24->read_register(nrf24, SETUP_AW) & 3) + 2),
              nrf24->getPayloadSize(nrf24),
              ((nrf24->read_register(nrf24, SETUP_RETR) >> ARD) * 250 + 250),
              (nrf24->read_register(nrf24, SETUP_RETR) & 0x0F),
              (nrf24->read_register(nrf24, OBSERVE_TX) >> 4),
              (nrf24->read_register(nrf24, OBSERVE_TX) & 0x0F),
              (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(nrf24->read_register(nrf24, FEATURE) & _BV(EN_DYN_ACK)) * 2])),
              (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(nrf24->read_register(nrf24, FEATURE) & _BV(EN_ACK_PAY)) * 1])),
              (char *)(pgm_read_ptr(&rf24_feature_e_str_P[(nrf24->read_register(nrf24, DYNPD) && (nrf24->read_register(nrf24, FEATURE) & _BV(EN_DPL))) * 1])),
              (autoack_status_char_array),
              (nrf24->read_register(nrf24, NRF_CONFIG) & _BV(PRIM_RX) ? 'R' : 'T'),
              (tx_address_char_array),
              ((char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen_array[0] + 3]))),
              (pipe_address_char_2d_array[0]),
              ((char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen_array[1] + 3]))),
              (pipe_address_char_2d_array[1]),
              ((char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen_array[2] + 3]))),
              (pipe_address_char_2d_array[2]),
              ((char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen_array[3] + 3]))),
              (pipe_address_char_2d_array[3]),
              ((char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen_array[4] + 3]))),
              (pipe_address_char_2d_array[4]),
              ((char *)(pgm_read_ptr(&rf24_feature_e_str_P[isOpen_array[5] + 3]))),
              (pipe_address_char_2d_array[5]));
}

/****************************************************************************/

static bool isChipConnected(struct rf24 *nrf24)
{
    uint8_t val = nrf24->read_register(nrf24, SETUP_AW);
    return val == (nrf24->addr_width - 2);
}

/****************************************************************************/

static GB_RESULT isValid(struct rf24 *nrf24)
{
    if (nrf24->ce_pin == 0xFFFF || nrf24->csn_pin == 0xFFFF)
        return GB_RF24_INVALID_PIN;
    return GB_OK;
}

/****************************************************************************/

static GB_RESULT startListening(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    nrf24->powerUp(nrf24);
    nrf24->config_reg |= _BV(PRIM_RX);
    CHK_RES(nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false));
    CHK_RES(nrf24->write_register(nrf24, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), false));
    nrf24->ce(nrf24, RF_HIGH);

    // Restore the pipe0 address, if exists
    if (nrf24->_is_p0_rx)
    {
        CHK_RES(nrf24->write_registers(nrf24, RX_ADDR_P0, nrf24->pipe0_reading_address, nrf24->addr_width));
    }
    else
    {
        CHK_RES(nrf24->closeReadingPipe(nrf24, 0));
    }
error_exit:
    return res;
}

/****************************************************************************/
static const  uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2,
                                                    ERX_P3, ERX_P4, ERX_P5};

static GB_RESULT stopListening(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    nrf24->ce(nrf24, RF_LOW);

    //delayMicroseconds(100);
    vTaskDelay(nrf24->txDelay / portTICK_PERIOD_MS);
    if (nrf24->ack_payloads_enabled)
    {
        nrf24->flush_tx(nrf24);
    }

    nrf24->config_reg = (nrf24->config_reg & ~_BV(PRIM_RX));
    CHK_RES(nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false));
    CHK_RES(nrf24->write_register(nrf24, EN_RXADDR, nrf24->read_register(nrf24, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0])), false)); // Enable RX on pipe0
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT powerDown(struct rf24 *nrf24)
{
    nrf24->ce(nrf24, RF_LOW); // Guarantee CE is low on powerDown
    nrf24->config_reg = nrf24->config_reg & ~_BV(PWR_UP);
    return nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false);
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
static GB_RESULT powerUp(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    // if not powered up then power up and wait for the radio to initialize
    if (!(nrf24->config_reg & _BV(PWR_UP)))
    {
        nrf24->config_reg |= _BV(PWR_UP);
        CHK_RES(nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        //delayMicroseconds(RF24_POWERUP_DELAY);
        vTaskDelay(RF24_POWERUP_DELAY / portTICK_PERIOD_MS);
    }
error_exit:
    return res;
}

/******************************************************************/

//Similar to the previous write, clears the interrupt flags
static GB_RESULT write_data(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast)
{
    GB_RESULT res = GB_OK;
    //Start Writing
    CHK_RES(nrf24->startFastWrite(nrf24, buf, len, multicast, true));

    while (!(nrf24->get_status(nrf24) & (_BV(TX_DS) | _BV(MAX_RT))))
        ;

    nrf24->ce(nrf24, RF_LOW);

    CHK_RES(nrf24->write_register(nrf24, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), false));

    //Max retries exceeded
    if (nrf24->status & _BV(MAX_RT))
    {
        nrf24->flush_tx(nrf24); // Only going to be 1 packet in the FIFO at a time using this method, so just flush
        //res = GB_RF24_W_DATA_FAIL;
        CHK_RES(res);
    }
error_exit:
    return res;
}

static GB_RESULT write(struct rf24 *nrf24, const void *buf, uint8_t len)
{
    return nrf24->write_data(nrf24, buf, len, 0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
static GB_RESULT writeBlocking(struct rf24 *nrf24, const void *buf, uint8_t len, uint32_t timeout)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //This way the FIFO will fill up and allow blocking until packets go through
    //The radio will auto-clear everything in the FIFO as long as CE remains high
    GB_RESULT res = GB_OK;
    uint64_t start_timer = 0;
    uint64_t end_timer = 0;

    GB_GetTimerMs(&start_timer);
    while ((nrf24->get_status(nrf24) & (_BV(TX_FULL))))
    { // Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

        if (nrf24->status & _BV(MAX_RT))
        {              // If MAX Retries have been reached
            CHK_RES(nrf24->reUseTX(nrf24)); // Set re-transmit and clear the MAX_RT interrupt flag
            GB_GetTimerMs(&end_timer);
            if (end_timer - start_timer > timeout)
            {
                return 0; // If this payload has exceeded the user-defined timeout, exit and return 0
            }
        }
    }

    //Start Writing
    CHK_RES(nrf24->startFastWrite(nrf24, buf, len, 0, true)); // Write the payload if a buffer is clear
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT reUseTX(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    CHK_RES(nrf24->write_register(nrf24, NRF_STATUS, _BV(MAX_RT), false)); //Clear max retry flag
    CHK_RES(nrf24->write_register(nrf24, REUSE_TX_PL, RF24_NOP, true));
    nrf24->ce(nrf24, RF_LOW); //Re-Transfer packet
    nrf24->ce(nrf24, RF_HIGH);
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT writeFast_data(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast)
{
    GB_RESULT res = GB_OK;
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retrys and set a timer or failure counter if required
    //The radio will auto-clear everything in the FIFO as long as CE remains high

    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((nrf24->get_status(nrf24) & (_BV(TX_FULL))))
    {
        if (nrf24->status & _BV(MAX_RT))
        {
            return GB_RF24_W_FAST_FAIL; //Return 0. The previous payload has not been retransmitted
            // From the user perspective, if you get a 0, just keep trying to send the same payload
        }
    }
    CHK_RES(nrf24->startFastWrite(nrf24, buf, len, multicast, true)); // Start Writing
error_exit:
    return res;
}

static GB_RESULT writeFast(struct rf24 *nrf24, const void *buf, uint8_t len)
{
    return nrf24->writeFast_data(nrf24, buf, len, 0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

static GB_RESULT startFastWrite(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast, bool startTx)
{
    GB_RESULT res = GB_OK;

    CHK_RES(nrf24->write_payload(nrf24, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD));
    //TMRh20
    if (startTx)
    {
        nrf24->ce(nrf24, RF_HIGH);
    }
error_exit:
    return res;
}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
static GB_RESULT startWrite(struct rf24 *nrf24, const void *buf, uint8_t len, const bool multicast)
{
    GB_RESULT res = GB_OK;
    // Send the payload
    CHK_RES(nrf24->write_payload(nrf24, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD));
    nrf24->ce(nrf24, RF_HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    nrf24->ce(nrf24, RF_LOW);
    CHK_BOOL(!(nrf24->status & _BV(TX_FULL)));
error_exit:
    return res;
}

/****************************************************************************/

static bool rxFifoFull(struct rf24 *nrf24)
{
    return read_register(nrf24, FIFO_STATUS) & _BV(RX_FULL);
}

/****************************************************************************/

static bool txStandBy(struct rf24 *nrf24)
{
    while (!(nrf24->read_register(nrf24, FIFO_STATUS) & _BV(TX_EMPTY)))
    {
        if (nrf24->status & _BV(MAX_RT))
        {
            nrf24->write_register(nrf24, NRF_STATUS, _BV(MAX_RT), false);
            nrf24->ce(nrf24, RF_LOW);
            nrf24->flush_tx(nrf24); //Non blocking, flush the data
            return 0;
        }
    }

    nrf24->ce(nrf24, RF_LOW); //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

static bool txStandByAfter(struct rf24 *nrf24, uint32_t timeout, bool startTx)
{
    if (startTx)
    {
        nrf24->stopListening(nrf24);
        nrf24->ce(nrf24, RF_HIGH);
    }

    uint64_t start_timer = 0;
    uint64_t end_timer = 0;

    GB_GetTimerMs(&start_timer);
    while (!(nrf24->read_register(nrf24, FIFO_STATUS) & _BV(TX_EMPTY)))
    {
        if (nrf24->status & _BV(MAX_RT))
        {
            nrf24->write_register(nrf24, NRF_STATUS, _BV(MAX_RT), false);
            nrf24->ce(nrf24, RF_LOW); // Set re-transmit
            nrf24->ce(nrf24, RF_HIGH);
            GB_GetTimerMs(&end_timer);
            if (end_timer - start_timer >= timeout)
            {
                nrf24->ce(nrf24, RF_LOW);
                nrf24->flush_tx(nrf24);
                return 0;
            }
        }
    }

    nrf24->ce(nrf24, RF_LOW); //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

static GB_RESULT maskIRQ(struct rf24 *nrf24, bool tx, bool fail, bool rx)
{
    /* clear the interrupt flags */
    nrf24->config_reg = (nrf24->config_reg & ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR));
    /* set the specified interrupt flags */
    nrf24->config_reg = (nrf24->config_reg | fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR);
    return nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false);
}

/****************************************************************************/

static uint8_t getDynamicPayloadSize(struct rf24 *nrf24)
{
    uint8_t result = nrf24->read_register(nrf24, R_RX_PL_WID);

    if (result > 32)
    {
        nrf24->flush_rx(nrf24);
        vTaskDelay(2 / portTICK_PERIOD_MS);
        return 0;
    }
    return result;
}

/****************************************************************************/

static bool isAvailable(struct rf24 *nrf24)
{
    return nrf24->available(nrf24, NULL);
}

/****************************************************************************/

static bool available(struct rf24 *nrf24, uint8_t *pipe_num)
{
    // get implied RX FIFO empty flag from status byte
    uint8_t status = nrf24->get_status(nrf24);

    GB_DEBUGV(RF24_TAG, "available status: %02x, fifo status: %02x\r\n", status, nrf24->read_register(nrf24, FIFO_STATUS));
    uint8_t pipe = (status >> RX_P_NO) & 0x07;
    if (pipe > 5)
        return 0;

    // If the caller wants the pipe number, include that
    if (pipe_num)
        *pipe_num = pipe;

    return 1;
}

/****************************************************************************/

static GB_RESULT read(struct rf24 *nrf24, void *buf, uint8_t len)
{
    GB_RESULT res = GB_OK;
    // Fetch the payload
    CHK_RES(nrf24->read_payload(nrf24, buf, len));

    //Clear the only applicable interrupt flags
    CHK_RES(nrf24->write_register(nrf24, NRF_STATUS, _BV(RX_DR), false));
error_exit:
    return res;
}

/****************************************************************************/

static void whatHappened(struct rf24 *nrf24, bool *tx_ok, bool *tx_fail, bool *rx_ready)
{
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    nrf24->write_register(nrf24, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), false);

    // Report to the user what happened
    *tx_ok = nrf24->status & _BV(TX_DS);
    *tx_fail = nrf24->status & _BV(MAX_RT);
    *rx_ready = nrf24->status & _BV(RX_DR);
}

/****************************************************************************/

static GB_RESULT openWritingPipe(struct rf24 *nrf24, uint64_t value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    GB_RESULT res = GB_OK;
    CHK_RES(nrf24->write_registers(nrf24, RX_ADDR_P0, (uint8_t*)&value, nrf24->addr_width));
    CHK_RES(nrf24->write_registers(nrf24, TX_ADDR, (uint8_t*)&value, nrf24->addr_width));
error_exit:
    return res;
}

/****************************************************************************/
static GB_RESULT openWritingPipeAddr(struct rf24 *nrf24, const uint8_t *address)
{
    GB_RESULT res = GB_OK;
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    CHK_RES(nrf24->write_registers(nrf24, RX_ADDR_P0, address, nrf24->addr_width));
    CHK_RES(nrf24->write_registers(nrf24, TX_ADDR, address, nrf24->addr_width));
error_exit:
    return res;
}

/****************************************************************************/
static const  uint8_t child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2,
                                             RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

static GB_RESULT openReadingPipe(struct rf24 *nrf24, uint8_t child, uint64_t address)
{
    GB_RESULT res = GB_OK;
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0)
    {
        memcpy(nrf24->pipe0_reading_address, &address, nrf24->addr_width);
        nrf24->_is_p0_rx = true;
    }

    if (child <= 5)
    {
        // For pipes 2-5, only write the LSB
        if (child < 2)
        {
            CHK_RES(nrf24->write_registers(nrf24, pgm_read_byte(&child_pipe[child]), (uint8_t*)(&address), nrf24->addr_width));
        }
        else
        {
            CHK_RES(nrf24->write_registers(nrf24, pgm_read_byte(&child_pipe[child]), (uint8_t*)(&address), 1));
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        CHK_RES(nrf24->write_register(nrf24, EN_RXADDR, (nrf24->read_register(nrf24, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child]))), false));
    }
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT setAddressWidth(struct rf24 *nrf24, uint8_t a_width)
{
    GB_RESULT res = GB_OK;
    a_width = a_width - 2;
    if (a_width)
    {
        CHK_RES(nrf24->write_register(nrf24, SETUP_AW, a_width % 4, false));
        nrf24->addr_width = (a_width % 4) + 2;
    }
    else
    {
        CHK_RES(nrf24->write_register(nrf24, SETUP_AW, 0, false));
        nrf24->addr_width = 2;
    }
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT openReadingPipeAddr(struct rf24 *nrf24, uint8_t child, const uint8_t *address)
{
    GB_RESULT res = GB_OK;
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0)
    {
        memcpy(nrf24->pipe0_reading_address, address, nrf24->addr_width);
        nrf24->_is_p0_rx = true;
    }
    if (child <= 5)
    {
        // For pipes 2-5, only write the LSB
        if (child < 2)
        {
            CHK_RES(nrf24->write_registers(nrf24, pgm_read_byte(&child_pipe[child]), address, nrf24->addr_width));
        }
        else
        {
            CHK_RES(nrf24->write_registers(nrf24, pgm_read_byte(&child_pipe[child]), address, 1));
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        CHK_RES(nrf24->write_register(nrf24, EN_RXADDR, (nrf24->read_register(nrf24, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child]))), false));
    }
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT closeReadingPipe(struct rf24 *nrf24, uint8_t pipe)
{
    GB_RESULT res = GB_OK;

    CHK_RES(nrf24->write_register(nrf24, EN_RXADDR, (nrf24->read_register(nrf24, EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe]))), false));
    if (!pipe)
    {
        // keep track of pipe 0's RX state to avoid null vs 0 in addr cache
        nrf24->_is_p0_rx = false;
    }
error_exit:
    return res;
}

/****************************************************************************/

static GB_RESULT toggle_features(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    uint8_t command = ACTIVATE;

    CHK_RES(nrf24->bus->readWriteBytes(nrf24->bus, nrf24->addr, 0x00, 1, &(nrf24->status), &command));
    CHK_RES(nrf24->bus->writeByte(nrf24->bus, nrf24->addr, 0x00, 0x73));
error_exit:
    return res;
}

/****************************************************************************/

static void enableDynamicPayloads(struct rf24 *nrf24)
{
    // Enable dynamic payload throughout the system

    //toggle_features();
    nrf24->write_register(nrf24, FEATURE, nrf24->read_register(nrf24, FEATURE) | _BV(EN_DPL), false);

    GB_DEBUGD(RF24_TAG, "FEATURE=%i\r\n", nrf24->read_register(nrf24, FEATURE));

    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    nrf24->write_register(nrf24, DYNPD, nrf24->read_register(nrf24, DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0), false);

    nrf24->dynamic_payloads_enabled = true;
}

/****************************************************************************/
static void disableDynamicPayloads(struct rf24 *nrf24)
{
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads

    //toggle_features();
    nrf24->write_register(nrf24, FEATURE, 0, false);

    GB_DEBUGD(RF24_TAG, "FEATURE=%i\r\n", nrf24->read_register(nrf24, FEATURE));

    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    nrf24->write_register(nrf24, DYNPD, 0, false);

    nrf24->dynamic_payloads_enabled = false;
    nrf24->ack_payloads_enabled = false;
}

/****************************************************************************/

static void enableAckPayload(struct rf24 *nrf24)
{
    // enable ack payloads and dynamic payload features

    if (!nrf24->ack_payloads_enabled)
    {
        nrf24->write_register(nrf24, FEATURE, nrf24->read_register(nrf24, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL), false);

        GB_DEBUGD(RF24_TAG, "FEATURE=%i\r\n", nrf24->read_register(nrf24, FEATURE));

        // Enable dynamic payload on pipes 0 & 1
        nrf24->write_register(nrf24, DYNPD, nrf24->read_register(nrf24, DYNPD) | _BV(DPL_P1) | _BV(DPL_P0), false);
        nrf24->dynamic_payloads_enabled = true;
        nrf24->ack_payloads_enabled = true;
    }
}

/****************************************************************************/

static void disableAckPayload(struct rf24 *nrf24)
{
    // disable ack payloads (leave dynamic payload features as is)
    if (nrf24->ack_payloads_enabled)
    {
        nrf24->write_register(nrf24, FEATURE, nrf24->read_register(nrf24, FEATURE) | ~_BV(EN_ACK_PAY), false);

        GB_DEBUGD(RF24_TAG, "FEATURE=%i\r\n", nrf24->read_register(nrf24, FEATURE));

        nrf24->ack_payloads_enabled = false;
    }
}

/****************************************************************************/

static void enableDynamicAck(struct rf24 *nrf24)
{
    //
    // enable dynamic ack features
    //
    //toggle_features();
    nrf24->write_register(nrf24, FEATURE, nrf24->read_register(nrf24, FEATURE) | _BV(EN_DYN_ACK), false);

    GB_DEBUGD(RF24_TAG, "FEATURE=%i\r\n", nrf24->read_register(nrf24, FEATURE));
}

/****************************************************************************/

static bool writeAckPayload(struct rf24 *nrf24, uint8_t pipe, const void *buf, uint8_t len)
{
    if (nrf24->ack_payloads_enabled)
    {
        const uint8_t *current = (buf);

        nrf24->write_payload(nrf24, current, len, W_ACK_PAYLOAD | (pipe & 0x07));
        return !(nrf24->status & _BV(TX_FULL));
    }
    return 0;
}

/****************************************************************************/

static bool isAckPayloadAvailable(struct rf24 *nrf24)
{
    return nrf24->available(nrf24, NULL);
}

/****************************************************************************/

static bool isPVariant(struct rf24 *nrf24)
{
    return nrf24->_is_p_variant;
}

/****************************************************************************/

static void setAutoAck(struct rf24 *nrf24, bool enable)
{
    if (enable)
    {
        nrf24->write_register(nrf24, EN_AA, 0x3F, false);
    }
    else
    {
        nrf24->write_register(nrf24, EN_AA, 0, false);
        // accomodate ACK payloads feature
        if (nrf24->ack_payloads_enabled)
        {
            nrf24->disableAckPayload(nrf24);
        }
    }
}

/****************************************************************************/

static void setAutoAckPipe(struct rf24 *nrf24, uint8_t pipe, bool enable)
{
    if (pipe < 6)
    {
        uint8_t en_aa = nrf24->read_register(nrf24, EN_AA);
        if (enable)
        {
            en_aa |= (_BV(pipe));
        }
        else
        {
            en_aa = (en_aa & ~_BV(pipe));
            if (nrf24->ack_payloads_enabled && !pipe)
            {
                nrf24->disableAckPayload(nrf24);
            }
        }
        nrf24->write_register(nrf24, EN_AA, en_aa, false);
    }
}

/****************************************************************************/

static bool testCarrier(struct rf24 *nrf24)
{
    return (nrf24->read_register(nrf24, CD) & 1);
}

/****************************************************************************/

static bool testRPD(struct rf24 *nrf24)
{
    return (nrf24->read_register(nrf24, RPD) & 1);
}

/****************************************************************************/

static GB_RESULT setPALevel(struct rf24 *nrf24, uint8_t level, bool lnaEnable)
{
    uint8_t setup = nrf24->read_register(nrf24, RF_SETUP) & (0xF8);
    setup |= nrf24->_pa_level_reg_value(nrf24, level, lnaEnable);
    return nrf24->write_register(nrf24, RF_SETUP, setup, false);
}

/****************************************************************************/

static uint8_t getPALevel(struct rf24 *nrf24)
{
    return (((nrf24->read_register(nrf24, RF_SETUP))) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/

static uint8_t getARC(struct rf24 *nrf24)
{
    return nrf24->read_register(nrf24, OBSERVE_TX) & 0x0F;
}

/****************************************************************************/

static bool setDataRate(struct rf24 *nrf24, rf24_datarate_e speed)
{
    bool result = false;
    uint8_t setup = nrf24->read_register(nrf24, RF_SETUP);

    // RF_HIGH and RF_LOW '00' is 1Mbs - our default
    setup = (setup & ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)));
    setup |= nrf24->_data_rate_reg_value(nrf24, speed);

    nrf24->write_register(nrf24, RF_SETUP, setup, false);

    // Verify our result
    if (nrf24->read_register(nrf24, RF_SETUP) == setup)
    {
        result = true;
    }
    return result;
}

/****************************************************************************/

static rf24_datarate_e getDataRate(struct rf24 *nrf24)
{
    GB_RESULT res = GB_OK;
    rf24_datarate_e result;
    uint8_t dr = (nrf24->read_register(nrf24, RF_SETUP)) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW))
    {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    }
    else if (dr == _BV(RF_DR_HIGH))
    {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    }
    else
    {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }

    CHK_LOGE(res, "NRF24 get data rate failed\n");
    return result;
}

/****************************************************************************/

static void setCRCLength(struct rf24 *nrf24, rf24_crclength_e length)
{
    nrf24->config_reg = (nrf24->config_reg & ~(_BV(CRCO) | _BV(EN_CRC)));

    // switch uses RAM (evil!)
    if (length == RF24_CRC_DISABLED)
    {
        // Do nothing, we turned it off above.
    }
    else if (length == RF24_CRC_8)
    {
        nrf24->config_reg |= _BV(EN_CRC);
    }
    else
    {
        nrf24->config_reg |= _BV(EN_CRC);
        nrf24->config_reg |= _BV(CRCO);
    }
    nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false);
}

/****************************************************************************/

static rf24_crclength_e getCRCLength(struct rf24 *nrf24)
{
    rf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t AA = nrf24->read_register(nrf24, EN_AA);
    nrf24->config_reg = nrf24->read_register(nrf24, NRF_CONFIG);

    if (nrf24->config_reg & _BV(EN_CRC) || AA)
    {
        if (nrf24->config_reg & _BV(CRCO))
        {
            result = RF24_CRC_16;
        }
        else
        {
            result = RF24_CRC_8;
        }
    }
    return result;
}

/****************************************************************************/

static void disableCRC(struct rf24 *nrf24)
{
    nrf24->config_reg = (nrf24->config_reg & ~_BV(EN_CRC));
    nrf24->write_register(nrf24, NRF_CONFIG, nrf24->config_reg, false);
}

/****************************************************************************/
static GB_RESULT setRetries(struct rf24 *nrf24, uint8_t delay, uint8_t count)
{
    uint8_t value = rf24_min(15, delay) << ARD | rf24_min(15, count);
    return nrf24->write_register(nrf24, SETUP_RETR, value, false);
}

/****************************************************************************/
static void startConstCarrier(struct rf24 *nrf24, rf24_pa_dbm_e level, uint8_t channel)
{
    nrf24->stopListening(nrf24);
    nrf24->write_register(nrf24, RF_SETUP, nrf24->read_register(nrf24, RF_SETUP) | _BV(CONT_WAVE) | _BV(PLL_LOCK), false);
    if (nrf24->isPVariant(nrf24))
    {
        nrf24->setAutoAck(nrf24, 0);
        nrf24->setRetries(nrf24, 0, 0);
        uint8_t dummy_buf[32];
        for (uint8_t i = 0; i < 32; ++i)
            dummy_buf[i] = 0xFF;

        // use write_register() instead of openWritingPipe() to bypass
        // truncation of the address with the current RF24::addr_width value
        nrf24->write_registers(nrf24, TX_ADDR, dummy_buf, 5);
        nrf24->flush_tx(nrf24); // so we can write to top level

        // use write_register() instead of write_payload() to bypass
        // truncation of the payload with the current RF24::payload_size value
        nrf24->write_registers(nrf24, W_TX_PAYLOAD, dummy_buf, 32);

        nrf24->disableCRC(nrf24);
    }
    nrf24->setPALevel(nrf24, level, true);
    nrf24->setChannel(nrf24, channel);
    GB_DEBUGD(RF24_TAG, PSTR("RF_SETUP=%02x\r\n"), nrf24->read_register(nrf24, RF_SETUP));
    nrf24->ce(nrf24, RF_HIGH);
    if (nrf24->isPVariant(nrf24))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        nrf24->ce(nrf24, RF_LOW);
        nrf24->reUseTX(nrf24);
    }
}

/****************************************************************************/

static void stopConstCarrier(struct rf24 *nrf24)
{
    /*
     * A note from the datasheet:
     * Do not use REUSE_TX_PL together with CONT_WAVE=1. When both these
     * registers are set the chip does not react when setting CE low. If
     * however, both registers are set PWR_UP = 0 will turn TX mode off.
     */
    nrf24->powerDown(nrf24); // per datasheet recommendation (just to be safe)
    nrf24->write_register(nrf24, RF_SETUP, nrf24->read_register(nrf24, RF_SETUP) & ~_BV(CONT_WAVE) & ~_BV(PLL_LOCK), false);
    nrf24->ce(nrf24, RF_LOW);
}

/****************************************************************************/

static void toggleAllPipes(struct rf24 *nrf24, bool isEnabled)
{
    nrf24->write_register(nrf24, EN_RXADDR, (isEnabled ? 0x3F : 0), false);
}

/****************************************************************************/

static uint8_t _data_rate_reg_value(struct rf24 *nrf24, rf24_datarate_e speed)
{
    nrf24->txDelay = 280;
    if (speed == RF24_250KBPS)
    {
        nrf24->txDelay = 505;
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        return _BV(RF_DR_LOW);
    }
    else if (speed == RF24_2MBPS)
    {
        nrf24->txDelay = 240;
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        return _BV(RF_DR_HIGH);
    }
    // RF_HIGH and RF_LOW '00' is 1Mbs - our default
    return 0;
}

/****************************************************************************/

static uint8_t _pa_level_reg_value(struct rf24 *nrf24, uint8_t level, bool lnaEnable)
{
    // If invalid level, go to max PA
    // Else set level as requested
    // + lnaEnable (1 or 0) to support the SI24R1 chip extra bit
    return ((level > RF24_PA_MAX ? RF24_PA_MAX : level) << 1) + lnaEnable;
}

/****************************************************************************/

static GB_RESULT setRadiation(struct rf24 *nrf24, uint8_t level, rf24_datarate_e speed, bool lnaEnable)
{
    uint8_t setup = nrf24->_data_rate_reg_value(nrf24, speed);
    setup |= nrf24->_pa_level_reg_value(nrf24, level, lnaEnable);
    return nrf24->write_register(nrf24, RF_SETUP, setup, false);
}
