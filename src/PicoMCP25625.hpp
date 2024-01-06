//
//    MCP25625 library for the Raspberry Pi Pico
//
//    Copyleft (C) 2024 Okawa Yusuke
//

#pragma once

#include <stdint.h>

#include <pico/stdlib.h>
#include <hardware/spi.h>

class PicoMCP25625
{

public:

    struct Config
    {

        // --------------------------------------------------------------
        //  SPI
        // --------------------------------------------------------------

        spi_inst_t* spiChannel;              // SPI channel (spi0 or spi1)
        uint32_t    spiClock = 1'000'000;    // SPI clock frequency [Hz]

        uint8_t pinCS;                                 // CS        pin
        uint8_t pinMOSI = PICO_DEFAULT_SPI_TX_PIN;     // MOSI (TX) pin
        uint8_t pinMISO = PICO_DEFAULT_SPI_RX_PIN;     // MISO (RX) pin
        uint8_t pinSCK  = PICO_DEFAULT_SPI_SCK_PIN;    // SCK       pin

        // --------------------------------------------------------------
        //  CAN
        // --------------------------------------------------------------

        uint8_t pinInterrupt;    // Receive interrupt pin

        enum class CanBaudrate : uint8_t
        {
            CAN_10KBPS,
            CAN_20KBPS,
            CAN_50KBPS,
            CAN_100KBPS,
            CAN_125KBPS,
            CAN_250KBPS,
            CAN_500KBPS,
            CAN_1MBPS,
        };

        CanBaudrate canBaudrate = CanBaudrate::CAN_1MBPS;    // CAN bus baudrate [bps]

        enum class CanMode : uint8_t
        {
            NORMAL,
            LOOPBACK,
            SLEEP,
            LISTEN_ONLY,
            CONFIG,
        };

        enum class ControllerClock : uint8_t
        {
            MCP_8MHZ,
            MCP_16MHZ,
        };

        ControllerClock controllerClock = ControllerClock::MCP_8MHZ;    // controller clock [MHz]
    };

    struct CanMessage
    {
        static constexpr size_t MAX_DATA_LENGTH = 8;

        uint16_t id;
        uint8_t  length;
        uint8_t  data[MAX_DATA_LENGTH];
    };

    PicoMCP25625() noexcept;

    /// @brief begin CAN and SPI communication
    void begin() noexcept;

    /// @brief begin CAN communication only
    /// @note SPI communication must be started by yourself
    void beginCanOnly() noexcept;

    void end() noexcept;

    void setMessage(const CanMessage& message) noexcept;

    CanMessage getMessage() const noexcept;
};

#include "PicoMCP25625Impl.hpp"
