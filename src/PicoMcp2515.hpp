//
//    MCP25625 library for the Raspberry Pi Pico
//
//    Copyright (C) 2024 Okawa Yusuke
//

#pragma once

#include "Mcp2515InstructionSet.hpp"
#include <cfloat>

#ifndef DEV_MODE
#    error this library is developing now
#endif

class PicoMcp2515
    : InstructionSet
{
public:
    struct Config
    {
        // SPI configuration
        spi_inst_t* spi      = PICO_DEFAULT_SPI_INSTANCE;    // SPI channel (spi0 or spi1)
        uint8_t     csPin    = PICO_DEFAULT_SPI_CSN_PIN;     // CS        pin
        uint8_t     mosiPin  = PICO_DEFAULT_SPI_TX_PIN;      // MOSI (TX) pin
        uint8_t     misoPin  = PICO_DEFAULT_SPI_RX_PIN;      // MISO (RX) pin
        uint8_t     sckPin   = PICO_DEFAULT_SPI_SCK_PIN;     // SCK       pin
        uint32_t    spiClock = 1'000'000;                    // SPI clock frequency [Hz]

        // CAN configuration
        uint32_t canBaudrate = 1'000'000;    // CAN bus baudrate [bps]

        // Oscillator configuration for CAN controller
        uint8_t  intPin;                   // Receive interrupt pin
        uint32_t oscClock = 16'000'000;    // External oscillator frequency [Hz]
    };

    struct CanMessage
    {
        static constexpr size_t MAX_DATA_LENGTH = 8;

        uint32_t id;                            // message ID (standard:11bit or extended:29bit)
        bool     extended = false;              // extended ID flag
        bool     remote   = false;              // remote frame flag
        size_t   length   = MAX_DATA_LENGTH;    // data length (0byte ~ 8byte)
        uint8_t  data[MAX_DATA_LENGTH];         // data
    };


    /**
     * @brief construct PicoMcp2515 instance
     * @param config configuration
     */
    PicoMcp2515(const Config& config) noexcept;


    /**
     * @brief begin CAN and SPI communication
     */
    bool begin() noexcept;


    /**
     * @brief end CAN and SPI communication
     */
    void end() noexcept;


    /**
     * @brief write CAN message
     * @param message CAN message
     * @return true: success, false: failure
     */
    bool writeMessage(const CanMessage& message) noexcept;


    /**
     * @brief Set the receive interrupt handler.
     * @param callback Receive Interrupt Handler. (const CanMessage&, void*) -> void
     * @param param    Arguments passed to the receive interrupt handler.
     * @note param is for passing 'this pointer', etc.
     */
    void onReceive(void (*callback)(const CanMessage&, void*), void* param = nullptr) noexcept;


    /**
     * @brief read CAN message
     * @return received CAN message
     */
    CanMessage readMessage() const noexcept;


    // 動作モード
    enum class OperationMode : uint8_t
    {
        Normal = 0b000'00000,
        // Sleep      = 0b001'00000,  // ウェイクアップ機能未実装
        Loopback   = 0b010'00000,
        ListenOnly = 0b011'00000,
        Config     = 0b100'00000,
    };

    bool setOperationMode(OperationMode mode) noexcept;

private:
    Config config;

    struct InterruptHandler
    {
        void (*callback)(const CanMessage&, void*) = {};
        void* param                                = {};

        void operator()(const CanMessage& message) const noexcept
        {
            if (callback)
            {
                callback(message, param);
            }
        }
    };
    InterruptHandler interruptHandler = {};
};


inline PicoMcp2515::PicoMcp2515(const Config& config) noexcept
    : InstructionSet{ config.spi, config.csPin }
    , config{ config }
{
}


inline bool PicoMcp2515::begin() noexcept
{
    // begin SPI communication
    {
        (void)spi_init(/* spi_inst_t* spi      */ config.spi,
                       /* uint        baudrate */ config.oscClock);

        spi_set_format(/* spi_inst_t* spi       */ config.spi,
                       /* uint        data_bits */ 8,
                       /* spi_cpol_t  cpol      */ SPI_CPOL_0,
                       /* spi_cpha_t  cpha      */ SPI_CPHA_0,
                       /* spi_order_t order     */ SPI_MSB_FIRST);

        spi_set_slave(/* spi_inst_t* spi   */ config.spi,
                      /* bool        slave */ false);

        gpio_set_function(config.mosiPin, GPIO_FUNC_SPI);
        gpio_set_function(config.misoPin, GPIO_FUNC_SPI);
        gpio_set_function(config.sckPin, GPIO_FUNC_SPI);

        gpio_init(config.csPin);
        gpio_set_dir(config.csPin, true);
        gpio_put(config.csPin, true);
    }

    // begin CAN communication
    {
        // MCP2515 Reset
        {
            InstructionSet::resetInstruction();
        }

        //
        {
            // CNF1 Register = SJW[2] | BRP[6]

            // Find the BRP with the smallest Tq count
            const auto nbt     = 1.f / config.canBaudrate;
            float      minDiff = FLT_MAX;
            uint8_t    brp     = 0;
            uint8_t    tqCount = 0;
            // uint8_t nbt     = 0;
            for (uint8_t testBrp = 0; testBrp < 0b111111; ++testBrp)
            {
                // Time Quantum (TQ)
                const auto tq = 2.f * (testBrp + 1) / config.oscClock;

                const auto testTqCount = nbt / tq;

                // Select BRPs near the center that fall within 8 ~ 25
                if (8 <= testTqCount && testTqCount <= 25)
                {
                    constexpr auto center = (8 + 25) / 2.f;

                    const auto diff = abs(testTqCount - center);

                    if (diff < minDiff)
                    {
                        minDiff = diff;
                        brp     = testBrp;
                        tqCount = testTqCount;
                    }
                }
            }
            if (minDiff == FLT_MAX)
            {
                // not found
                // TODO: error
                return false;
            }

            constexpr uint8_t sjw = 0b00;    // When a crystal oscillator is used, it is usually 1TQ (0b00)

            const uint8_t cnf1 = (sjw << 6) | (brp << 0);


            // CNF2 Register = BTLMODE[1] | SAM[1] | PHSEG1[3] | PRSEG[3]

            // Calculate segment times (PS1, PS2)
            // TqCount = SyncSeg + PropSeg + PS1 + PS2
            //       samplingPoint (60% ~ 70%) --^
            constexpr uint8_t syncSeg       = 1;
            constexpr uint8_t propSeg       = 2;
            constexpr uint8_t tDelay        = 2;    // 1TQ ~ 2TQ
            const float       samplingPoint = tqCount * ((0.6f + 0.7f) / 2.f);
            const uint8_t     ps1           = samplingPoint - syncSeg - propSeg;
            const uint8_t     ps2           = tqCount - ps1 - syncSeg - propSeg;

            // PS1 & PS2 の範囲をチェック
            if (propSeg + ps1 < ps2)
            {
                // TODO: error
                Serial.printf("propSeg + ps1 < ps2\n");
                return false;
            }
            if (propSeg + ps1 < tDelay)
            {
                // TODO: error
                Serial.printf("propSeg + ps1 < tDelay\n");
                return false;
            }
            if (ps2 <= sjw)
            {
                // TODO: error
                Serial.printf("propSeg + ps1 < tDelay\n");
                return false;
            }

            constexpr uint8_t btlmode = 0b1;
            constexpr uint8_t sam     = 0b0;            // Bus line sampling count 0: 1 time, 1: 3 times
            const uint8_t     phseg1  = ps1 - 1;        // Since TQ starts with 1, subtract 1
            constexpr uint8_t prseg   = propSeg - 1;    // Since TQ starts with 1, subtract 1

            const uint8_t cnf2 = (btlmode << 7) | (sam << 6) | (phseg1 << 3) | (prseg << 0);


            // CNF3 Register = SOF[1] | WAKFIL[1] | Uninstalled[3] | PHSEG2[3]
            constexpr uint8_t sof    = 0b0;        // 0: SOF disabled, 1: SOF enabled（No effect when CANCTRL.CLKEN is 0)
            constexpr uint8_t wakfil = 0b0;        // 0: Wake-up filter disabled, 1: Wake-up filter enabled
            const uint8_t     phseg2 = ps2 - 1;    // Since TQ starts with 1, subtract 1

            const uint8_t cnf3 = (sof << 7) | (wakfil << 6) | (phseg2 << 0);


            const uint8_t transmitData[] = { cnf3, cnf2, cnf1 };    // CNFn registers are in order, so send them at the same time
            InstructionSet::writeInstruction(Register::CNF3, transmitData, sizeof transmitData);
        }

        // 送信バッファの優先順位を設定
        {
            // Transmit priority: TXB0 < TXB1 < TXB2 (To send older data first. Data is set from TXB0)
            InstructionSet::bitModifyInstruction(Register::TXB0CTRL, RegisterBitmask::TXP, 0b0000'0000);
            InstructionSet::bitModifyInstruction(Register::TXB1CTRL, RegisterBitmask::TXP, 0b0000'0001);
            InstructionSet::bitModifyInstruction(Register::TXB2CTRL, RegisterBitmask::TXP, 0b0000'0010);
        }

        // 受信割り込み有効化
        {
            InstructionSet::bitModifyInstruction(Register::CANINTE,
                                                 RegisterBitmask::RX1IE | RegisterBitmask::RX0IE,
                                                 AsUnderlying(RegisterBitmask::RX1IE | RegisterBitmask::RX0IE));

            // GPIO interrupt configuration
            {
                gpio_init(config.intPin);
                gpio_set_dir(config.intPin, GPIO_IN);
                gpio_pull_up(config.intPin);
                gpio_set_irq_enabled_with_callback(config.intPin, GPIO_IRQ_EDGE_RISE, true, [](uint gpio, uint32_t event_mask)
                                                   { Serial.println("receive"); });
            }
        }

        // 通常モードに移行
        {
            if (not setOperationMode(OperationMode::Normal))
            {
                return false;
            }
        }

        return true;
    }
}


inline void PicoMcp2515::end() noexcept
{
    // end CAN communication
    {}

    // end SPI communication
    {
        spi_deinit(config.spi);
    }
}


inline bool PicoMcp2515::setOperationMode(OperationMode mode) noexcept
{
    const RegisterBitmask mask = RegisterBitmask::REQOP2 | RegisterBitmask::REQOP1 | RegisterBitmask::REQOP0;

    InstructionSet::bitModifyInstruction(Register::CANCTRL1, mask, AsUnderlying(mode));

    const uint8_t registerResult = InstructionSet::readInstruction(Register::CANSTAT1);

    return (registerResult & AsUnderlying(mask)) == AsUnderlying(mode);
}


inline bool PicoMcp2515::writeMessage(const CanMessage& message) noexcept
{
    // Check message arguments
    if (message.length > CanMessage::MAX_DATA_LENGTH)
    {
        // error: data length is too long
        return false;
    }
    if (message.extended)
    {
        if (message.id > 0x1FFFFFFF)    // 29bit
        {
            // error: extended ID is too long
            return false;
        }
    }
    else
    {
        if (message.id > 0x7FF)    // 11bit
        {
            // error: standard ID is too long
            return false;
        }
    }
    if (message.remote)
    {
        if (message.length > 0)
        {
            // error: remote frame must be 0 length
            return false;
        }
    }
    else
    {
        if (message.length == 0)
        {
            // error: data frame must be 1 ~ 8 length
            return false;
        }
    }


    // Determine the TX buffer to be used for transmission
    const auto status = InstructionSet::readStatusInstruction();
    TxBuffer   TXBn;
    Register   TXBnSIDH;
    Register   TXBnCTRL;
    if (not status.TXB0CNTRL_TXREQ)
    {
        // status.TXBnCNTRL_TXREQ == false: TXBn is free
        TXBn     = TxBuffer::TXB0;
        TXBnSIDH = Register::TXB0SIDH;
        TXBnCTRL = Register::TXB0CTRL;
    }
    else if (not status.TXB1CNTRL_TXREQ)
    {
        TXBn     = TxBuffer::TXB1;
        TXBnSIDH = Register::TXB1SIDH;
        TXBnCTRL = Register::TXB1CTRL;
    }
    else if (not status.TXB2CNTRL_TXREQ)
    {
        TXBn     = TxBuffer::TXB2;
        TXBnSIDH = Register::TXB2SIDH;
        TXBnCTRL = Register::TXB2CTRL;
    }
    else
    {
        // Transmission buffer is not free.
        return false;
    }


    uint8_t transmitData[13];
    if (message.extended)
    {
        // uint8_t message
        return false;    // TODO: not implemented
    }
    else
    {
        const uint8_t idh   = static_cast<uint8_t>(message.id >> 3);          // ID[10:3]
        const uint8_t idl   = static_cast<uint8_t>(message.id << 5);          // ID[2:0]
        const uint8_t exide = static_cast<uint8_t>(message.extended << 3);    // 0: Standard frame, 1: Extended frame

        transmitData[0] = idh;
        transmitData[1] = idl | exide;
    }

    const uint8_t rtr = static_cast<uint8_t>(message.remote << 6);    // 0: Data frame, 1: Remote frame
    const uint8_t dlc = static_cast<uint8_t>(message.length << 0);    // Data length code

    transmitData[4] = rtr | dlc;

    if (not message.remote)
    {
        memcpy(&transmitData[5], message.data, message.length);
    }

    // Write data to TX buffer
    InstructionSet::writeTxBufferInstruction(TXBnSIDH, transmitData, 5 + message.length);

    // transmit request
    InstructionSet::requestToSendInstruction(TXBn);

    const uint8_t transmitResult = InstructionSet::readInstruction(TXBnCTRL);

    if (transmitResult & AsUnderlying(RegisterBitmask::MLOA))
    {
        // error: Arbitration lost (other nodes on the bus are transmitting)
        return false;
    }

    if (transmitResult & AsUnderlying(RegisterBitmask::TXERR))
    {
        // error: Bus error occurred during message transmission.
        return false;
    }

    return true;
}


inline void PicoMcp2515::onReceive(void (*callback)(const PicoMcp2515::CanMessage&, void*), void* param) noexcept
{
    interruptHandler = { callback, param };
}


inline PicoMcp2515::CanMessage PicoMcp2515::readMessage() const noexcept
{
    return {};
}
