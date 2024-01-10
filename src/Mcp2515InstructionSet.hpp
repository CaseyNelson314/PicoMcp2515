#pragma once

#include <hardware/gpio.h>
#include <hardware/spi.h>

#include <type_traits>


/// @brief MCP2515 instruction set
class InstructionSet
{

    /// @brief SPI channel (spi0 or spi1)
    spi_inst_t* channel;

    /// @brief CS pin
    uint8_t cs;

public:
    InstructionSet() noexcept
        : channel(spi_default)
        , cs(PICO_DEFAULT_SPI_CSN_PIN)
    {
    }

    InstructionSet(spi_inst_t* channel, uint8_t cs) noexcept
        : channel(channel)
        , cs(cs)
    {
    }

    void setChannel(spi_inst_t* channel) noexcept { this->channel = channel; }

    void setCs(uint8_t cs) noexcept { this->cs = cs; }


    //--------------------------------------------------------------------------------
    //  Register Map
    //--------------------------------------------------------------------------------

    /// @brief Control Register Map
    enum class Register : uint8_t
    {
        // clang-format off

        /* low  */  /* high       0000                     0001                     0010                     0011                     0100                     0101                     0110                     0111      */
        /* 0000 */  RXF0SIDH  = 0b0000'0000,  RXF3SIDH = 0b0001'0000,  RXM0SIDH = 0b0010'0000,  TXB0CTRL = 0b0011'0000,  TXB1CTRL = 0b0100'0000,  TXB2CTRL = 0b0101'0000,  RXB0CTRL = 0b0110'0000,  RXB1CTRL = 0b0111'0000,
        /* 0001 */  RXF0SIDL  = 0b0000'0001,  RXF3SIDL = 0b0001'0001,  RXM0SIDL = 0b0010'0001,  TXB0SIDH = 0b0011'0001,  TXB1SIDH = 0b0100'0001,  TXB2SIDH = 0b0101'0001,  RXB0SIDH = 0b0110'0001,  RXB1SIDH = 0b0111'0001,
        /* 0010 */  RXF0EID8  = 0b0000'0010,  RXF3EID8 = 0b0001'0010,  RXM0EID8 = 0b0010'0010,  TXB0SIDL = 0b0011'0010,  TXB1SIDL = 0b0100'0010,  TXB2SIDL = 0b0101'0010,  RXB0SIDL = 0b0110'0010,  RXB1SIDL = 0b0111'0010,
        /* 0011 */  RXF0EID0  = 0b0000'0011,  RXF3EID0 = 0b0001'0011,  RXM0EID0 = 0b0010'0011,  TXB0EID8 = 0b0011'0011,  TXB1EID8 = 0b0100'0011,  TXB2EID8 = 0b0101'0011,  RXB0EID8 = 0b0110'0011,  RXB1EID8 = 0b0111'0011,
        /* 0100 */  RXF1SIDH  = 0b0000'0100,  RXF4SIDH = 0b0001'0100,  RXM1SIDH = 0b0010'0100,  TXB0EID0 = 0b0011'0100,  TXB1EID0 = 0b0100'0100,  TXB2EID0 = 0b0101'0100,  RXB0EID0 = 0b0110'0100,  RXB1EID0 = 0b0111'0100,
        /* 0101 */  RXF1SIDL  = 0b0000'0101,  RXF4SIDL = 0b0001'0101,  RXM1SIDL = 0b0010'0101,  TXB0DLC  = 0b0011'0101,  TXB1DLC  = 0b0100'0101,  TXB2DLC  = 0b0101'0101,  RXB0DLC  = 0b0110'0101,  RXB1DLC  = 0b0111'0101,
        /* 0110 */  RXF1EID8  = 0b0000'0110,  RXF4EID8 = 0b0001'0110,  RXM1EID8 = 0b0010'0110,  TXB0D0   = 0b0011'0110,  TXB1D0   = 0b0100'0110,  TXB2D0   = 0b0101'0110,  RXB0D0   = 0b0110'0110,  RXB1D0   = 0b0111'0110,
        /* 0111 */  RXF1EID0  = 0b0000'0111,  RXF4EID0 = 0b0001'0111,  RXM1EID0 = 0b0010'0111,  TXB0D1   = 0b0011'0111,  TXB1D1   = 0b0100'0111,  TXB2D1   = 0b0101'0111,  RXB0D1   = 0b0110'0111,  RXB1D1   = 0b0111'0111,
        /* 1000 */  RXF2SIDH  = 0b0000'1000,  RXF5SIDH = 0b0001'1000,  CNF3     = 0b0010'1000,  TXB0D2   = 0b0011'1000,  TXB1D2   = 0b0100'1000,  TXB2D2   = 0b0101'1000,  RXB0D2   = 0b0110'1000,  RXB1D2   = 0b0111'1000,
        /* 1001 */  RXF2SIDL  = 0b0000'1001,  RXF5SIDL = 0b0001'1001,  CNF2     = 0b0010'1001,  TXB0D3   = 0b0011'1001,  TXB1D3   = 0b0100'1001,  TXB2D3   = 0b0101'1001,  RXB0D3   = 0b0110'1001,  RXB1D3   = 0b0111'1001,
        /* 1010 */  RXF2EID8  = 0b0000'1010,  RXF5EID8 = 0b0001'1010,  CNF1     = 0b0010'1010,  TXB0D4   = 0b0011'1010,  TXB1D4   = 0b0100'1010,  TXB2D4   = 0b0101'1010,  RXB0D4   = 0b0110'1010,  RXB1D4   = 0b0111'1010,
        /* 1011 */  RXF2EID0  = 0b0000'1011,  RXF5EID0 = 0b0001'1011,  CANINTE  = 0b0010'1011,  TXB0D5   = 0b0011'1011,  TXB1D5   = 0b0100'1011,  TXB2D5   = 0b0101'1011,  RXB0D5   = 0b0110'1011,  RXB1D5   = 0b0111'1011,
        /* 1100 */  BFPCTRL   = 0b0000'1100,  TEC      = 0b0001'1100,  CANINTF  = 0b0010'1100,  TXB0D6   = 0b0011'1100,  TXB1D6   = 0b0100'1100,  TXB2D6   = 0b0101'1100,  RXB0D6   = 0b0110'1100,  RXB1D6   = 0b0111'1100,
        /* 1101 */  TXRTSCTRL = 0b0000'1101,  REC      = 0b0001'1101,  EFLG     = 0b0010'1101,  TXB0D7   = 0b0011'1101,  TXB1D7   = 0b0100'1101,  TXB2D7   = 0b0101'1101,  RXB0D7   = 0b0110'1101,  RXB1D7   = 0b0111'1101,
        /* 1110 */  CANSTAT1  = 0b0000'1110,  CANSTAT2 = 0b0001'1110,  CANSTAT3 = 0b0010'1110,  CANSTAT4 = 0b0011'1110,  CANSTAT5 = 0b0100'1110,  CANSTAT6 = 0b0101'1110,  CANSTAT7 = 0b0110'1110,  CANSTAT8 = 0b0111'1110,
        /* 1111 */  CANCTRL1  = 0b0000'1111,  CANCTRL2 = 0b0001'1111,  CANCTRL3 = 0b0010'1111,  CANCTRL4 = 0b0011'1111,  CANCTRL5 = 0b0100'1111,  CANCTRL6 = 0b0101'1111,  CANCTRL7 = 0b0110'1111,  CANCTRL8 = 0b0111'1111,

        // clang-format on
    };

    /// @brief Control Register Bitmask Map
    /// @note For registers with bit-by-bit set values
    enum class RegisterBitmask : uint8_t
    {
        // clang-format off

        /* Register  */
        /* BFPCTRL   */                                                 B1BFS   = 0b0010'0000,  B0BFS   = 0b0001'0000,  B1BFE   = 0b0000'1000,  B0BFE   = 0b0000'0100,  B1BFM   = 0b0000'0010,  B0BFM   = 0b0000'0001,
        /* TXRTSCTRL */                                                 B2RTS   = 0b0010'0000,  B1RTS   = 0b0001'0000,  B0RTS   = 0b0000'1000,  B2RTSM  = 0b0000'0100,  B1RTSM  = 0b0000'0010,  B0RTSM  = 0b0000'0001,
        /* CANSTATn  */  OPMOD2  = 0b1000'0000,  OPMOD1 = 0b0100'0000,  OPMOD0  = 0b0010'0000,                          ICOD2   = 0b0000'1000,  ICOD1   = 0b0000'0100,  ICOD0   = 0b0000'0010, 
        /* CANCTRLn  */  REQOP2  = 0b1000'0000,  REQOP1 = 0b0100'0000,  REQOP0  = 0b0010'0000,  ABAT    = 0b0001'0000,  CLKEN   = 0b0000'1000,  CLKPCE  = 0b0000'0100,  CLKPRE1 = 0b0000'0010,  CLKPRE0 = 0b0000'0001,
        /* CNF3      */  SOF     = 0b1000'0000,  WAKFIL = 0b0010'0000,                                                                          PHSEG22 = 0b0000'0100,  PHSEG21 = 0b0000'0010,  PHSEG20 = 0b0000'0001,
        /* CNF2      */  BTLMODE = 0b1000'0000,  SAM    = 0b0100'0000,  PHSEG12 = 0b0010'0000,  PHSEG11 = 0b0001'0000,  PHSEG10 = 0b0000'1000,  PRSEG2  = 0b0000'0100,  PRSEG1  = 0b0000'0010,  PRSEG0  = 0b0000'0001,
        /* CNF1      */  SJW1    = 0b1000'0000,  SJW0   = 0b0100'0000,  BRP5    = 0b0010'0000,  BRP4    = 0b0001'0000,  BRP3    = 0b0000'1000,  BRP2    = 0b0000'0100,  BRP1    = 0b0000'0010,  BRP0    = 0b0000'0001,
        /* CANINTE   */  MERRE   = 0b1000'0000,  WAKIE  = 0b0100'0000,  ERRIE   = 0b0010'0000,  TX2IE   = 0b0001'0000,  TX1IE   = 0b0000'1000,  TX0IE   = 0b0000'0100,  RX1IE   = 0b0000'0010,  RX0IE   = 0b0000'0001,
        /* CANINTF   */  MERRF   = 0b1000'0000,  WAKIF  = 0b0100'0000,  ERRIF   = 0b0010'0000,  TX2IF   = 0b0001'0000,  TX1IF   = 0b0000'1000,  TX0IF   = 0b0000'0100,  RX1IF   = 0b0000'0010,  RX0IF   = 0b0000'0001,
        /* EFLG      */  RX1OVR  = 0b1000'0000,  RX0OVR = 0b0100'0000,  TXBO    = 0b0010'0000,  TXEP    = 0b0001'0000,  RXEP    = 0b0000'1000,  TXWAR   = 0b0000'0100,  RXWAR   = 0b0000'0010,  EWARN   = 0b0000'0001,
        /* TXB0CTRL  */                          ABTF   = 0b0100'0000,  MLOA    = 0b0010'0000,  TXERR   = 0b0001'0000,  TXREQ   = 0b0000'1000,                          TXP1    = 0b0000'0010,  TXP0    = 0b0000'0001,
        /* TXB1CTRL  */                          /*      ABTF       */  /*      MLOA        */  /*      TXERR       */  /*      TXREQ       */                          /*      TXP1        */  /*      TXP0        */
        /* TXB2CTRL  */                          /*      ABTF       */  /*      MLOA        */  /*      TXERR       */  /*      TXREQ       */                          /*      TXP1        */  /*      TXP0        */
        /* RXB0CTRL  */                          RXM1   = 0b0010'0000,  RXM0    = 0b0001'0000,                          RXRTR   = 0b0000'1000,  BUKT0   = 0b0000'0100,  BUKT1   = 0b0000'0010,  FILHIT0 = 0b0000'0001,
        /* RXB1CTRL  */                          /*      RXM1       */  /*      RXM0        */                          /*      RXRTR       */  FILHIT2 = 0b0000'0100,  FILHIT1 = 0b0000'0010,  /*     FILHIT0      */

        // clang-format on
    };


    /// @brief Allow bit masks to be combined with the operator|.
    friend constexpr RegisterBitmask operator|(RegisterBitmask lhs, RegisterBitmask rhs) noexcept
    {
        return static_cast<RegisterBitmask>(AsUnderlying(lhs) | AsUnderlying(rhs));
    }

    /// @brief Transform enum class to underlying type.
    template <typename Enum>
    static constexpr std::underlying_type_t<Enum> AsUnderlying(Enum e) noexcept
    {
        return static_cast<std::underlying_type_t<Enum>>(e);
    }

    //--------------------------------------------------------------------------------
    //  SPI instruction set
    //--------------------------------------------------------------------------------

private:
    enum class Instruction : uint8_t
    {
        // resetInstruction()
        Reset = 0b1100'0000,

        // readInstruction()
        Read = 0b0000'0011,

        // readRxStatusInstruction()
        ReadStartRXB0SIDH = 0b10010'00'0,
        ReadStartRXB0D0   = 0b10010'01'0,
        ReadStartRXB1SIDH = 0b10010'10'0,
        ReadStartRXB1D0   = 0b10010'11'0,

        // writeInstruction()
        Write = 0b0000'0010,

        // writeTxBufferInstruction()
        WriteStartTXB0SIDH = 0b01000'00'0,
        WriteStartTXB0D0   = 0b01000'01'0,
        WriteStartTXB1SIDH = 0b01000'10'0,
        WriteStartTXB1D0   = 0b01000'11'0,
        WriteStartTXB2SIDH = 0b01001'00'0,
        WriteStartTXB2D0   = 0b01001'01'0,

        // requestToSendInstruction()
        RequestToSendTXB0 = 0b10000'001,
        RequestToSendTXB1 = 0b10000'010,
        RequestToSendTXB2 = 0b10000'100,

        // readStatusInstruction()
        ReadStatus = 0b1010'0000,

        // readRxStatusInstruction()
        ReadRxStatus = 0b1011'0000,

        // bitModifyInstruction()
        BitModify = 0b0000'0101,
    };

public:
    /// @brief Internal register initialization
    /// @note After initialization, it will be in configuration mode.
    /// @note Data sheet: p63 12.2
    inline void resetInstruction() noexcept
    {
        // SPI transmit format (1byte): [instruction]
        // SPI receive  format (0byte):

        const uint8_t instruction = AsUnderlying(Instruction::Reset);

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ &instruction,
                           /* size_t         len */ sizeof instruction);

        gpio_put(cs, true);
    }

    /// @brief Read 1 byte at a time 'in order from' the selected register
    /// @note Data sheet: p63 12.3
    inline void readInstruction(Register startAddress, uint8_t* dist, size_t length) noexcept
    {
        // SPI transmit format (2byte): [instruction] + [startAddress]
        // SPI receive  format (nbyte): [startAddress data] + [startAddress + 1 data] + ... (until CS is high)

        const uint8_t data[] = {
            AsUnderlying(Instruction::Read),
            AsUnderlying(startAddress),
        };

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ data,
                           /* size_t         len */ sizeof data);

        spi_read_blocking(/* spi_inst_t* spi              */ channel,
                          /* uint8_t     repeated_tx_data */ 0x00,
                          /* uint8_t*    dst              */ dist,
                          /* size_t      len              */ length);

        gpio_put(cs, true);
    }

    inline uint8_t readInstruction(Register startAddress) noexcept
    {
        uint8_t dist{};
        readInstruction(startAddress, &dist, sizeof dist);
        return dist;
    }

    /// @brief Read 1 byte at a time 'in order from' the selected register (For RX buffer register)
    /// @param startAddress register (RXB0SIDH, RXB1SIDH, RXB0D0, RXB1D0 only valid)
    /// @note Data sheet: p63 12.4
    /// @note Compared to instructionRead, the number of bytes sent is reduced and speed is improved by including register selection bits in the instruction.
    inline void readRxBufferInstruction(Register startAddress, uint8_t* dist, size_t length) noexcept
    {
        // SPI transmit format (1byte): [instruction | startAddress(2bit)]
        // SPI receive  format (nbyte): [startAddress data] + [startAddress + 1 data] + ... (until CS is high)

        uint8_t instruction;
        switch (startAddress)
        {
        case Register::RXB0SIDH: instruction = AsUnderlying(Instruction::ReadStartRXB0SIDH); break;
        case Register::RXB1SIDH: instruction = AsUnderlying(Instruction::ReadStartRXB1SIDH); break;
        case Register::RXB0D0: instruction = AsUnderlying(Instruction::ReadStartRXB0D0); break;
        case Register::RXB1D0: instruction = AsUnderlying(Instruction::ReadStartRXB1D0); break;
        default: return;
        }

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ &instruction,
                           /* size_t         len */ sizeof instruction);

        spi_read_blocking(/* spi_inst_t* spi              */ channel,
                          /* uint8_t     repeated_tx_data */ 0x00,
                          /* uint8_t*    dst              */ dist,
                          /* size_t      len              */ length);

        gpio_put(cs, true);
    }

    /// @brief Write 1 byte at a time 'in order from' the selected register
    /// @note Data sheet: p63 12.5
    inline void writeInstruction(Register startAddress, uint8_t const* src, size_t length) noexcept
    {
        // SPI transmit format (1byte): [instruction] + [startAddress] + [data] + [data] + ...
        // SPI receive  format (0byte): []

        const uint8_t data[] = {
            AsUnderlying(Instruction::Write),
            AsUnderlying(startAddress),
        };

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ data,
                           /* size_t         len */ sizeof data);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ src,
                           /* size_t         len */ length);

        gpio_put(cs, true);
    }

    inline void writeInstruction(Register startAddress, uint8_t src) noexcept
    {
        writeInstruction(startAddress, &src, sizeof src);
    }

    /// @brief Write 1 byte at a time 'in order from' the selected register (For TX buffer register)
    /// @param address register (TXB0SIDH, TXB1SIDH, TXB2SIDH, TXB0D0, TXB1D0, TXB2D0 only valid)
    /// @note Data sheet: p63 12.6
    /// @note Compared to instructionRead, the number of bytes sent is reduced and speed is improved by including register selection bits in the instruction.
    inline void writeTxBufferInstruction(Register startAddress, uint8_t const* src, size_t length) noexcept
    {
        // SPI transmit format (1byte): [instruction | startAddress(3bit)] + [data] + [data] + ...
        // SPI receive  format (0byte): []

        uint8_t instruction;
        switch (startAddress)
        {
        case Register::TXB0SIDH: instruction = AsUnderlying(Instruction::WriteStartTXB0SIDH); break;
        case Register::TXB1SIDH: instruction = AsUnderlying(Instruction::WriteStartTXB1SIDH); break;
        case Register::TXB2SIDH: instruction = AsUnderlying(Instruction::WriteStartTXB2SIDH); break;
        case Register::TXB0D0: instruction = AsUnderlying(Instruction::WriteStartTXB0D0); break;
        case Register::TXB1D0: instruction = AsUnderlying(Instruction::WriteStartTXB1D0); break;
        case Register::TXB2D0: instruction = AsUnderlying(Instruction::WriteStartTXB2D0); break;
        default: return;
        }

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ &instruction,
                           /* size_t         len */ sizeof instruction);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ src,
                           /* size_t         len */ length);

        gpio_put(cs, true);
    }

    enum class TxBuffer
    {
        TXB0,
        TXB1,
        TXB2,
    };

    /// @brief 送信要求
    /// @param txBuffer 送信バッファ
    /// @note Data sheet: p63 12.7
    inline void requestToSendInstruction(TxBuffer txBuffer) noexcept
    {
        // SPI transmit format (1byte): [instruction | txBuffer(3bit)]
        // SPI receive  format (0byte): []

        uint8_t instruction;
        switch (txBuffer)
        {
        case TxBuffer::TXB0: instruction = AsUnderlying(Instruction::RequestToSendTXB0); break;
        case TxBuffer::TXB1: instruction = AsUnderlying(Instruction::RequestToSendTXB1); break;
        case TxBuffer::TXB2: instruction = AsUnderlying(Instruction::RequestToSendTXB2); break;
        default: return;
        }

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ &instruction,
                           /* size_t         len */ sizeof instruction);

        gpio_put(cs, true);
    }

    struct PartialStatus
    {
        bool CANINTF_RX0IF : 1;
        bool CANINTFL_RX1IF : 1;
        bool TXB0CNTRL_TXREQ : 1;
        bool CANINTF_TX0IF : 1;
        bool TXB1CNTRL_TXREQ : 1;
        bool CANINTF_TX1IF : 1;
        bool TXB2CNTRL_TXREQ : 1;
        bool CANINTF_TX2IF : 1;
    };

    /// @brief 部分的な状態読み込み
    /// @note Data sheet: p64 12.8
    inline PartialStatus readStatusInstruction() noexcept
    {
        // SPI transmit format (1byte): [instruction]
        // SPI receive  format (1byte): [status] ... (same data until CS is high)

        const uint8_t instruction = AsUnderlying(Instruction::ReadStatus);

        uint8_t status{};

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ &instruction,
                           /* size_t         len */ sizeof instruction);

        spi_read_blocking(/* spi_inst_t* spi              */ channel,
                          /* uint8_t     repeated_tx_data */ 0x00,
                          /* uint8_t*    dst              */ &status,
                          /* size_t      len              */ sizeof status);

        gpio_put(cs, true);

        return {
            .CANINTF_RX0IF   = static_cast<bool>((status >> 0) & 0b1),
            .CANINTFL_RX1IF  = static_cast<bool>((status >> 1) & 0b1),
            .TXB0CNTRL_TXREQ = static_cast<bool>((status >> 2) & 0b1),
            .CANINTF_TX0IF   = static_cast<bool>((status >> 3) & 0b1),
            .TXB1CNTRL_TXREQ = static_cast<bool>((status >> 4) & 0b1),
            .CANINTF_TX1IF   = static_cast<bool>((status >> 5) & 0b1),
            .TXB2CNTRL_TXREQ = static_cast<bool>((status >> 6) & 0b1),
            .CANINTF_TX2IF   = static_cast<bool>((status >> 7) & 0b1),
        };
    }

    struct RxStatus
    {
        enum class MessageBuffer : uint8_t
        {
            NONE,
            RXB0,
            RXB1,
            BOTH,
        } messageBuffer;

        enum class MessageType : uint8_t
        {
            StandardData,
            StandardRemote,
            ExtendedData,
            ExtendedRemote,
        } messageType;

        enum class FilterMatch : uint8_t
        {
            RXF0,
            RXF1,
            RXF2,
            RXF3,
            RXF4,
            RXF5,
            RXF0Forwarded,    // RXB0に転送
            RXF1Forwarded,    // RXB1に転送
        } filterMatch;
    };

    /// @brief RX状態読み込み
    /// @note Data sheet: p64 12.9
    inline RxStatus readRxStatusInstruction() noexcept
    {
        // SPI transmit format (1byte): [instruction]
        // SPI receive  format (1byte): [status] ... (same data until CS is high)

        const uint8_t instruction = AsUnderlying(Instruction::ReadRxStatus);

        uint8_t status{};

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ &instruction,
                           /* size_t         len */ sizeof instruction);

        spi_read_blocking(/* spi_inst_t* spi              */ channel,
                          /* uint8_t     repeated_tx_data */ 0x00,
                          /* uint8_t*    dst              */ &status,
                          /* size_t      len              */ sizeof status);

        gpio_put(cs, true);

        // status: [7-6] Message Buffer, [4-3] Message Type, [2-0] Filter Match

        RxStatus::MessageBuffer messageBuffer;
        switch ((status >> 6) & 0b11)
        {
        case 0b00: messageBuffer = RxStatus::MessageBuffer::NONE; break;
        case 0b01: messageBuffer = RxStatus::MessageBuffer::RXB0; break;
        case 0b10: messageBuffer = RxStatus::MessageBuffer::RXB1; break;
        case 0b11: messageBuffer = RxStatus::MessageBuffer::BOTH; break;
        default: return {};    // unreachable
        }

        RxStatus::MessageType messageType;
        switch ((status >> 3) & 0b11)
        {
        case 0b00: messageType = RxStatus::MessageType::StandardData; break;
        case 0b01: messageType = RxStatus::MessageType::StandardRemote; break;
        case 0b10: messageType = RxStatus::MessageType::ExtendedData; break;
        case 0b11: messageType = RxStatus::MessageType::ExtendedRemote; break;
        default: return {};    // unreachable
        }

        RxStatus::FilterMatch filterMatch;
        switch (status & 0b111)
        {
        case 0b000: filterMatch = RxStatus::FilterMatch::RXF0; break;
        case 0b001: filterMatch = RxStatus::FilterMatch::RXF1; break;
        case 0b010: filterMatch = RxStatus::FilterMatch::RXF2; break;
        case 0b011: filterMatch = RxStatus::FilterMatch::RXF3; break;
        case 0b100: filterMatch = RxStatus::FilterMatch::RXF4; break;
        case 0b101: filterMatch = RxStatus::FilterMatch::RXF5; break;
        case 0b110: filterMatch = RxStatus::FilterMatch::RXF0Forwarded; break;
        case 0b111: filterMatch = RxStatus::FilterMatch::RXF1Forwarded; break;
        default: return {};    // unreachable
        }

        return { messageBuffer, messageType, filterMatch };
    }

    /// @brief ビット変更
    /// @param address register (BFPCTRL TXRTSCTRL CANSTATn CANCTRLn CNF3 CNF2 CNF1 CANINTE CANINTF EFLG TXB0CTRL TXB1CTRL TXB2CTRL RXB0CTRL RXB1CTRL only valid)
    /// @note Data sheet: p64 12.10
    inline void bitModifyInstruction(Register address, RegisterBitmask mask, uint8_t data) noexcept
    {
        // SPI transmit format (4byte): [instruction] + [address] + [mask] + [data]
        // SPI receive  format (0byte): []

        const uint8_t transmitData[] = {
            AsUnderlying(Instruction::BitModify),
            AsUnderlying(address),
            AsUnderlying(mask),
            data,
        };

        gpio_put(cs, false);

        spi_write_blocking(/* spi_inst_t*    spi */ channel,
                           /* const uint8_t* src */ transmitData,
                           /* size_t         len */ sizeof transmitData);

        gpio_put(cs, true);
    }
};
