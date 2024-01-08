#pragma once

static PicoMcp2515 mcp2515;

void setup()
{
    const PicoMcp2515::SpiConfig spiConfig = {
        .channel = spi0,
        .cs      = 5,
        .mosi    = 3,
        .miso    = 4,
        .sck     = 2,
        .clock   = 1000000,
    };

    const PicoMcp2515::CanConfig canConfig = {
        .baudrate = 1'000'000,
        .oscClock = 16'000'000,
    };

    mcp2515.begin(canConfig, spiConfig);
    mcp2515.beginCanOnly(canConfig, spiConfig);

    mcp2515.endCanOnly();
    mcp2515.end();

    const PicoMcp2515::CanMessage message = {
        .id     = 0x123,
        .length = 8,
        .data   = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
    };
    mcp2515.writeMessage(message);

    const auto readMessage = mcp2515.readMessage();
}

void loop()
{
}
