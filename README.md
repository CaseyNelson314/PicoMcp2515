# PicoMCP2515

MCP2515 Library for Raspberry Pi Pico

## Usage

```cpp
#include <PicoMcp2515.hpp>

static PicoMcp2515 mcp;

void setup()
{
    // Construct setting structure

    PicoMcp2515::CanConfig canConfig {
        .baudrate = 1'000'000,
        .oscClock = 16'000'000
    };

    PicoMcp2515::SpiConfig spiConfig {};


    // Start CAN communication
    mcp.begin(canConfig, spiConfig);


    // Set receive interrupt handler
    const uint8_t interruptPin = 20;
    attachInterruptParam(interruptPin, [](void*) {

        const auto message = mcp.readMessage();

        Serial.println(message.id);

        for (const auto& element : message.data)
        {
            Serial.print(element);
            Serial.print(", ");
        }
        Serial.println();

    }, LOW, nullptr);

}

void loop()
{

    PicoMcp2515::CanMessage message {
        .id     = 0x0101,
        .length = 8,
        .data   = { 0, 1, 2, 3, 4, 5, 6, 7 },
    };

    mcp.writeMessage(message);

    delay(10);

}
```

## API
