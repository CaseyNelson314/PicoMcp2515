# PicoMcp2515 [![CI](https://github.com/CaseyNelson314/PicoMcp2515/actions/workflows/CI.yml/badge.svg)](https://github.com/CaseyNelson314/PicoMcp2515/actions/workflows/CI.yml)

> [!CAUTION]
> Currently under development in the develop branch.

MCP2515 (CAN Controller) Library for Raspberry Pi Pico

![ojigi_animal_inu](https://github.com/CaseyNelson314/PicoMCP2515/assets/91818705/99f06205-bf15-4401-8a79-5c7d85ddc217)

## Target code

### Transmitter

```cpp
#include <PicoMcp2515.hpp>

static PicoMcp2515 mcp {
    .csPin = 0,
    ...
    .baudrate = 1'000'000,
    .oscClock = 16'000'000
};

void setup()
{
    mcp.begin();
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

### Receiver


```cpp
#include <PicoMcp2515.hpp>

static PicoMcp2515 mcp {
    .csPin = 0,
    .intPin = 2,
    ...
    .baudrate = 1'000'000,
    .oscClock = 16'000'000
};

void onReceive(const PicoMcp2515::CanMessage& message, void*)
{
    for (const auto& e : message.data)
    {
        Serial.print(e);
        Serial.print(", ");
    }
    Serial.println();
}

void setup()
{
    mcp.begin();
    mcp.onReceive(0x0101, onReceive, nullptr);
}

void loop()
{
}
```
