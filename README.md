# CANView

This Arduino sketch can be used to display the message traffic on a CAN bus. Please note that the
software has some restrictions, because I wrote it only for my personal use. However, you can use
it freely and modify it as you want.


## 1. Hardware

The sketch can run on different hardware combinations. Here are a two examples which I have tested:

- Arduino Uno with a CAN Bus shield
- Arduino Uno with an MCP2515 CAN Bus module

Depending on the CAN interface hardware it might be necessary to adapt the configuration:

|                                 |       Clock       | Chip Select (CS) pin | Interrupt pin |
| :------------------------------ | :---------------: | :------------------: | :-----------: |
| CAN Bus Shield v0.9 and v1.0    |      16 MHz       |          9           |       2       |
| CAN Bus Shield v1.1 and greater |      16 MHz       |         10           |       2       |
| MCP2515 CAN Bus Module          |       8 MHz       |         10           |       2       |

Modify the definitions of the constants at the beginning of the Arduino source code accordingly:

```C++
const int SPI_CS_PIN = 9;               // CS pin: D10 for versions v0.9b and v1.0, D9 for versions after v1.1
const int CAN_INT_PIN = 2;              // interrupt pin
const int CAN_CLOCK = MCP_8MHz;         // crystal clock frequency of the CAN bus interface: 8 or 16 MHz
const int CAN_SPEED = CAN_250KBPS;      // fixed CAN bus speed
```


## 2. Software

### 2.1 Arduino Sketch

The sketch is implemented in the source file `CANView.ino`. Two C++ classes are defined here: `CanMsg`
and `CanMsgQueue`. CAN messages are received by interrupt. The interrupt function reads the message
from the MCP2515 and pushes it into the CAN message queue. In the `loop()` function messages are
taken from the queue and printed over the serial line in a readable form.

### 2.2 Used Libraries

The Arduino sketch uses the following library:

- [CAN-BUS Shield Library](https://github.com/Seeed-Studio/CAN_BUS_Shield) version 1.0.0 by Seeed Studio

Install the library in the Arduino IDE by using the Library Manager. Alternatively download it from Github
and install it manually in the library folder of the Arduino IDE.
