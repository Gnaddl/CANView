/**
 *  CANView - view messages on a CAN bus
 *
 *  This Arduino sketch reads messages on a CAN bus and prints them in a readable
 *  form on a serial console.
 *
 *  Author:      Gnaddl
 *  Date:        16-APR-2020
 *  Last change: 16-APR-2020
 */

#include "Arduino.h"
#include <SPI.h>
#include "mcp_can.h"

// the CS pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
const int CAN_SPEED = CAN_250KBPS;      // fixed CAN bus speed
const int CAN_CLOCK = MCP_8MHz;         // Xtal clock of the CAN bus interface, either 8 or 16 MHz

const int FIFO_BUFFER_SIZE = 64;        // must be a power of 2 for faster calculation
const int FIFO_BUFFER_MASK = FIFO_BUFFER_SIZE - 1;


/*
 * CAN message structure
 */
struct CanMsg
{
    uint32_t timestamp;                 // number of milliseconds since last reset
    uint16_t id : 12;                   // CAN ID
    uint16_t len : 4;                   // length of the data field
    byte data[8];                       // payload data

    CanMsg(){};                         // Constructors
    CanMsg(uint16_t id, byte len, const byte *data);
    void send(MCP_CAN &port);           // send a message via CAN bus
    void print();                       // decode and print the message in a readable form
};


/* 
 * Simple FIFO class for CAN messages; no overflow detection.
 */
class CanMsgFifo
{
public:
    CanMsgFifo();                       // Constructor
    int put(CanMsg *p);                 // add a CAN message to the FIFO
    bool get(CanMsg *p);                // get a CAN message from the FIFO, if available

private:
    volatile int readIdx;               // index of the next CAN message to read
    volatile int writeIdx;              // index of the next CAN message to write
    CanMsg buffer[FIFO_BUFFER_SIZE];    // buffer for a number of CAN messages
};


MCP_CAN CAN(SPI_CS_PIN);                // Create a CAN port instance
CanMsgFifo fifo;                        // FIFO for the CAN bus messages


/*
 * Constructor with initialization data
 */
CanMsg::CanMsg(uint16_t id, byte len, const byte *data) :
    id(id),
    len(len)
{
    memcpy(this->data, data, len);
}


/*
 * Send a CAN message via a CAN port
 */
void CanMsg::send(MCP_CAN &port)
{
    timestamp = millis();
    port.sendMsgBuf(id, 0, len, data);
    this->print();
}


/*
 * Decode and print a CAN message
 */
void CanMsg::print()
{
    char buffer[80];
    char *p = buffer;

    // print time stamp
    const uint16_t ms = (uint16_t) (timestamp % 1000UL);
    const uint32_t sec = timestamp / 1000UL;
    p += sprintf(p, "%5lu.%03u   ", sec, ms);

    // print CAN ID
    p += sprintf(p, "%03X:", id);

    // print the CAN data
    for (int i = 0; i < len; i++)
    {
        p += sprintf(p, " %02X", data[i]);
    }

    Serial.println(buffer);
}


/* 
 * Constructor
 */
CanMsgFifo::CanMsgFifo() :
    readIdx(0),
    writeIdx(0)
{
}


/* 
 * Add a CAN message to the FIFO
 */
int CanMsgFifo::put(CanMsg *p)
{
    buffer[writeIdx++] = *p;            // write CAN message to next free position in the FIFO
    writeIdx &= FIFO_BUFFER_MASK;       // increment write index and handle wrap-around
    return 0;
}


/* 
 * Get a CAN message from the FIFO, if available
 */
bool CanMsgFifo::get(CanMsg *p)
{
    bool rc = (readIdx != writeIdx);    // is a CAN message available in the FIFO?
    if (rc)
    {
        *p = buffer[readIdx++];         // copy the CAN message
        readIdx &= FIFO_BUFFER_MASK;    // increment read index and handle wrap-around
    }
    return rc;
}


/*
 * Handle user input from serial console. Currently sends only a predefined CAN message.
 * Will be improved later to execute more flexible commands.
 */
void userInput(int c)
{
    switch (c)
    {
        case 's':
        {
            // Send CANopen message "Start Remote Node" to all slave devices.
            const byte data[] = { 0x01, 0x00 };
            CanMsg msg(0x00, 2, data);
            msg.send(CAN);
            break;
        }

        default:
            break;
    }
}


void MCP2515_isr()
{
    CanMsg msg;
    unsigned long id;
    byte len;

    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
        msg.timestamp = millis();
        CAN.readMsgBufID(&id, &len, msg.data);
        msg.id = id;
        msg.len = len;

        fifo.put(&msg);
    }
}


void setup()
{
    Serial.begin(115200);

    // init can bus: baudrate = 250kBit/s @ 8 MHz
    while (CAN_OK != CAN.begin(CAN_SPEED, CAN_CLOCK))
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(500);
    }

    Serial.println("CAN BUS Shield init ok!");

    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_isr, FALLING); // start interrupt
}


void loop()
{
    CanMsg msg;

    while (fifo.get(&msg))
    {
        // CAN bus message received, decode and print it
        msg.print();
    }

    int c = Serial.read();
    if (c != -1)
    {
        // Character from serial console received
        userInput(c);
    }
}
