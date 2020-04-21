/**
 *  CANView - view messages on a CAN bus
 *
 *  This Arduino sketch reads messages on a CAN bus and prints them in a readable
 *  form on a serial console.
 *
 *  Author:      Gnaddl
 *  Date:        16-APR-2020
 *  Last change: 21-APR-2020
 */

#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"

// Configuration
const int SPI_CS_PIN = 9;               // CS pin: D10 for versions v0.9b and v1.0, D9 for versions after v1.1
const int CAN_INT_PIN = 2;              // interrupt pin
const int CAN_CLOCK = MCP_8MHz;         // crystal clock frequency of the CAN bus interface: 8 or 16 MHz
const int CAN_SPEED = CAN_250KBPS;      // fixed CAN bus speed

const int QUEUE_SIZE = 64;              // max. number of elements in a queue


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
 * Simple implementation of a Queue class for CAN messages; no overflow detection.
 */
class CanMsgQueue
{
public:
    CanMsgQueue();                      // Constructor
    bool empty();                       // check whether the queue is empty
    int push_back(CanMsg *p);           // add a CAN message to the queue
    bool pop_front(CanMsg *p);          // get a CAN message from the queue, if available

private:
    volatile int readIdx;               // index of the next CAN message to read
    volatile int writeIdx;              // index of the next CAN message to write
    CanMsg buffer[QUEUE_SIZE];          // buffer for a number of CAN messages
};


MCP_CAN CAN(SPI_CS_PIN);                // Create a CAN port instance
CanMsgQueue queue;                      // queue for the CAN bus messages


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
CanMsgQueue::CanMsgQueue() :
    readIdx(0),
    writeIdx(0)
{
}


/*
 * Check whether the queue is empty
 */
bool CanMsgQueue::empty()
{
    return readIdx == writeIdx;
}


/* 
 * Add a CAN message to the queue
 */
int CanMsgQueue::push_back(CanMsg *p)
{
    buffer[writeIdx++] = *p;            // write CAN message to next position in the queue
    if (writeIdx >= QUEUE_SIZE)
        writeIdx = 0;
    return 0;
}


/* 
 * Get a CAN message from the queue, if available
 */
bool CanMsgQueue::pop_front(CanMsg *p)
{
    bool rc = !empty();
    if (rc)
    {
        *p = buffer[readIdx++];         // copy the CAN message
        if (readIdx >= QUEUE_SIZE)
            readIdx = 0;
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

        case 't':
        {
            // Send CANopen message "Stop Remote Node" to all slave devices.
            const byte data[] = { 0x02, 0x00 };
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

        queue.push_back(&msg);
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
    int c;

    while (queue.pop_front(&msg))
    {
        // CAN bus message received, decode and print it
        msg.print();
    }

    while ((c = Serial.read()) != -1)
    {
        // Character from serial console received
        userInput(c);
    }
}
