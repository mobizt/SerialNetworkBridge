#ifndef INTERRUPT_STREAM_H
#define INTERRUPT_STREAM_H

#include <Arduino.h>
#include <NeoHWSerial.h>

// Define buffer size (Power of 2 is best for performance)
#define ISR_BUF_SIZE 1024

class InterruptStream : public Stream {
private:
    volatile uint8_t _buffer[ISR_BUF_SIZE];
    volatile uint16_t _head;
    volatile uint16_t _tail;
    NeoHWSerial *_txPort; // Pointer to the real hardware port for sending data OUT

public:
    // Constructor: Pass the NeoSerial port you want to write to (e.g., NeoSerial1)
    InterruptStream(NeoHWSerial &port) : _head(0), _tail(0), _txPort(&port) {}

    // -----------------------------------------------------------------
    // ISR METHOD: Call this inside your NeoHWSerial attachInterrupt function
    // -----------------------------------------------------------------
    inline void push(uint8_t c) {
        uint16_t next = (_head + 1) % ISR_BUF_SIZE;
        
        // Only write if buffer is not full
        if (next != _tail) {
            _buffer[_head] = c;
            _head = next;
        }
        // Else: Buffer overflow - byte dropped (increase ISR_BUF_SIZE if this happens)
    }

    // -----------------------------------------------------------------
    // STREAM INTERFACE (Used by SerialNetworkBridge)
    // -----------------------------------------------------------------
    virtual int available() {
        return (ISR_BUF_SIZE + _head - _tail) % ISR_BUF_SIZE;
    }

    virtual int read() {
        if (_head == _tail) return -1;
        uint8_t c = _buffer[_tail];
        _tail = (_tail + 1) % ISR_BUF_SIZE;
        return c;
    }

    virtual int peek() {
        if (_head == _tail) return -1;
        return _buffer[_tail];
    }

    // Pass output data directly to the hardware UART
    virtual size_t write(uint8_t c) {
        return _txPort->write(c);
    }
    
    // Support for writing buffers
    virtual size_t write(const uint8_t *buffer, size_t size) {
        return _txPort->write(buffer, size);
    }

    virtual void flush() {
        _txPort->flush();
    }
};

#endif