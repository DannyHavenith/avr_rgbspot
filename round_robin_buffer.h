#ifndef ROUND_ROBIN_BUFFER_H
#define ROUND_ROBIN_BUFFER_H

#include "stdint.h"

template<uint8_t buffer_size = 64, typename datatype = uint8_t>
struct round_robin_buffer
{

    typedef datatype value_type;

    bool    is_full;
    uint8_t tentative_index;
    uint8_t write_index;
    uint8_t read_index;
    value_type buffer[buffer_size];

    /// tentative writes, write to the buffer, but don't
    /// make the data available to readers yet.
    bool write_tentative( uint8_t value) volatile
    {
        if (
                tentative_index != read_index 
            ||  (tentative_index == write_index && !is_full)
            )
        {
            buffer[tentative_index] = value;
            tentative_index = (tentative_index + 1) % buffer_size;

            return true;
        }
        else
        {
            return false;
        }
    }

    /// remove all tentative writes
    void reset_tentative() volatile
    {
        tentative_index = write_index;
    }


    /// commit all tentative writes, making them
    /// available to readers.
    void commit() volatile
    {
        write_index = tentative_index;
        is_full = (write_index == read_index);
    }

    /// write a value to the queue. 
    /// Don't mix write calls with write_tentative.
    bool write( value_type value) volatile
    {

        if (!is_full)
        {
            buffer[write_index] = value;
            write_index = (write_index + 1) % buffer_size;
            if (write_index == read_index)
            {
                is_full = true;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// read a value from the queue
    bool read(  value_type *value) volatile
    {
        if (read_index == write_index && !is_full)
        {
            return false;
        }
        else
        {
            *value = buffer[read_index];
            read_index = (read_index + 1) % buffer_size;
            is_full = false;
            return true;
        }
    }

    /// wait for a value and read it.
    value_type read_w() volatile
    {
        value_type value;
        while (!read(&value)) /*nop*/ ;
        return value;
    }
};


#endif //ROUND_ROBIN_BUFFER_H
