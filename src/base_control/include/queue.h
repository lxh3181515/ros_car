#ifndef __QUEUE_H
#define __QUEUE_H

#include "ros/ros.h"
#define MAXSIZE 256 * 16


class queue
{
private:
    int capacity;
    int size, front, rear;
    uint8_t data[MAXSIZE];
public:
    queue();
    bool is_empty();
    bool is_full();
    bool enqueue(uint8_t element);
    bool dequeue();
    uint8_t get_front();
    uint8_t get_front_second();
    int get_queue_length();
    void show_queue();
};

#endif
