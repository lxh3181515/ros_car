#include "queue.h"



queue::queue()
{
    capacity = MAXSIZE;
    size = front = rear = 0;
}

bool queue::is_empty()
{
    return size == 0;
}

bool queue::is_full()
{
    return size == capacity;
}

bool queue::enqueue(uint8_t element)
{
    if (is_full())
    {
        ROS_ERROR("queue is full");
        return false;
    }
    data[rear] = element;
    size++;
    rear = (rear + 1) % capacity;
    return true;
}

bool queue::dequeue()
{
    if (is_empty())
    {
        ROS_ERROR("queue is empty");
        return false;
    }
    size--;
    front = (front + 1) % capacity;
    return true;
}

uint8_t queue::get_front()
{
    return data[front];
}

uint8_t queue::get_front_second()
{
    return data[(front + 1) % capacity];
}

int queue::get_queue_length()
{
    return size;
}

void queue::show_queue()
{
    for (int i = 0; i < capacity; i++)
    {
        printf("%d ", data[i]);
    }
    printf("\n");
}

