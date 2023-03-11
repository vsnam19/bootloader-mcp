#include "queue.h"

SREC_Record_Type queue[MAX_QUEUE_SIZE];
static uint8_t g_u8Back = 0;
static volatile uint8_t g_u8Front = 0;
static uint8_t g_u8Size = 0;
volatile uint8_t g_u8Index = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
__ramfunc uint8_t queue_insertChar(uint8_t ch)
{
    uint8_t u8IsSuccess = 1u;

    queue[g_u8Back].record[g_u8Index++] = ch;

    if (ch == '\n')
    {
        if (queue_enqueue())
        {
            /** If queue free */
            g_u8Index = 0;
        }
        else
        {
            u8IsSuccess = 0;
        }
    }

    return u8IsSuccess;
}

__ramfunc void queue_reset(void)
{
    g_u8Size = 0;
    g_u8Back = 0;
    g_u8Front = 0;
    g_u8Index = 0;
}

__ramfunc uint8_t queue_enqueue(void)
{
    uint8_t u8IsSuccess = 1u;

    /** Check queue is fully */
    if (g_u8Size < MAX_QUEUE_SIZE)
    {
        g_u8Back += 1u;
        g_u8Size += 1u;

        if (g_u8Back == MAX_QUEUE_SIZE)
        {
            g_u8Back = 0;
        }
    }
    else
    {
        u8IsSuccess = 0;
    }

    return u8IsSuccess;
}

uint8_t queue_dequeue(void)
{
    uint8_t u8IsSuccess = 1u;

    if (g_u8Size == 0)
    {
        u8IsSuccess = 0u;
    }
    else
    {
        g_u8Front += 1u;
        g_u8Size -= 1u;
        if (g_u8Front == MAX_QUEUE_SIZE)
        {
            g_u8Front = 0;
        }
    }

    return u8IsSuccess;
}

uint8_t queue_getFront(void)
{
    return (g_u8Size == 0 ? (MAX_QUEUE_SIZE + 1u) : g_u8Front);
}

uint8_t queue_getBack(void)
{
    return g_u8Back;
}
