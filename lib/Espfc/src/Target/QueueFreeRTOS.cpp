#include "Target.h"

#ifdef ESPFC_FREE_RTOS_QUEUE

#include "Queue.h"

namespace Espfc {

namespace Target {

void Queue::begin()
{
  _q = xQueueCreate(64, sizeof(Event));
}

bool Queue::send(const Event& e)
{
  if (!_q)
  {
    return false;
  }

  return xQueueSend(
             _q,
             &e,
             (TickType_t)0) == pdTRUE;
}

Event Queue::receive()
{
  Event e;
  xQueueReceive(_q, &e, portMAX_DELAY);
  return e;
}

bool Queue::isEmpty() const
{
  return uxQueueMessagesWaiting(_q) == 0;
}

bool Queue::isFull() const
{
  return uxQueueMessagesWaiting(_q) == 64;
}

}

}

#endif
