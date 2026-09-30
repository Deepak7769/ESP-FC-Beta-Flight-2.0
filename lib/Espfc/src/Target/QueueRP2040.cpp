#include "Target.h"

#if defined(ARCH_RP2040)

#include "Queue.h"

namespace Espfc {

namespace Target {

void Queue::begin()
{
  queue_init(&_q, sizeof(Event), 128);
}

bool Queue::send(const Event& e)
{
  if (isFull())
  {
    return false;
  }

  queue_add_blocking(&_q, &e);

  return true;
}

Event Queue::receive()
{
  Event e;
  queue_remove_blocking(&_q, &e);
  return e;
}

bool Queue::isEmpty() const
{
  return queue_is_empty(const_cast<TargetQueueHandle*>(&_q));
}

bool Queue::isFull() const
{
  return queue_is_full(const_cast<TargetQueueHandle*>(&_q));
}

}

}

#endif
