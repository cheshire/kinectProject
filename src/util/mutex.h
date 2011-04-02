#ifndef MUTEX_H_
#define MUTEX_H_

#include <pthread.h>

class Mutex {
public:
  Mutex();
  void lock();
  void unlock();
private:
  pthread_mutex_t m_mutex;
};

#endif
