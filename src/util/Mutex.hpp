#include <pthread.h>

class Mutex {
public:
  Mutex();
  void lock();
  void unlock();
private:
  pthread_mutex_t m_mutex;
};
