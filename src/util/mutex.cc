#include "mutex.h"

Mutex::Mutex() {
  pthread_mutex_init(&m_mutex, NULL);
}

void Mutex::lock() {
  pthread_mutex_lock(&m_mutex);
}

void Mutex::unlock() {
  pthread_mutex_unlock(&m_mutex);
}
