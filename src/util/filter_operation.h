#ifndef FILTEROPERATION_H_
#define FILTEROPERATION_H_

#include "util/matrix_operation.h"

template <class T>
class FilterOperation : public MatrixOperation<T> {
private:
  T min;
  T max;
  T masked;
protected:
  virtual T apply(T value, int r, int c) {
    if (value > max || value < min) {
      return masked;
    } else {
      return value;
    }
  }
public:
  FilterOperation(T minimum, T maximum, T masked) : min(minimum), max(maximum), masked(masked) {}
};

#endif
