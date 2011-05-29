#ifndef MATRIXOPERATION_H_
#define MATRIXOPERATION_H_
#include <cv.h>

template <class T>
class MatrixOperation {
  protected:
    virtual T apply(T value, int r, int c) = 0;
    virtual cv::Mat generate(cv::Mat input);
  public:
    cv::Mat perform(cv::Mat input);
};

cv::Mat MatrixOperation::generate(cv::Mat input) {
  return cv::Mat(input.size(), input.type());
}

cv::Mat MatrixOperation::perform(cv::Mat input) {
  cv::Mat result = generate(input);
  for (int r = 0; r < input.rows; r++) {
    for (int c = 0; c < input.cols; c++) {
      result.at<T>(r, c) = apply(input.at<T>(r, c), r, c);
    }
  }
  return result;
}

#endif
