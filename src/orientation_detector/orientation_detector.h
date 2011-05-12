#ifndef ORIENTATION_DETECTOR_H_
#define ORIENTATION_DETECTOR_H_

#include <cv.h>

namespace orientationdetector {
  
struct EllipticalContour {
  double contour_area;
  double error;
  cv::Point2f circle_center;
  vector<cv::Point> contour;
};  

class OrientationDetector{
  
public:
  /**
   * Get the orientation angle of the turntable.
   * 
   * @param homography Homography matrix for the translation
   * from/to LS plane.
   * @param rgb_image Image of the turntable.
   * @param visualisation Anything we might want to visualise.
   * @param orientation_angle Output param to write orientation angle
   * into. It is measured in the range of -\pi to +\pi. We can detect
   * full rotation from an abrupt jump from ~ \pi to ~ -\pi.
   * 
   * @return Whether the angle was found.
   */
  bool find_orientation_angle(
    const cv::Mat homography,
    const cv::Mat rgb_image,
    cv::Mat visualisation,
    float& orientation_angle
  );
  
  void initialize();
  
private:
  
  /**
   * Tries to fit an ellipse onto a contour, if an ellipse
   * was found sets it's center and the possible error to struct.
   * 
   * @return Whether the elliptical contour was found.
   */
  bool create_elliptical_contour(EllipticalContour &result, int min_size = 400);
  
  /**
   * Find ellipses in the given image.
   * 
   * @param rgb_image Image to find the contours in.
   * @param visualisation Canvas to draw any visualisations upon.
   * 
   * @return vector of found contours.
   */
  vector<EllipticalContour> find_circles(const cv::Mat rgb_image,
                  cv::Mat visualisation,
                  int blur_amount = 7,
                  int canny_threshold1 = 125,
                  int canny_threshold2 = 200,
                  int canny_aperture_size = 3);
  
  // Helper functions  
  /**
   * Create and return a vector containing obj.
   */
  template <typename T>
  vector<T> one_element_vector(T obj);
  

};

} // end namespace

#endif /* BACKGROUND_FILTER_H */