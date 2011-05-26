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
  OrientationDetector(int circle_min_size_=500, double min_circularity_=0.55) : 
      min_circularity(min_circularity_),
      circle_min_size(circle_min_size_),
      prev_angle(-1),
      num_jumps(0){};
  
  /**
   * Should be called before anything else.
   */
  void initialize();
  
  /**
   * Get the orientation angle of the turntable.
   * 
   * @param homography Homography matrix for the translation
   * from/to LS plane.
   * @param rgb_image Image of the turntable.
   * @param visualisation Anything we might want to visualise.
   * @param orientation_angle Angle in range [0, 2*PI].
   * 
   * @return Whether the angle was found.
   */
  bool find_orientation_angle(
    const cv::Mat homography,
    const cv::Mat rgb_image,
    cv::Mat visualisation,
    float& orientation_angle
  );
  
  /**
   * Returns the coordinates of the chessboard in an image frame.
   *
   * @param homography the homography matrix.
   *
   * @return point of the chessboard center in an image.
   */
  cv::Point get_chessboard_centre_in_image(const cv::Mat homography);
    
private:    
  /* PARAMETERS */    

  int circle_min_size;
  double min_circularity;
  
  
  /* STATE VARIABLES */
  
  // Previously measured angle
  // in range [0, 2*PI].
  float prev_angle;
  
  // Number of "jumps" from ~-PI/2
  // to ~PI/2 (or vice versa). Twice
  // more then the number of full
  // rotations.
  int num_jumps;    
  
  
  /**
   * Find ellipses in the given image.
   * 
   * @param rgb_image Image to find the contours in.
   * @param visualisation Canvas to draw any visualisations upon.
   * 
   * @return vector of found contours.
   */
  vector<EllipticalContour> find_ellipses(const cv::Mat rgb_image,
                  cv::Mat visualisation);
  
  
  /**
   * Tries to fit an ellipse onto a contour, if an ellipse
   * was found sets it's center and the possible error to struct.
   * 
   * @return Whether the elliptical contour was found.
   */
  bool create_elliptical_contour(EllipticalContour &result);
  
  
  /**
   * Coordinates of two points give rotation angle in range from
   * -PI/2 to + PI/2. We need to extend it to the full rotation range.
   *
   * @param current_angle Currently measured angle in range [-PI/2, PI/2].
   * @param corrected_angle Output param - corrected angle in range [0, 2*PI]
   *
   * @return Whether the angle was found.
   */  
  bool correct_rotation_angle(float current_measured_angle,
    float& corrected_angle
  );    
  
  // Helper functions  
  /**
   * Create and return a vector containing obj.
   */
  template <typename T>
  vector<T> one_element_vector(T obj);
  

};

} // end namespace

#endif /* BACKGROUND_FILTER_H */
