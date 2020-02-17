#include "stereo-camera-calibrator.hpp"
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;
using namespace stereo_pi::mono;
using namespace stereo_pi::stereo;


StereoCameraCalibrator::StereoCameraCalibrator(stereo_pi::mono::MonoCameraCalibrator& left_calibrator,
                                       stereo_pi::mono::MonoCameraCalibrator& right_calibrator)
{
  this->left_calibrator = std::shared_ptr<stereo_pi::mono::MonoCameraCalibrator>(&left_calibrator);
  this->right_calibrator = std::shared_ptr<stereo_pi::mono::MonoCameraCalibrator>(&right_calibrator);
}

StereoCameraCalibrator::StereoCameraCalibrator(int board_width, int board_height, float square_size)
{
  this->left_calibrator = std::make_shared<stereo_pi::mono::MonoCameraCalibrator>(board_width, board_height, square_size);
  this->right_calibrator = std::make_shared<stereo_pi::mono::MonoCameraCalibrator>(board_width, board_height, square_size);
}

bool StereoCameraCalibrator::process(int height, int weight, Mat& R, Mat& F, Mat& E, Vec3d& T)
{
  printf("Starting Calibration\n");
  Size size = Size(weight, height);
  std::vector< cv::Mat > rvecs;
  std::vector< cv::Mat > tvecs;
  Mat K1, K2;
  Mat D1, D2;
  left_calibrator->process(height, weight,rvecs, tvecs,
                                K1 , D1);
  printf("Left camera reprojection error: %f\n", left_calibrator->computeReprojectionErrors(rvecs,tvecs, K1, D1));
  right_calibrator->process(height, weight,rvecs, tvecs,
                                K2 , D2);
  printf("Right camera reprojection error: %f\n", right_calibrator->computeReprojectionErrors(rvecs,tvecs, K1, D1));
  int flag = 0;
  flag |= CALIB_FIX_INTRINSIC;

  stereoCalibrate(left_calibrator->get_object_points(), left_calibrator->get_img_points(), right_calibrator->get_img_points(), K1, D1, K2, D2, size, R, T, E, F,flag);

}