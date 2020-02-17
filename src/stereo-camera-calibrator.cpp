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

bool StereoCameraCalibrator::process(int height, int weight, cv::Mat &R, cv::Mat &F,  cv::Mat &E, cv::Vec3d &T)
{
  printf("Starting Calibration\n");
  Size size = Size(weight, height);
  std::vector< cv::Mat > rvecs;
  std::vector< cv::Mat > tvecs;
  printf("Starting left camera calibration\n");
  left_calibrator->process(height, weight,rvecs, tvecs,
                                K1 , D1);
  printf("Left camera reprojection error: %f\n", left_calibrator->computeReprojectionErrors(rvecs,tvecs, K1, D1));
  printf("Starting right camera calibration\n");
  right_calibrator->process(height, weight,rvecs, tvecs,
                                K2 , D2);
  printf("Right camera reprojection error: %f\n", right_calibrator->computeReprojectionErrors(rvecs,tvecs, K1, D1));
  int flag = 0;
  flag |= CALIB_FIX_INTRINSIC;
  printf("Starting Stereo calibration\n");
  stereoCalibrate(left_calibrator->get_object_points(), left_calibrator->get_img_points(), right_calibrator->get_img_points(), K1, D1, K2, D2, size, R, T, E, F,flag);
  printf("Done\n");
}

void StereoCameraCalibrator::add_left_sample(cv::Mat& image, bool verbose)
{
  left_calibrator->add_chessboard_sample(image, verbose);
}

void StereoCameraCalibrator::add_right_sample(cv::Mat& image, bool verbose)
{
  right_calibrator->add_chessboard_sample(image, verbose);
}

void StereoCameraCalibrator::rectify(int weight, int height, cv::Mat &R, cv::Vec3d& T, cv::Mat& R1, cv::Mat& R2, cv::Mat& P1, cv::Mat& P2, cv::Mat& Q)
{
  cv::Size size = cv::Size(weight, height);
  stereoRectify(K1, D1, K2, D2, size, R, T, R1, R2, P1, P2, Q);
}