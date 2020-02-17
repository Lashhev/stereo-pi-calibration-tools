#include <opencv2/highgui/highgui.hpp>
#include "mono-camera-calibrator.hpp"
#include <memory>

namespace stereo_pi
{
    namespace stereo
    {
        class StereoCameraCalibrator
        {
            public:
                StereoCameraCalibrator(stereo_pi::mono::MonoCameraCalibrator& left_calibrator,
                                       stereo_pi::mono::MonoCameraCalibrator& right_calibrator);

                StereoCameraCalibrator(int board_width, int board_height, float square_size);

                void add_left_sample(cv::Mat& image, bool verbose=false);
                
                void add_right_sample(cv::Mat& image, bool verbose=false);

                void rectify(int weight, int height, cv::Mat &R, cv::Vec3d& T, cv::Mat& R1, cv::Mat& R2, cv::Mat& P1, cv::Mat& P2, cv::Mat& Q);
                
                bool process(int height, int weight, cv::Mat &R, cv::Mat &F,  cv::Mat &E, cv::Vec3d &T);
            public:
                std::shared_ptr<stereo_pi::mono::MonoCameraCalibrator>  left_calibrator;
                std::shared_ptr<stereo_pi::mono::MonoCameraCalibrator>  right_calibrator;
                cv::Mat K1, K2, D1, D2;
        };
    }
}