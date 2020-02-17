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

                bool process(int height, int weight, Mat& R, Mat& F, Mat& E, Vec3d& T);
            public:
                std::shared_ptr<stereo_pi::mono::MonoCameraCalibrator>  left_calibrator;
                std::shared_ptr<stereo_pi::mono::MonoCameraCalibrator>  right_calibrator;
        };
    }
}