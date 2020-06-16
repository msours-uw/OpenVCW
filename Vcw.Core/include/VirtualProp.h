
#pragma  once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace Vcw
{
    class VirtualProp
    {
    public:
        VirtualProp(const cv::Mat &Image, const cv::Size2d &PropSize_m, const cv::Affine3d &Room_T_Prop = cv::Affine3d::Identity());

        cv::Mat Image;
        cv::Affine3d Room_T_Prop;
        cv::Size2d PropSize_m;

    };
}
