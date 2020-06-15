
#include "VirtualProp.h"

namespace Vcw
{
    VirtualProp::VirtualProp(const cv::Mat &Image, const cv::Size2d &PropSize_m, const cv::Affine3d &Room_T_Prop) : Image(Image), PropSize_m(PropSize_m), Room_T_Prop(Room_T_Prop) { }
}
