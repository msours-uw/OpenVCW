
#include "MatrixUtilities.h"

namespace Vcw
{
	byte MatrixUtilities::BilinearInterpolate(const cv::Mat &Image, const cv::Point2d &Probe)
	{
		int left = static_cast<int>(std::floor(Probe.x));
		int top = static_cast<int>(std::floor(Probe.y));

		const double w_right = Probe.x - left;
		const double w_bottom = Probe.y - top;
		const int imageWidth = Image.size().width;
		const int imageHeight = Image.size().height;
		const int right = std::min(imageWidth - 1, left + 1);
		const int bottom = std::min(imageHeight - 1, top + 1);

		left = std::max(0, left);
		top = std::max(0, top);

		const double v_tl = static_cast<double>(Image.at<byte>(top, left));
		const double v_tr = static_cast<double>(Image.at<byte>(top, right));
		const double v_bl = static_cast<double>(Image.at<byte>(bottom, left));
		const double v_br = static_cast<double>(Image.at<byte>(bottom, right));

		return static_cast<byte>(std::round(v_tl * (1.0 - w_bottom) * (1.0 - w_right) + v_tr * (1.0 - w_bottom) * w_right + v_bl * w_bottom * (1.0 - w_right) + v_br * w_bottom * w_right));
	}

    cv::Affine3d MatrixUtilities::CreateAffineTransform(const double Rx, const double Ry, const double Rz, const double Tx, const double Ty, const double Tz)
	{
		cv::Affine3d T = cv::Affine3d::Identity();
		T.matrix(0, 0) = cos(Ry) * cos(Rz);
		T.matrix(0, 1) = cos(Ry) * sin(Rz);
		T.matrix(0, 2) = -sin(Ry);
		T.matrix(0, 3) = Tx;

		T.matrix(1, 0) = (sin(Rx) * sin(Ry) * cos(Rz) - cos(Rx) * sin(Rz));
		T.matrix(1, 1) = (sin(Rx) * sin(Ry) * sin(Rz) + cos(Rx) * cos(Rz));
		T.matrix(1, 2) = sin(Rx) * cos(Ry);
		T.matrix(1, 3) = Ty;

		T.matrix(2, 0) = (cos(Rx) * sin(Ry) * cos(Rz) + sin(Rx) * sin(Rz));
		T.matrix(2, 1) = (cos(Rx) * sin(Ry) * sin(Rz) - sin(Rx) * cos(Rz));
		T.matrix(2, 2) = cos(Rx) * cos(Ry);
		T.matrix(2, 3) = Tz;

		return T;
	}

    cv::Point2d MatrixUtilities::Normalize(const cv::Point3d &Point)
    {
        return cv::Point2d(Point.x/Point.z, Point.y/Point.z);
    }
}
