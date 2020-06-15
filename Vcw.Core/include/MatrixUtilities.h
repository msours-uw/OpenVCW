
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef unsigned char byte;

namespace Vcw
{
	class MatrixUtilities final
	{
	public:

		static byte BilinearInterpolate(const cv::Mat &Image, const cv::Point2d &Probe);

		static cv::Affine3d CreateTransform(const double Rx, const double Ry, const double Rz, const double Tx, const double Ty, const double Tz);

        static cv::Point2d Normalize(const cv::Point3d &Point);

	private:

		MatrixUtilities() = delete;
	};
}
