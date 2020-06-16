
#pragma once

#include "MatrixUtilities.h"
#include "VirtualProp.h"
#include <execution>

namespace Vcw
{
	class VirtualCamera
	{
	public:

        VirtualCamera(){}
        VirtualCamera(const cv::Size &Resolution, const cv::Point2d &PrincipalPoint, const cv::Point2d &FocalLength, const std::vector<double> &DistortionCoefficients, const cv::Affine3d &Room_T_Camera = cv::Affine3d::Identity(), const int ID = -1);

        VirtualCamera(const VirtualCamera &virtualCamera);

		cv::Point2d UndistortPoint(const cv::Point2d &Point, bool KeepAsDirection = true) const;
		cv::Point2d DistortPoint(const cv::Point3d &Point) const;

        cv::Mat ComputeCameraPerspectiveOfProp(const VirtualProp &virtualProp) const;

        cv::Size Resolution;
        cv::Point2d PrincipalPoint, FocalLength;
        std::vector<double> DistortionCoefficients;

        cv::Affine3d Room_T_Camera;

        int ID;

	private:

		void ComputePerspectiveIterationRange(const cv::Size &PropImageSize, const cv::Point2d &PropScale, const cv::Affine3d &Camera_T_Prop, std::vector<int> &OutIterationsX, std::vector<int> &OutIterationsY) const;

		cv::Mat_<double> CameraMatrix;
	};
}
