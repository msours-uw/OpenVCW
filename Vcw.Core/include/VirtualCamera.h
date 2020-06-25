
#pragma once

#include "MatrixUtilities.h"
#include "VirtualProp.h"
#include <execution>

namespace Vcw
{
    struct CameraProperties;

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

        int PhotonsPerPixel;
        double QuantumEfficiency;
        double TemporalDarkNoise;

	private:

		void ComputePerspectiveIterationRange(const cv::Size &PropImageSize, const cv::Point2d &PropScale, const cv::Affine3d &Camera_T_Prop, std::vector<int> &OutIterationsX, std::vector<int> &OutIterationsY) const;

		cv::Mat_<double> CameraMatrix;
	};

    struct CameraProperties
    {
            // Initialize with some default values
            CameraProperties(const cv::Size &Resolution, const cv::Point2d &PrincipalPoint, const cv::Point2d &FocalLength, const std::vector<double> &DistortionCoefficients,
                           const int BitDepth = 12, const int PhotonsPerPixel = 500, const double QuantumEfficiency = 0.69, const double TemporalDarkNoise = 2.29, const double PhotonSensitivity = 5.88, const uint32_t IntensityBaseline = 100)
             : Resolution(Resolution), PrincipalPoint(PrincipalPoint), FocalLength(FocalLength), DistortionCoefficients(DistortionCoefficients),
            BitDepth(BitDepth), PhotonsPerPixel(PhotonsPerPixel), QuantumEfficiency(QuantumEfficiency), TemporalDarkNoise(TemporalDarkNoise), PhotonSensitivity(PhotonSensitivity), IntensityBaseline(IntensityBaseline) {}

            cv::Size Resolution;
            cv::Point2d PrincipalPoint, FocalLength;
            std::vector<double> DistortionCoefficients;

            // Should be available from camera manufacturer
            int BitDepth;
            int PhotonsPerPixel;
            double QuantumEfficiency;
            double TemporalDarkNoise;
            double PhotonSensitivity;
            uint32_t IntensityBaseline;
    };
}
