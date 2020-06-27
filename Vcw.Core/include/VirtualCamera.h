
#pragma once

#include "MatrixUtilities.h"
#include "VirtualProp.h"
#include <execution>

namespace Vcw
{
    struct CameraProperties
    {
            CameraProperties(){}

            // Initialize with some default values
            CameraProperties(const cv::Size &Resolution, const cv::Point2d &PrincipalPoint, const cv::Point2d &FocalLength, const std::vector<double> &DistortionCoefficients,
                             const int BitDepth = 12, const int PhotonsPerPixel = 300, const double QuantumEfficiency = 0.55, const double TemporalDarkNoise = 6.09, const double PhotonSensitivity = 5.88, const uint32_t IntensityBaseline = 43)
                : Resolution(Resolution), PrincipalPoint(PrincipalPoint), FocalLength(FocalLength), DistortionCoefficients(DistortionCoefficients),
                  BitDepth(BitDepth), PhotonsPerPixel(PhotonsPerPixel), QuantumEfficiency(QuantumEfficiency), TemporalDarkNoise(TemporalDarkNoise), PhotonSensitivity(PhotonSensitivity), IntensityBaseline(IntensityBaseline) {}

            cv::Size Resolution;
            cv::Point2d PrincipalPoint, FocalLength;
            std::vector<double> DistortionCoefficients;

            // Should be available from camera manufacturer
            int BitDepth;
            int PhotonsPerPixel;
            float QuantumEfficiency;
            float TemporalDarkNoise;
            float PhotonSensitivity;
            uint32_t IntensityBaseline;
    };

    class VirtualCamera
    {
        public:

            VirtualCamera(){}

            VirtualCamera(const CameraProperties &cameraProperties, const cv::Affine3d &Room_T_Camera = cv::Affine3d::Identity(), const int ID = -1);

            VirtualCamera(const VirtualCamera &virtualCamera);

            cv::Point2d UndistortPoint(const cv::Point2d &Point, bool KeepAsDirection = true) const;
            cv::Point2d DistortPoint(const cv::Point3d &Point) const;

            cv::Mat ComputeCameraPerspectiveOfProp(const VirtualProp &virtualProp, const bool AddCameraNoise = true) const;
            cv::Mat SimulateCameraNoise(const cv::Mat &Image) const;

            cv::Affine3d Room_T_Camera;

            int ID;

            int BitDepth() const;
            int PhotonsPerPixel() const;
            float QuantumEfficiency() const;
            float TemporalDarkNoise() const;
            float PhotonSensitivity() const;
            uint32_t IntensityBaseline() const;

            cv::Size Resolution() const;
            cv::Point2d PrincipalPoint() const;
            cv::Point2d FocalLength() const;
            std::vector<double> DistortionCoefficients() const;

        private:

            cv::Mat_<double> CameraMatrix;
            CameraProperties cameraProperties;

            void ComputePerspectiveIterationRange(const cv::Size &PropImageSize, const cv::Point2d &PropScale, const cv::Affine3d &Camera_T_Prop, std::vector<int> &OutIterationsX, std::vector<int> &OutIterationsY) const;
    };
}
