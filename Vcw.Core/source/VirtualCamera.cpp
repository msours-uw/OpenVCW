
#include "VirtualCamera.h"

namespace Vcw
{
    VirtualCamera::VirtualCamera(const cv::Size &Resolution, const cv::Point2d &PrincipalPoint, const cv::Point2d &FocalLength, const std::vector<double> &DistortionCoefficients, const cv::Affine3d &Room_T_Camera, const int ID)
        : Resolution(Resolution), PrincipalPoint(PrincipalPoint), FocalLength(FocalLength), DistortionCoefficients(DistortionCoefficients), Room_T_Camera(Room_T_Camera), ID(ID)
	{
		this->CameraMatrix = cv::Mat_<double>(cv::Matx33d(FocalLength.x, 0, PrincipalPoint.x, 0, FocalLength.y, PrincipalPoint.y, 0, 0, 1));
	}
    VirtualCamera::VirtualCamera(const VirtualCamera &virtualCamera)
    {
        this->ID = virtualCamera.ID;
        this->Resolution = virtualCamera.Resolution;
        this->PrincipalPoint = virtualCamera.PrincipalPoint;
        this->FocalLength = virtualCamera.FocalLength;
        this->DistortionCoefficients = virtualCamera.DistortionCoefficients;
        this->Room_T_Camera = virtualCamera.Room_T_Camera;
        this->CameraMatrix = cv::Mat_<double>(cv::Matx33d(FocalLength.x, 0, PrincipalPoint.x, 0, FocalLength.y, PrincipalPoint.y, 0, 0, 1));
    }

	cv::Point2d VirtualCamera::UndistortPoint(const cv::Point2d &Point, bool KeepAsDirection) const
	{
		std::vector<cv::Point2d> undistortedPoints;
		cv::undistortPoints(std::vector<cv::Point2d>({ Point }), undistortedPoints, CameraMatrix, DistortionCoefficients);

		if (KeepAsDirection) return undistortedPoints[0];

		return cv::Point2d(undistortedPoints[0].x * FocalLength.x + PrincipalPoint.x, undistortedPoints[0].y * FocalLength.y + PrincipalPoint.y);
	}

	cv::Point2d VirtualCamera::DistortPoint(const cv::Point3d &Point) const
	{
		const cv::Point2d normalizedPoint(Point.x / Point.z, Point.y / Point.z);

		const double r_2 = pow(normalizedPoint.x, 2.0) + pow(normalizedPoint.y, 2.0);
		const double k1 = DistortionCoefficients[0];
		const double k2 = DistortionCoefficients[1];
		const double p1 = DistortionCoefficients[2];
		const double p2 = DistortionCoefficients[3];
		const double k3 = DistortionCoefficients[4];

		const double x_distorted = normalizedPoint.x * (1.0 + k1 * r_2 + k2 * pow(r_2, 2.0) + k3 * pow(r_2, 3.0)) + 2.0 * p1 * normalizedPoint.x * normalizedPoint.y + p2 * (r_2 + 2 * pow(normalizedPoint.x, 2.0));
		const double y_distorted = normalizedPoint.y * (1.0 + k1 * r_2 + k2 * pow(r_2, 2.0) + k3 * pow(r_2, 3.0)) + 2.0 * p2 * normalizedPoint.x * normalizedPoint.y + p1 * (r_2 + 2 * pow(normalizedPoint.y, 2.0));

		return cv::Point2d(FocalLength.x * x_distorted + PrincipalPoint.x, FocalLength.y * y_distorted + PrincipalPoint.y);
	}

	cv::Mat VirtualCamera::ComputeCameraPerspectiveOfProp(const cv::Mat &PropImage, const cv::Affine3d &Room_T_Prop, const cv::Size2d &PropSize_m) const 
	{
		const cv::Affine3d &Camera_T_Prop = this->Room_T_Camera.inv() * Room_T_Prop;

		const cv::Affine3d::Mat3 Prop_R_Camera = Camera_T_Prop.inv().linear();
		const cv::Vec3d Prop_t_Camera = Camera_T_Prop.inv().translation();

		const cv::Size propImageSize(PropImage.size());

		const cv::Point2d propScale(static_cast<double>(propImageSize.width) / PropSize_m.width, static_cast<double>(propImageSize.height) / PropSize_m.height);
		const cv::Point2d propPrincipalPoint((static_cast<double>(propImageSize.width) - 1.0) / 2.0, (static_cast<double>(propImageSize.height) - 1.0) / 2.0);

		cv::Mat CameraPerspective = cv::Mat::zeros(this->Resolution.height, this->Resolution.width, CV_8UC1);

		std::vector<int> IterationsX, IterationsY;
		ComputePerspectiveIterationRange(propImageSize, propScale, Camera_T_Prop, IterationsX, IterationsY);

		std::for_each(std::execution::par_unseq, IterationsY.begin(), IterationsY.end(), [IterationsX, Prop_R_Camera, Prop_t_Camera, propPrincipalPoint, propScale, &CameraPerspective, propImageSize, PropImage, this](auto&& y)
		{
			std::for_each(std::execution::par_unseq, IterationsX.begin(), IterationsX.end(), [y, Prop_R_Camera, Prop_t_Camera, propPrincipalPoint, propScale, &CameraPerspective, propImageSize, PropImage, this](auto&& x)
			{
				const cv::Point2d &cameraNormalizedDirection = this->UndistortPoint(cv::Point2d(x, y));
				
				const cv::Point3d cameraRay(cameraNormalizedDirection.x, cameraNormalizedDirection.y, 1.0);
				
                const cv::Point2d &propRay = MatrixUtilities::Normalize(Prop_R_Camera * cameraRay);

				const cv::Point2d p_prop(Prop_t_Camera[0] - propRay.x * Prop_t_Camera[2], Prop_t_Camera[1] - propRay.y * Prop_t_Camera[2]);

				const cv::Point2d p_prop_pixel(propPrincipalPoint.x + p_prop.x * propScale.x, propPrincipalPoint.y + p_prop.y * propScale.y);

				if (p_prop_pixel.x <= -0.5 || p_prop_pixel.x >= propImageSize.width - 0.5 || p_prop_pixel.y <= -0.5 || p_prop_pixel.y >= propImageSize.height - 0.5) return;

				CameraPerspective.at<uchar>(y, x) = MatrixUtilities::BilinearInterpolate(PropImage, p_prop_pixel);
			});
		});
		
		return CameraPerspective;
	}

	void VirtualCamera::ComputePerspectiveIterationRange(const cv::Size &PropImageSize, const cv::Point2d &PropScale, const cv::Affine3d &Camera_T_Prop, std::vector<int> &OutIterationsX, std::vector<int> &OutIterationsY) const
	{
		const cv::Point2d propPrincipalPoint((static_cast<double>(PropImageSize.width) - 1.0) / 2.0, (static_cast<double>(PropImageSize.height) - 1.0) / 2.0);

		std::vector<cv::Point2d> cameraPerspectiveLimits(PropImageSize.width * 2 + (PropImageSize.height - 2) * 2);

		for (int k = 0; k < PropImageSize.width; k++)
		{
			const cv::Point3d propPointTop(((double)k - propPrincipalPoint.x) / PropScale.x, (0.0 - propPrincipalPoint.y) / PropScale.y, 0.0);
			const cv::Point3d propPointBottom(((double)k - propPrincipalPoint.x) / PropScale.x, ((PropImageSize.height - 1.0) - propPrincipalPoint.y) / PropScale.y, 0.0);

			const cv::Point3d &cameraPointTop = Camera_T_Prop * propPointTop;
			const cv::Point3d &cameraPointBottom = Camera_T_Prop * propPointBottom;

			const cv::Point2d &cameraPointTop_px = DistortPoint(cameraPointTop);
            const cv::Point2d &cameraPointBottom_px = DistortPoint(cameraPointBottom);

			cameraPerspectiveLimits[k] = cameraPointTop_px;
			cameraPerspectiveLimits[k + PropImageSize.width] = cameraPointBottom_px;
		}

		for (int k = 1; k < PropImageSize.height - 1; k++)
		{
			const cv::Point3d propPointLeft((0.0 - propPrincipalPoint.x) / PropScale.x, ((double)k - propPrincipalPoint.y) / PropScale.y, 0.0);
			const cv::Point3d propPointRight(((PropImageSize.width - 1.0) - propPrincipalPoint.x) / PropScale.x, ((double)k - propPrincipalPoint.y) / PropScale.y, 0.0);

			const cv::Point3d &cameraPointLeft = Camera_T_Prop * propPointLeft;
			const cv::Point3d &cameraPointRight = Camera_T_Prop * propPointRight;

			const cv::Point2d &cameraPointLeft_px = DistortPoint(cameraPointLeft);
			const cv::Point2d &cameraPointRight_px = DistortPoint(cameraPointRight);

			cameraPerspectiveLimits[k + 2 * PropImageSize.width - 1] = cameraPointLeft_px;
			cameraPerspectiveLimits[k + 2 * PropImageSize.width + PropImageSize.height - 3] = cameraPointRight_px;
		}

		decltype(cameraPerspectiveLimits)::iterator minX, maxX, minY, maxY;
		std::tie(minX, maxX) = std::minmax_element(begin(cameraPerspectiveLimits), std::end(cameraPerspectiveLimits), [](cv::Point2d const& p0, cv::Point2d const& p1) {return p0.x < p1.x; });
		std::tie(minY, maxY) = std::minmax_element(begin(cameraPerspectiveLimits), std::end(cameraPerspectiveLimits), [](cv::Point2d const& p0, cv::Point2d const& p1) {return p0.y < p1.y; });

		const cv::Range RangeX(cv::max(0, (int)floor(minX->x)), cv::min(Resolution.width - 1, (int)ceil(maxX->x)));
		const cv::Range RangeY(cv::max(0, (int)floor(minY->y)), cv::min(Resolution.height - 1, (int)ceil(maxY->y)));

		OutIterationsX = std::vector<int>(RangeX.size() + 1);
		OutIterationsY = std::vector<int>(RangeY.size() + 1);

		std::iota(std::begin(OutIterationsX), std::end(OutIterationsX), RangeX.start);
		std::iota(std::begin(OutIterationsY), std::end(OutIterationsY), RangeY.start);
	}
}
