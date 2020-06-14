
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "VirtualCamera.h"

int Test()
{
	const cv::Size cameraResolution(3006, 4104);
	const cv::Point2d cameraPrincipalPoint(1487.8077004549134, 2060.6656996509428);
	const cv::Point2d cameraFocalLength(3570.8487771614123, 3570.5813860875437);
	const std::vector<double> cameraDistortionCoefficients(std::vector<double>({ -0.3184124974828117 , 0.1655891487212314, -0.00017802041075148632, 0.00017411575646864198, -0.056445932029310308 }));

    Vcw::VirtualCamera virtualCamera(cameraResolution, cameraPrincipalPoint, cameraFocalLength, cameraDistortionCoefficients);

	const cv::Mat &PropImage = cv::imread("P.png", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

	const double propDepth = 300.0;
	const double propWidth = 144.0;
	const double propHeight = 176.0;

	const cv::Size2d propSize(propWidth, propHeight);

    const cv::Affine3d &Room_T_Prop = Vcw::MatrixUtilities::CreateTransform(3 * 0.0872664625997165, 2 * 0.0872664625997165, 2 * 0.0872664625997165, 5.8, -8.4, propDepth);

	const cv::Mat &cameraPerspective = virtualCamera.ComputeCameraPerspectiveOfProp(PropImage, Room_T_Prop, propSize);

	cv::imwrite("Test.png", cameraPerspective);

	return 0;
}

int main()
{
	Test();

	return 0;
}
