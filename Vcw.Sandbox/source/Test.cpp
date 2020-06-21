
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "VirtualWorld.h"

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

    const cv::Affine3d &Room_T_Prop = Vcw::MatrixUtilities::CreateTransform(1.5 * 0.0872664625997165, 2 * 0.0872664625997165, 2 * 0.0872664625997165, 5.8, -8.4, propDepth);

    const cv::Mat &cameraPerspective = virtualCamera.ComputeCameraPerspectiveOfProp(Vcw::VirtualProp(PropImage, propSize, Room_T_Prop));

	cv::imwrite("Test.png", cameraPerspective);

	return 0;
}

int Test0()
{
    /*
    const cv::Size cameraResolution(1016, 1016);
    const cv::Point2d cameraPrincipalPoint(509.39244928806124, 492.06837448912717);
    const cv::Point2d cameraFocalLength(588.11751976594621, 588.15082187788494);
    const std::vector<double> cameraDistortionCoefficients(std::vector<double>({ 0.05135140864279755, -0.090126734678614179, 0.0010457541065092286, 0.00061563690270563249, -0.016839003638018732 }));
    */
    const cv::Size cameraResolution(3006, 4104);
    const cv::Point2d cameraPrincipalPoint(1487.8077004549134, 2060.6656996509428);
    const cv::Point2d cameraFocalLength(3570.8487771614123, 3570.5813860875437);
    const std::vector<double> cameraDistortionCoefficients(std::vector<double>({ -0.3184124974828117 , 0.1655891487212314, -0.00017802041075148632, 0.00017411575646864198, -0.056445932029310308 }));

    Vcw::VirtualWorld virtualWorld = Vcw::VirtualWorld();
    virtualWorld.AddCamera(Vcw::VirtualCamera(cameraResolution, cameraPrincipalPoint, cameraFocalLength, cameraDistortionCoefficients));

    const cv::Mat &PropImage = cv::imread("CharucoGrid.png", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

    const double propDistance = 5.0;
    const double propWidth = 3.0;
    const double propHeight = 2.25;

    const cv::Size2d propSize(propWidth, propHeight);

    const cv::Affine3d &Room_T_Prop = Vcw::MatrixUtilities::CreateTransform(3 * 0.0872664625997165, 2 * 0.0872664625997165, 2 * 0.0872664625997165, 5.8, -8.4, propDistance);


    const Vcw::VirtualProp virtualProp(PropImage,propSize, Room_T_Prop);

    std::vector<Vcw::CameraPerspective> cameraPerspectives = virtualWorld.ComputeAllCameraPerspectivesOfProp(virtualProp);

    for(int k=0; k< cameraPerspectives.size(); k++)
    {
        cv::imwrite(std::to_string(cameraPerspectives[k].CameraID) + ".png", cameraPerspectives[k].Image);
    }

    return 0;
}

int main()
{
    Test0();

	return 0;
}







































