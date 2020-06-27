
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ProbabilityDistributions.h"
#include "VirtualWorld.h"

const double ArcminToRads = 0.000290888;
const double DegToRads = 0.0174533;

std::vector<cv::Affine3d> DescribePropMotion()
{
    double propDistance = 0.5;

    // Describe the motion of the prop with reference to the world
    std::vector<cv::Affine3d> Room_T_Props;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, -20.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, -15.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, -10.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, -5.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, 0, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, 5.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, 10.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, 15.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, 20.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-25.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-20.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-15.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-10.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-5.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(0.0, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(5.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(10.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(15.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance += 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, 25.0 * DegToRads, 0, 0, 0, propDistance));

    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, 20.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, 15.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, 10.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, 5.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, 0.0, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, -5.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, -10.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, -15.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, -20.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(25.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(20.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(15.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(10.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(5.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(0.0, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-5.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-10.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-15.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));
    propDistance -= 0.02;
    Room_T_Props.push_back(Vcw::MatrixUtilities::CreateAffineTransform(-20.0 * DegToRads, -25.0 * DegToRads, 0, 0, 0, propDistance));

    return Room_T_Props;
}

int TestMultipleCameraPerspectives()
{

    Vcw::VirtualWorld virtualWorld = Vcw::VirtualWorld();

    const cv::Size cameraResolution(2000.0, 2000.0);
    const cv::Point2d cameraPrincipalPoint(1004.22305051, 989.892016271);
    const cv::Point2d cameraFocalLength(1785.29069304, 1785.42438858);
    const std::vector<double> cameraDistortionCoefficients(std::vector<double>({ -0.3184124974828117 , 0.1655891487212314, -0.00017802041075148632, 0.00017411575646864198, -0.056445932029310308 }));

    // Top Left Camera
    const cv::Affine3d &Room_T_Camera0 = Vcw::MatrixUtilities::CreateAffineTransform(4.191 * ArcminToRads, -603.286 * ArcminToRads, 0.673 * ArcminToRads, -0.053, -0.042, 0.0003);

    //Top Right Camera
    const cv::Affine3d &Room_T_Camera1 = Vcw::MatrixUtilities::CreateAffineTransform(-0.942 * ArcminToRads, 601.971 * ArcminToRads, 0.493 * ArcminToRads, 0.051, -0.044, -0.0001);

    //Bottom Left Camera
    const cv::Affine3d &Room_T_Camera2 = Vcw::MatrixUtilities::CreateAffineTransform(1.258 * ArcminToRads, -600.430 * ArcminToRads, -0.320 * ArcminToRads, -0.050, 0.031, 0.0003);

    //Bottom Right Camera
    const cv::Affine3d &Room_T_Camera3 = Vcw::MatrixUtilities::CreateAffineTransform(5.234 * ArcminToRads, 605.869 * ArcminToRads, -0.830 * ArcminToRads, 0.052, 0.035, 0.0003);

    const Vcw::CameraProperties cameraProperties(cameraResolution, cameraPrincipalPoint, cameraFocalLength, cameraDistortionCoefficients);

    // Position four cameras within the world, all with reference to the same point. All cameras are set to have the same intrinsic parameters
    virtualWorld.AddCamera(Vcw::VirtualCamera(cameraProperties, Room_T_Camera0));
    virtualWorld.AddCamera(Vcw::VirtualCamera(cameraProperties, Room_T_Camera1));
    virtualWorld.AddCamera(Vcw::VirtualCamera(cameraProperties, Room_T_Camera2));
    virtualWorld.AddCamera(Vcw::VirtualCamera(cameraProperties, Room_T_Camera3));

    const cv::Mat &PropImage = cv::imread("CharucoGrid.png", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

    const double propWidth = 0.75;
    const double propHeight = 0.75;

    const cv::Size2d propSize(propWidth, propHeight);

    const std::vector<cv::Affine3d> &Room_T_Props = DescribePropMotion();

    for(int k=0; k< Room_T_Props.size(); k++)
    {
        const Vcw::VirtualProp virtualProp(PropImage, propSize, Room_T_Props[k]);

        std::vector<Vcw::CameraPerspective> cameraPerspectives = virtualWorld.ComputeAllCameraPerspectivesOfProp(virtualProp);

        cv::Mat TopPerspectives;
        cv::hconcat(cameraPerspectives[0].Image, cameraPerspectives[1].Image, TopPerspectives);

        cv::Mat BottomPerspectives;
        cv::hconcat(cameraPerspectives[2].Image, cameraPerspectives[3].Image, BottomPerspectives);

        cv::Mat Perspectives;
        cv::vconcat(TopPerspectives, BottomPerspectives, Perspectives);

        cv::imwrite("Perspectives_" + std::to_string(k) + ".png", Perspectives);
    }

    return 0;
}

int main()
{
    TestMultipleCameraPerspectives();

	return 0;
}







































