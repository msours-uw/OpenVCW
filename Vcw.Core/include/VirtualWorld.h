
#pragma once

#include "VirtualCamera.h"

namespace Vcw
{
	struct CameraPerspective;

	class VirtualWorld
	{
	public:

		VirtualWorld() {}
		VirtualWorld(const std::vector<VirtualCamera> &VirtualCameras);

		std::vector<CameraPerspective> ComputeAllCameraPerspectivesOfProp(const VirtualProp &virtualProp);

		void AddCamera(const VirtualCamera &virtualCamera);
		void RemoveCamera(const int CameraID);
		void MoveCamera(const int CameraID, const cv::Affine3d &Room_T_Camera_New);

		VirtualCamera FindCamera(const int CameraID) const;
		cv::Affine3d FindTransformBetweenCameras(const int CameraID_0, const int CameraID_1) const;

		std::vector<VirtualCamera> VirtualCameras;

	private:

		int MaxCameraID = -1;
	};

	struct CameraPerspective
	{
		CameraPerspective() {}
		CameraPerspective(const cv::Mat &Image, const int CameraID) : Image(Image), CameraID(CameraID) { }

		CameraPerspective(const CameraPerspective &cameraPerspective)\
		{
			this->Image = cameraPerspective.Image;
			this->CameraID = cameraPerspective.CameraID;
		}

		cv::Mat Image;
		int CameraID;
	};
}