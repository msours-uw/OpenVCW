
#pragma once

#include "VirtualCamera.h"

namespace Vcw
{
    class VirtualWorld
    {
        public:

            VirtualWorld() {}
            VirtualWorld(const std::vector<VirtualCamera> &VirtualCameras);

            void AddCamera(const VirtualCamera &virtualCamera);
            void RemoveCamera(const int CameraID);
            void MoveCamera(const int CameraID, const cv::Affine3d &Room_T_Camera_New);

            VirtualCamera FindCamera(const int CameraID) const;
            cv::Affine3d FindTransformBetweenCameras(const int CameraID_0, const int CameraID_1) const;

            std::vector<VirtualCamera> VirtualCameras;

        private:

            int MaxCameraID = -1;
    };
}


