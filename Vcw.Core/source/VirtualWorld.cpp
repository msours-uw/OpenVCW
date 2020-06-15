
#include "VirtualWorld.h"

namespace Vcw
{
    VirtualWorld::VirtualWorld(const std::vector<VirtualCamera> &VirtualCameras) : VirtualCameras(VirtualCameras)
    {
        int k = -1;
        for(k = 0; k < VirtualCameras.size(); k++) this->VirtualCameras[k].ID = k;
        this->MaxCameraID = k;
    }
    void VirtualWorld::AddCamera(const VirtualCamera &virtualCamera)
    {
        this->VirtualCameras.push_back(virtualCamera);
        this->VirtualCameras.back().ID = ++this->MaxCameraID;
    }
    void VirtualWorld::RemoveCamera(const int CameraID)
    {
       this->VirtualCameras.erase(
                    std::remove_if(this->VirtualCameras.begin(), this->VirtualCameras.end(), [&](const VirtualCamera vc)->bool {
                        return vc.ID == CameraID;
                    }), this->VirtualCameras.end());

    }
    void VirtualWorld::MoveCamera(const int CameraID, const cv::Affine3d &Room_T_Camera_New)
    {
        for(int k = 0; k < this->VirtualCameras.size(); k++) if(VirtualCameras[k].ID == CameraID) VirtualCameras[k].Room_T_Camera = Room_T_Camera_New;
    }
    VirtualCamera VirtualWorld::FindCamera(const int CameraID) const
    {
        for(int k = 0; k < this->VirtualCameras.size(); k++) if(VirtualCameras[k].ID == CameraID) return VirtualCameras[k];
        return VirtualCamera();
    }
    cv::Affine3d VirtualWorld::FindTransformBetweenCameras(const int CameraID_0, const int CameraID_1) const
    {
        return FindCamera(CameraID_0).Room_T_Camera.inv() * FindCamera(CameraID_1).Room_T_Camera;
    }
}
