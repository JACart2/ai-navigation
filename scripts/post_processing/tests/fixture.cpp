// Small field-rich SimpleMap for native preservation tests; no mapper is run.
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <iostream>
int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "create") {
        mrpt::maps::CSimpleMap sm;
        for (int i=0;i<6;++i) {
            auto p=mrpt::maps::CGenericPointsMap::Create();
            p->registerField_uint16("ring");p->registerField_float("t");p->resize(2);
            p->setPointFast(0,-1,0,-.1);p->setPointFast(1,2,0,0);
            p->setPointField_uint16(0,"ring",10);p->setPointField_uint16(1,"ring",11);
            p->setPointField_float(0,"t",.01);p->setPointField_float(1,"t",.02);
            auto obs=mrpt::obs::CObservationPointCloud::Create();
            obs->pointcloud=p;obs->timestamp=mrpt::Clock::fromDouble(1000+i);
            auto sf=mrpt::obs::CSensoryFrame::Create();sf->insert(obs);
            auto pose=mrpt::poses::CPose3DPDFGaussian::Create();pose->mean=mrpt::poses::CPose3D(i,0,0,0,0,0);
            sm.insert(pose,sf);
        }
        return sm.saveToFile(argv[2])?0:1;
    }
    if (argc == 3 && std::string(argv[1]) == "check-gps") {
        mrpt::maps::CSimpleMap sm;if(!sm.loadFromFile(argv[2])||sm.size()!=6)return 1;
        for(const auto& kf:sm) {
            auto gps=kf.sf->getObservationByClass<mrpt::obs::CObservationGPS>();
            if(!gps||!gps->covariance_enu||std::abs((*gps->covariance_enu)(0,0)-.25)>1e-9||std::abs(gps->sensorPose.z()-.2)>1e-9)return 2;
            auto obs=kf.sf->getObservationByClass<mrpt::obs::CObservationPointCloud>();
            if(!obs||obs->pointcloud->size()!=2)return 3;
        }
        std::cout<<"PASS: six original timestamped fixes attached; covariance, antenna TF and clouds preserved\n";
        return 0;
    }
    return 1;
}
