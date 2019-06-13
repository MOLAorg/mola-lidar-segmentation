/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarFilterBase.cpp
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mola-lidar-segmentation/LidarFilterBase.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <yaml-cpp/yaml.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT_NS_PREFIX(
    LidarFilterBase, mrpt::rtti::CObject, mola::lidar_segmentation);

using namespace mola::lidar_segmentation;

void LidarFilterBase::filter(
    const mrpt::obs::CObservation::Ptr& o, mp2p_icp::pointcloud_t& out)
{
    MRPT_START
    using namespace mrpt::obs;

    bool processed = false;

    if (auto o0 = mrpt::ptr_cast<CObservationPointCloud>::from(o); o0)
    {
        ASSERT_(o0->pointcloud);
        processed = filterPointCloud(*o0->pointcloud, out);
    }
    else if (auto o1 = mrpt::ptr_cast<CObservation2DRangeScan>::from(o); o1)
        processed = filterScan2D(*o1, out);
    else if (auto o2 = mrpt::ptr_cast<CObservation3DRangeScan>::from(o); o2)
        processed = filterScan3D(*o2, out);
    else if (auto o3 = mrpt::ptr_cast<CObservationVelodyneScan>::from(o); o3)
        processed = filterVelodyneScan(*o3, out);
    else
    {
        THROW_EXCEPTION_FMT(
            "Unhandled observation type: `%s`",
            o->GetRuntimeClass()->className);
    }

    // done?
    if (processed) return;

    // convert into a point cloud and filter it:
    MRPT_LOG_ONCE_WARN(
        "Filter did not implemented an observation-specific implementation. "
        "Falling back to default point-cloud container filter.");

    mrpt::maps::CSimplePointsMap pc;
    if (!o->insertObservationInto(&pc))
    {
        THROW_EXCEPTION(
            "Observation could not be converted into a point cloud. Do not "
            "know what to do with this observation!");
    }

    if (!filterPointCloud(pc, out))
    {
        THROW_TYPED_EXCEPTION(
            "Derived class does not implement filterPointCloud()",
            NotImplementedError);
    }

    MRPT_END
}

bool LidarFilterBase::filterScan2D(  //
    [[maybe_unused]] const mrpt::obs::CObservation2DRangeScan& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&                   out)
{
    return false;  // Not implemented
}

bool LidarFilterBase::filterVelodyneScan(  //
    [[maybe_unused]] const mrpt::obs::CObservationVelodyneScan& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&                    out)
{
    return false;  // Not implemented
}

bool LidarFilterBase::filterScan3D(  //
    [[maybe_unused]] const mrpt::obs::CObservation3DRangeScan& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&                   out)
{
    return false;  // Not implemented
}

bool LidarFilterBase::filterPointCloud(  //
    [[maybe_unused]] const mrpt::maps::CPointsMap& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&       out)
{
    return false;  // Not implemented
}
