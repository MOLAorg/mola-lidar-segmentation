/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterEdgesPlanes.h
 * @brief  Classify pointcloud voxels into planes / "edges".
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mola-lidar-segmentation/LidarFilterBase.h>
#include <mola-lidar-segmentation/PointCloudToVoxelGrid.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/maps/CPointsMap.h>

namespace mola::lidar_segmentation
{
/** Accepts as input a point cloud, and classifies points into edges and planes.
 *
 *
 * \ingroup mola_lidar_segmentation_grp
 */
class FilterEdgesPlanes : public mola::lidar_segmentation::LidarFilterBase
{
    DEFINE_MRPT_OBJECT(FilterEdgesPlanes, mola::lidar_segmentation)
   public:
    FilterEdgesPlanes();

    void initialize(const std::string& cfg_block) override;

    struct Parameters : public FilterParameters
    {
        /** Size of the voxel filter [meters] */
        unsigned int full_pointcloud_decimation{20};
        double       voxel_filter_resolution{.5};
        unsigned int voxel_filter_decimation{1};
        float        voxel_filter_max_e2_e0{30.f}, voxel_filter_max_e1_e0{30.f};
        float voxel_filter_min_e2_e0{100.f}, voxel_filter_min_e1_e0{100.f},
            voxel_filter_min_e1{.0f};
    };

    /** Algorithm parameters */
    Parameters params_;

   protected:
    // See base docs
    bool filterPointCloud(
        const mrpt::maps::CPointsMap& pc, mp2p_icp::pointcloud_t& out) override;
    bool filterScan2D(
        const mrpt::obs::CObservation2DRangeScan& obs,
        mp2p_icp::pointcloud_t&                   out) override;

   private:
    PointCloudToVoxelGrid filter_grid_;
};

/** @} */

}  // namespace mola::lidar_segmentation
