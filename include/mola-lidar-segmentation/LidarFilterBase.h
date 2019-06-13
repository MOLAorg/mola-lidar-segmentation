/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarFilterBase.h
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mola-lidar-segmentation/FilterParameters.h>
#include <mp2p_icp/pointcloud.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <cstdint>
#include <stdexcept>

namespace mola::lidar_segmentation
{
/** Used in LidarFilterBase
 * \ingroup mola_lidar_segmentation_grp
 */
struct NotImplementedError : public std::runtime_error
{
    NotImplementedError() = default;
    template <typename T>
    NotImplementedError(T v) : std::runtime_error(v)
    {
    }
};

/** Provides filter() methods to convert a raw point cloud into segments,
 * features, etc. depending on the derived class implementation.
 *
 * Different signatures exists for:
 * - 2D LiDAR range scans (mrpt::obs::CObservation2DRangeScan)
 * - 3D Velodyne scans (mrpt::obs::CObservationVelodyneScan)
 * - 3D RGBD camera images (mrpt::obs::CObservation3DRangeScan)
 * - Generic 2D/3D point clouds (mrpt::obs::CObservationPointCloud)
 *
 * Notice that the methods using scans instead of point cloud have
 * computational advantages since they have a direct access to the raw
 * ordering of points in the scan, instead of simply their 3D coordinates.
 *
 * A call to filter() will first try to use the derived-class specific methods
 * for the particular kind of observation passed as input. If that
 * observation-specific method is not implemented in the base class, the
 * observation will then be converted on-the-fly into a point cloud, and the
 * corresponding method will be called. If that one is neither implemented, an
 * exception will be raised.
 *
 * Derived classes may implement all or only one of those methods. An
 * exception NotImplementedError will be thrown if an non-implemented method is
 * called.
 *
 * \note filter() is not required to be thread (multientry) safe.
 *
 * \sa Implementation in FilterEdgesPlanes
 *
 * \ingroup mola_lidar_segmentation_grp
 */
class LidarFilterBase : public mrpt::rtti::CObject,  // RTTI support
                        public mrpt::system::COutputLogger  // Logging support
{
    DEFINE_VIRTUAL_MRPT_OBJECT(LidarFilterBase)

   public:
    LidarFilterBase();

    /** \name API for all filtering/segmentation algorithms
     *  @{ */

    /** Loads, from a YAML configuration block, all the common, and
     * implementation-specific parameters. */
    virtual void initialize(const std::string& cfg_block) = 0;

    /** See docs above for LidarFilterBase.
     * This method dispatches the observation by type to the corresponding
     * virtual method
     */
    void filter(
        const mrpt::obs::CObservation::Ptr& input_raw,
        mp2p_icp::pointcloud_t&             out);

    /** @} */

   protected:
    // To be overrided in derived classes, if implemented:
    /** Process a 2D lidar scan. \return false if not implemented */
    virtual bool filterScan2D(
        const mrpt::obs::CObservation2DRangeScan& pc,
        mp2p_icp::pointcloud_t&                   out);
    /** Process a depth camera observation. \return false if not implemented */
    virtual bool filterScan3D(
        const mrpt::obs::CObservation3DRangeScan& pc,
        mp2p_icp::pointcloud_t&                   out);
    /** Process a 3D lidar scan. \return false if not implemented   */
    virtual bool filterVelodyneScan(
        const mrpt::obs::CObservationVelodyneScan& pc,
        mp2p_icp::pointcloud_t&                    out);
    /** Process a 2D/3D point-cloud. \return false if not implemented  */
    virtual bool filterPointCloud(
        const mrpt::maps::CPointsMap& pc, mp2p_icp::pointcloud_t& out);
};

}  // namespace mola::lidar_segmentation
