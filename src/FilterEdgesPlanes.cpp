/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterEdgesPlanes.cpp
 * @brief  Classify pointcloud voxels into planes / "edges".
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mola-kernel/yaml_helpers.h>
#include <mola-lidar-segmentation/FilterEdgesPlanes.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <yaml-cpp/yaml.h>

IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(
    FilterEdgesPlanes, mola::lidar_segmentation::FilterBase,
    mola::lidar_segmentation);

using namespace mola::lidar_segmentation;

FilterEdgesPlanes::FilterEdgesPlanes() = default;

void FilterEdgesPlanes::initialize(const std::string& cfg_block)
{
    MRPT_START

    // Parse YAML:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["pointcloud_filter_params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_OPT(params_, voxel_filter_resolution, double);
    YAML_LOAD_OPT(params_, voxel_filter_decimation, unsigned int);
    YAML_LOAD_OPT(params_, full_pointcloud_decimation, unsigned int);
    YAML_LOAD_OPT(params_, voxel_filter_max_e2_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter_max_e1_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter_min_e2_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter_min_e1_e0, float);

    filter_grid_.resize(
        {-75, -75.0, -10.0}, {75.0, 75.0, 10.0},
        params_.voxel_filter_resolution);

    MRPT_END
}

bool FilterEdgesPlanes::filterPointCloud(
    const mrpt::maps::CPointsMap& pc, mp2p_icp::pointcloud_t& out)
{
    MRPT_START

    out.clear();

    auto& pc_edges      = out.point_layers["edges"];
    auto& pc_planes     = out.point_layers["planes"];
    auto& pc_full_decim = out.point_layers["full_decim"];
    if (!pc_edges) pc_edges = mrpt::maps::CSimplePointsMap::Create();
    if (!pc_planes) pc_planes = mrpt::maps::CSimplePointsMap::Create();
    if (!pc_full_decim) pc_full_decim = mrpt::maps::CSimplePointsMap::Create();

    pc_edges->clear();
    pc_edges->reserve(pc.size() / 10);
    pc_planes->clear();
    pc_planes->reserve(pc.size() / 10);
    pc_full_decim->clear();
    pc_full_decim->reserve(pc.size() / 10);

    filter_grid_.clear();
    filter_grid_.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const float max_e20 = params_.voxel_filter_max_e2_e0;
    const float max_e10 = params_.voxel_filter_max_e1_e0;
    const float min_e20 = params_.voxel_filter_min_e2_e0;
    const float min_e10 = params_.voxel_filter_min_e1_e0;

    std::size_t nEdgeVoxels = 0, nPlaneVoxels = 0, nTotalVoxels = 0;
    for (const auto& vxl_pts : filter_grid_.pts_voxels)
    {
        if (!vxl_pts.indices.empty()) nTotalVoxels++;
        if (vxl_pts.indices.size() < 5) continue;

        // Analyze the voxel contents:
        mrpt::math::TPoint3Df mean{0, 0, 0};
        const float           inv_n = (1.0f / vxl_pts.indices.size());
        for (size_t i = 0; i < vxl_pts.indices.size(); i++)
        {
            const auto pt_idx = vxl_pts.indices[i];
            mean.x += xs[pt_idx];
            mean.y += ys[pt_idx];
            mean.z += zs[pt_idx];
        }
        mean.x *= inv_n;
        mean.y *= inv_n;
        mean.z *= inv_n;

        mrpt::math::CMatrixFixed<double, 3, 3> mat_a;
        mat_a.setZero();
        for (size_t i = 0; i < vxl_pts.indices.size(); i++)
        {
            const auto                  pt_idx = vxl_pts.indices[i];
            const mrpt::math::TPoint3Df a(
                xs[pt_idx] - mean.x, ys[pt_idx] - mean.y, zs[pt_idx] - mean.z);
            mat_a(0, 0) += a.x * a.x;
            mat_a(1, 0) += a.x * a.y;
            mat_a(2, 0) += a.x * a.z;
            mat_a(1, 1) += a.y * a.y;
            mat_a(2, 1) += a.y * a.z;
            mat_a(2, 2) += a.z * a.z;
        }
        mat_a *= inv_n;

        // Find eigenvalues & eigenvectors:
        // This only looks at the lower-triangular part of the cov matrix.
        mrpt::math::CMatrixFixed<double, 3, 3> eig_vectors;
        std::vector<double>                    eig_vals;
        mat_a.eig_symmetric(eig_vectors, eig_vals);

        const float e0 = eig_vals[0], e1 = eig_vals[1], e2 = eig_vals[2];

        mrpt::maps::CPointsMap* dest = nullptr;
        if (e2 < max_e20 * e0 && e1 < max_e10 * e0)
        {
            nEdgeVoxels++;
            dest = pc_edges.get();
        }
        else if (e2 > min_e20 * e0 && e1 > min_e10 * e0)
        {
            // Filter out horizontal planes, since their uneven density
            // makes ICP fail to converge.
            // A plane on the ground has its 0'th eigenvector like [0 0 1]

            const auto ev0 =
                eig_vectors.extractColumn<mrpt::math::TVector3D>(0);

            if (std::abs(ev0.z) < 0.9f)
            {
                nPlaneVoxels++;
                dest = pc_planes.get();
            }
        }
        if (dest != nullptr)
        {
            for (size_t i = 0; i < vxl_pts.indices.size();
                 i += params_.voxel_filter_decimation)
            {
                const auto pt_idx = vxl_pts.indices[i];
                dest->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
            }
        }
        for (size_t i = 0; i < vxl_pts.indices.size();
             i += params_.full_pointcloud_decimation)
        {
            const auto pt_idx = vxl_pts.indices[i];
            pc_full_decim->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
        }
    }
    MRPT_LOG_DEBUG_STREAM(
        "[VoxelGridFilter] Voxel counts: total=" << nTotalVoxels
                                                 << " edges=" << nEdgeVoxels
                                                 << " planes=" << nPlaneVoxels);

    return true;  // virtual method is implemented
    MRPT_END
}
