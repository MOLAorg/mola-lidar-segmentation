/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterParameters.h
 * @brief  Common parameter structure for all point cloud filters/detectors.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <cstdint>
#include <memory>

namespace mola::lidar_segmentation
{
/** Base class for parameter sets of lidar filtering/segmentation algorithms.
 *
 * \ingroup mola_lidar_segmentation_grp
 */
struct FilterParameters
{
    using Ptr = std::shared_ptr<FilterParameters>;
};

}  // namespace mola::lidar_segmentation
