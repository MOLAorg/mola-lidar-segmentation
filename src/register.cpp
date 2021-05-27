/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  RTTI registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mola-lidar-segmentation/FilterEdgesPlanes.h>
#include <mrpt/core/initializer.h>

MRPT_INITIALIZER(register_mola_lidar_segmentation)
{
    using mrpt::rtti::registerClass;

    registerClass(CLASS_ID(mola::lidar_segmentation::FilterEdgesPlanes));
}
