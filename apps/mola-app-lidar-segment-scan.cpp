/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mola-fe-lidar-segment-scan.cpp
 * @brief  test for 3D lidar scan segmentation
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2019
 */

#include <mola-lidar-segmentation/FilterEdgesPlanes.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

MRPT_TODO("Improve: allow changing FilterEdgesPlanes from cli flags");

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("test-mola-fe-lidar-segment-scan");

static TCLAP::ValueArg<std::string> arg_lidar_kitti_file(
    "k", "input-kitti-file", "Load 3D scan from a Kitti lidar (.bin) file",
    true, "", "./00000.bin", cmd);

// clang-format off
static TCLAP::ValueArg<std::string> arg_lidar_pose(
    "", "lidar-pose",
    "Defines the 4x4 homogeneous matrix of the LiDAR sensor in the vehicle "
    "frame.\n"
    "Use Matlab format, quoted as a string. For example:\n"
    "For KAIST left_VLP :\n"
    " --lidar-pose \"[-5.1406e-01 -7.0220e-01 -4.9259e-01 -4.4069e-01;"
    "           4.8648e-01 -7.1167e-01 5.0680e-01 3.9705e-01 ;"
    "          -7.0644e-01  2.0893e-02 7.0745e-01 1.9095e+00 ;"
    "           0 0 0 1 ]\"\n"
    "For KAIST right_VLP:\n"
    " --lidar-pose \"[-5.1215e-01 6.9924e-01 -4.9876e-01 -4.4988e-01;"
    "      -4.9481e-01 -7.1485e-01 -4.9412e-01 -4.1671e-01;"
    "      -7.0204e-01 -6.2641e-03 7.1210e-01 1.9129e+00;"
    "      0 0 0 1 ]\"\n",
    false, "",
    "[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]", cmd);
// clang-format on

static TCLAP::ValueArg<std::string> arg_params_file(
    "c", "config-file",
    "Load parameters from a YAML config file, containing a top-level YAML "
    "entry named `params` with all the parameters under it",
    true, "", "config.yml", cmd);

static mrpt::system::CTimeLogger timlog;

void do_scan_segment_test()
{
    // Load input point cloud:
    auto pc = mrpt::maps::CPointsMapXYZI::Create();

    const auto fil = arg_lidar_kitti_file.getValue();
    std::cout << "Loading: " << fil << "\n";

    timlog.enter("loadFromKittiVelodyneFile");
    pc->loadFromKittiVelodyneFile(fil);
    timlog.leave("loadFromKittiVelodyneFile");
    std::cout << "Done. " << pc->size() << " points.\n";

    // Set in the coordinate frame of the vehicle:
    if (arg_lidar_pose.isSet())
    {
        mrpt::math::CMatrixDouble44 HM;
        const auto                  sMat = arg_lidar_pose.getValue();
        if (!HM.fromMatlabStringFormat(sMat))
        {
            THROW_EXCEPTION_FMT(
                "Malformed matlab-like 4x4 homogeneous matrix: `%s`",
                sMat.c_str());
        }
        auto p = mrpt::poses::CPose3D(HM);
        std::cout << "Using sensor pose: " << p.asString() << "\n";
        pc->changeCoordinatesReference(p);
    }

    // Load params:
    const auto cfg_file = arg_params_file.getValue();
    ASSERT_FILE_EXISTS_(cfg_file);

    std::cout << "Loading param file: " << cfg_file << "\n";
    const auto cfg = YAML::LoadFile(cfg_file);
    std::cout << "Done.\n";
    std::string str_params;
    {
        std::stringstream ss;
        ss << cfg;
        str_params = ss.str();
    }

    std::cout << "Initializing with these params:\n" << str_params << "\n";

    MRPT_TODO("Convert to class factory!");
    auto filter = mola::lidar_segmentation::FilterEdgesPlanes::Create();
    filter->initialize(str_params);

    // Convert into input type:
    auto raw_input        = mrpt::obs::CObservationPointCloud::Create();
    raw_input->pointcloud = pc;
    mp2p_icp::pointcloud_t pc_features;

    mrpt::system::CTimeLoggerEntry tle1(timlog, "filterPointCloud");

    filter->filter(raw_input, pc_features);

    tle1.stop();

    // Display "layers":
    std::map<std::string, mrpt::gui::CDisplayWindow3D::Ptr> wins;
    int                                                     x = 5, y = 5;
    for (const auto& layer : pc_features.point_layers)
    {
        const auto name = layer.first;

        auto& win = wins[name] = mrpt::gui::CDisplayWindow3D::Create(name);

        mrpt::opengl::COpenGLScene::Ptr scene;

        {
            mrpt::gui::CDisplayWindow3DLocker lck(*win, scene);

            scene->clear();
            scene->insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 4.0f));

            auto gl_pc = mrpt::opengl::CSetOfObjects::Create();
            layer.second->getAs3DObject(gl_pc);
            scene->insert(gl_pc);

            auto msg = mrpt::format(
                "layer=`%s`  => %u points.", name.c_str(),
                static_cast<unsigned int>(layer.second->size()));
            win->addTextMessage(
                5, 5, msg, mrpt::img::TColorf(1, 1, 1), "sans", 10.0);

            win->setPos(x, y);
            y += 350;
        }
        win->repaint();
    }

    std::cout << "Close windows or hit a key on first window to quit.\n";
    if (!wins.empty()) wins.begin()->second->waitForKey();
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        do_scan_segment_test();
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
