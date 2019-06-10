/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mola-fe-lidar-3d-segment-scan.cpp
 * @brief  test for 3D lidar scan segmentation
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2019
 */

#include <mola-fe-lidar-3d/LidarOdometry3D.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("test-mola-fe-lidar-3d-segment-scan");

static TCLAP::ValueArg<std::string> arg_lidar_kitti_file(
    "k", "input-kitti-file", "Load 3D scan from a Kitti lidar (.bin) file",
    true, "", "./00000.bin", cmd);

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

    // Filter it:
    mola::LidarOdometry3D module;

    // Not needed outside of a real SLAM system:
    // module.initialize_common();

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

    module.initialize(str_params);

    mola::LidarOdometry3D::lidar_scan_t pcs;
    pcs.pc.point_layers["original"] = pc;

    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "filterPointCloud");

        module.filterPointCloud(pcs);
    }

    // Display "layers":
    std::map<std::string, mrpt::gui::CDisplayWindow3D::Ptr> wins;
    int                                                     x = 5, y = 5;
    for (const auto& layer : pcs.pc.point_layers)
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
