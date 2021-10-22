/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk \n", argv[0]);
}

/**
 * @brief 将输入点云转换为凸包体，并计算凸包的体积和表面积
 * 
 * @param cloud_in 输入点云
 * @param mesh_out 凸包体
 */
void
compute (PointCloud<PointXYZ>::ConstPtr cloud_in,
         PolygonMesh &mesh_out)
{
  print_info ("Computing the convex hull of a cloud with %lu points.\n", cloud_in->size ());
  ConvexHull<PointXYZ> convex_hull;
  convex_hull.setComputeAreaVolume(true);
  convex_hull.setInputCloud (cloud_in);
  convex_hull.reconstruct (mesh_out);
  print_info("Total Area %lf\n", convex_hull.getTotalArea());
  print_info("Total Volume %lf\n", convex_hull.getTotalVolume());
}

/**
 * @brief 点云投影到一个平面
 * 
 * @param input 输入的点云数据
 * @param output 输出的点云数据，只有一个平面了
 * @param a 
 * @param b  平面公式 Ax + By + Cz + D = 0 
 * @param c 
 * @param d 
 */
void
project (const PointCloud<PointXYZ>::Ptr &input, PointCloud<PointXYZ>::Ptr &output, float a, float b, float c, float d)
{
  Eigen::Vector4f coeffs;
  coeffs << a, b, c, d;

  // Estimate
  TicToc tt;
  tt.tic ();

  //First, we'll find a point on the plane
  print_highlight (stderr, "Projecting ");

  //PointCloud<PointXYZ>::Ptr projected_cloud_pcl (new PointCloud<PointXYZ>);
  output->width = input->width;
  output->height = input->height;
  output->is_dense = input->is_dense;
  output->sensor_origin_ = input->sensor_origin_;
  output->sensor_orientation_ = input->sensor_orientation_;

  for (const auto& point: *input)
  {
    pcl::PointXYZ projection;
    pcl::projectPoint <pcl::PointXYZ> (point, coeffs, projection);
    output->points.push_back(projection);
  }


  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
  pcl::io::savePCDFile ("foo.pcd", *output);

}

/* ---[ */
int
main (int argc, char** argv)
{
  printHelp(argc, argv);
  std::vector<int> pcd_file_indices;
  pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need one input PCD file to continue.\n");
    return (-1);
  }

  std::vector<int> vtk_file_indices;
  vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  if (vtk_file_indices.size () != 1)
  {
    print_error ("Need one output VTK file to continue.\n");
    return (-1);
  }

  // Load in the point cloud
  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
  if (loadPCDFile (argv[pcd_file_indices[0]], *cloud_in) != 0)
  {
    print_error ("Could not load input file %s\n", argv[pcd_file_indices[0]]);
    return (-1);
  }

  // Compute the hull
  PolygonMesh mesh_out;
  compute (cloud_in, mesh_out);

  // Save the mesh
  io::saveVTKFile (argv[vtk_file_indices[0]], mesh_out);
  // Command line parsing
  float a = static_cast<float> (atof (argv[3]));
  float b = static_cast<float> (atof (argv[4]));
  float c = static_cast<float> (atof (argv[5]));
  float d = static_cast<float> (atof (argv[6]));

  PointCloud<PointXYZ>::Ptr cloud_out (new PointCloud<PointXYZ> ());
  project(cloud_in, cloud_out, a, b, c, d);
  return (0);
}

