#include <iostream>
#include <algorithm>
#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "visualize_correspondences.h"

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>


void
loadFile(const char* fileName,
	 pcl::PointCloud<pcl::PointXYZ> &cloud
)
{
  //pcl::PolygonMesh mesh;
  
  // if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  if ( pcl::io::loadPCDFile<pcl::PointXYZ> ( fileName, cloud ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  // else
    // pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

int main (int argc, char** argv)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  {
    // load source
    loadFile ( argv[1], *cloud_source );
    // load target
    // loadFile ( argv[2], *cloud_target );
    std::cout << "Loaded "
            << cloud_source->width << " x " << cloud_source->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  }
  
   // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_source);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (7.0, 10.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_target); 
  
  std::cerr << "Origin " << cloud_source->size () << " data points to " << argv[1] << std::endl;
  pcl::io::savePCDFileASCII (argv[2], *cloud_target);
  std::cerr << "Saved " << cloud_target->size () << " data points to " << argv[2] << std::endl;
  
  return(0);
}

