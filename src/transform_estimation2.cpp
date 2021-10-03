#include <iostream>
#include <fstream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <boost/random.hpp>

enum methods {
  SVD,
  DQ,
  LM
};

void loadExcel(std::string excel, pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
  std::ifstream ifs(excel.c_str());
  std::string line, value;
  while (getline(ifs, line)) {
    std::istringstream isstr(line);
    float fval[6];
    for (int j = 0; j < 6; ++j) {
      std::getline(isstr, value, ',');
      fval[j] = std::stof(value);
      std::cout << fval[j] << std::endl;
    }
    source->push_back(pcl::PointXYZ (fval[0], fval[1], fval[2]));
    // PTZ 的转换
    float p = fval[3] * PI / 1800, t = fval[4] * PI / 1800, z = fval[5];
    fval[3] = z * cos(t) * cos(p);
    fval[4] = z * cos(t) * sin(p);
    fval[5] = z * sin(t);
    target->push_back(pcl::PointXYZ (fval[3], fval[4], fval[5]));
  }
}
int main (int argc, char** argv)
{
  
  int method = SVD;
  pcl::console::parse_argument (argc, argv, "-m", method);
  std::cout << "method: ";
  switch (method) {
    case SVD: std::cout << "SVD"; break;
    case DQ:  std::cout << "DQ";  break;
    case LM:  std::cout << "LM";  break;
    default: std::cout << "undefined. ERROR" << std::endl; exit(0);
  }
  std::cout << std::endl;

  bool use_scale = false;
  pcl::console::parse_argument (argc, argv, "-s", use_scale);
  std::cout << "use scale: " << (use_scale ? "true" : "false") << std::endl;
  if (use_scale) {
      std::cout << "forse SVD." << std::endl;
      method = SVD;
  }

  std::string excel_file;
  pcl::console::parse_argument (argc, argv, "-e", excel_file);
  std::cout << "Excel File: " << excel_file << std::endl;
  /*
  std::string pcd_from, pcd_to;
  pcl::console::parse_argument (argc, argv, "-f", pcd_from);
  std::cout << "PCD from: " << pcd_from << std::endl;
  pcl::console::parse_argument (argc, argv, "-t", pcd_to);
  std::cout << "PCD to: " << pcd_to << std::endl;
  */

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  loadExcel(excel_file, cloud_source, cloud_target);
  // create random source point cloud
  /*
  for (int i = 0; i < 1000; i++) {
    cloud_source->push_back (pcl::PointXYZ (frand(gen), frand(gen), frand(gen) ));
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_from, *cloud_source) != -1) {
    PCL_ERROR("Cloudn't read file %s \n", pcd_from);
    return -1;
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_to, *cloud_target) != -1) {
    PCL_ERROR("Cloudn't read file %s \n", pcd_to);
    return -1;
  }
  */

  /*
  cloud_source->push_back (pcl::PointXYZ (4.576,-0.099,-0.467 ));
  cloud_source->push_back (pcl::PointXYZ (4.456,0.01,-0.347 ));
  cloud_source->push_back (pcl::PointXYZ (4.456,0.01,-0.397 ));
  cloud_source->push_back (pcl::PointXYZ (2.576,-0.099,-0.467 ));
  cloud_source->push_back (pcl::PointXYZ (2.456,0,-0.347 ));
  cloud_source->push_back (pcl::PointXYZ (2.456,0,-0.397 ));
  cloud_target->push_back (pcl::PointXYZ (97,1806,43 ));
  cloud_target->push_back (pcl::PointXYZ (136,1622,43 ));
  cloud_target->push_back (pcl::PointXYZ (110,1713,43 ));
  cloud_target->push_back (pcl::PointXYZ (139,1806,23 ));
  cloud_target->push_back (pcl::PointXYZ (220,1622,23 ));
  cloud_target->push_back (pcl::PointXYZ (220,1713,23 ));
  */
  boost::shared_ptr< pcl::registration::TransformationEstimation< pcl::PointXYZ, pcl::PointXYZ > > estPtr;
  if ( use_scale )
    // estimator of R and T along with scale
    estPtr.reset ( new pcl::registration::TransformationEstimationSVDScale < pcl::PointXYZ, pcl::PointXYZ > () );
  else 
    // estimator of R and T
    switch (method) {
    case SVD:
      estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    case DQ:
      estPtr.reset ( new pcl::registration::TransformationEstimationDualQuaternion < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    case LM:
      estPtr.reset ( new pcl::registration::TransformationEstimationLM < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    }

    
  Eigen::Affine3f transformation_est;
  estPtr->estimateRigidTransformation ( *cloud_source,
                                        *cloud_target,
                                        transformation_est.matrix() );
  
  if ( use_scale ) {
    Eigen::Matrix3f R = transformation_est.matrix().topLeftCorner(3,3);
    std::cout << "estimated scale " << std::sqrt( (R.transpose() * R).trace() / 3.0 ) << std::endl;
  }
  std::cout << "estimated transformation " << std::endl << transformation_est.matrix()  << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptz ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::transformPointCloud ( *cloud_source, *ptz, transformation_est);
  std::cout << "estimated results " << std::endl << *ptz;
  std::cout << ptz->points[0] << std::endl;
  std::cout << ptz->points[1] << std::endl;
  return ( 0 );
}

