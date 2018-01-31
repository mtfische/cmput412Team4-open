#include <ctime>
#include <stdint.h>
#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
  std::clock_t    start, filter_time, planar_time, cluster_time;
  start = std::clock();
  filter_time = std::clock();
  planar_time = std::clock();
  cluster_time = std::clock();
  std::cout << "Start:" << std::endl;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);
  //std::cout << "PointCloud before filtering has: " << input_cloud->points.size () << " data points." << std::endl; //*

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZ>);
  
  Eigen::Vector4f minPoint;
    minPoint[0]=-10;  // define minimum point x
    minPoint[1]=0;  // define minimum point y
    minPoint[2]=0;  // define minimum point z
  Eigen::Vector4f maxPoint;
    maxPoint[0]=10;  // define max point x
    maxPoint[1]=3;  // define max point y
    maxPoint[2]=4;  // define max point z

     pcl::CropBox<pcl::PointXYZ> cropFilter;
        cropFilter.setInputCloud (input_cloud);
        cropFilter.setMin(minPoint);
        cropFilter.setMax(maxPoint);

    cropFilter.filter(*cloud_cropped); 

  
 



  std::stringstream ss;
  pcl::PCDWriter writer;
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud_cropped);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  filter_time = std::clock();
  std::cout << "Filter computation time: " << (filter_time-start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    
  // Create the segmentation object for the planar model and set all the parameters
  //This will be used to remove planes from the depth data (walls)
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  //pub.publish(*cloud_filtered);

  planar_time = std::clock();
  std::cout << "planar computation time: " << (planar_time-filter_time) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (200);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZ>);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    float min_x,min_y,max_x,max_y, z;
    min_x = 10; min_y = 10; max_x = -10; max_y = -10;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    { 
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
      z = cloud_filtered->points[*pit].z;
      if(min_x > cloud_filtered->points[*pit].x)
        min_x = cloud_filtered->points[*pit].x;
      if(max_x < cloud_filtered->points[*pit].x)
        max_x = cloud_filtered->points[*pit].x;
      if(min_y > cloud_filtered->points[*pit].y)
        min_y = cloud_filtered->points[*pit].y;
      if(max_y < cloud_filtered->points[*pit].y)
        max_y = cloud_filtered->points[*pit].y;
    }
    //for (std::vector<pcl_po>; cloud_cluster.begin (); )
    //std::cout << "min_x: " << min_x << " min_y: "<<min_y<< " max_x: "<<max_x<< " max_y: "<<max_y <<std::endl;
    //std::cout << "height: " << max_y-min_y << " width: " << max_x-min_x <<std::endl; 
    //std::cout << "The height of the cluster: " << << " Total width: "<< <<std::endl;
    bool reject = false;
    if(z > 6){
      //std::cout << "rejecting cluster due to distance!" <<std::endl; 
      continue;
    }
    if(max_y-min_y < 0.05 || max_y-min_y > 0.25)
      {
        //std::cout << "rejecting cluster due to height!" <<std::endl; 
        reject = true;
        //continue;
      }
    if(max_x-min_x < 0.1 || max_x-min_x > 0.4)
      {
        //std::cout << "rejecting cluster due to width!" <<std::endl; 
        reject = true;
        //continue;
      }
    //std::cout << "min_x: " << min_x << " min_y: "<<min_y<< " max_x: "<<max_x<< " max_y: "<<max_y <<std::endl;
    //std::cout << "height: " << max_y-min_y << " width: " << max_x-min_x <<std::endl; 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header.frame_id = "camera_rgb_optical_frame";


    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    if(reject){
      ss << "rejected_cluster_" << j << ".pcd";  
    }else{
      *cloud_output = *cloud_cluster;
      std::cout << "Cluster Found!" << std::endl;
      ss << "cloud_cluster_" << j << ".pcd";  
    }
    //ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
    j++;
  }

  pub.publish(*cloud_output); 
  

  cluster_time = std::clock();
  std::cout << "Cluster computation time: " << (cluster_time-planar_time) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Total time elapsed: " << (std::clock()-start) / (double)(CLOCKS_PER_SEC / 1000) << " ms\n" << std::endl;
     
  //ss << "cloud_cluster_" << "filter" << ".pcd";
  //writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false); 
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "find_target_node");
  ros::NodeHandle nh;

  //ROS subscriber to pointcloud topic
  //CB will convert into pcl::pointCloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);

  //ROS publisher for outputing a centroid obj of the target
  pub = nh.advertise<PointCloud>("target_cluster", 1);
//  ros::Publisher pub = nh.advertise<pcl::PointCloud> ("points2", 1);

  //cause spinning
  ros::spin();
}
/*
int
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (200);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    float min_x,min_y,max_x,max_y, z;
    min_x = 10; min_y = 10; max_x = -10; max_y = -10;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
      z = cloud_filtered->points[*pit].z;
      if(min_x > cloud_filtered->points[*pit].x)
        min_x = cloud_filtered->points[*pit].x;
      if(max_x < cloud_filtered->points[*pit].x)
        max_x = cloud_filtered->points[*pit].x;
      if(min_y > cloud_filtered->points[*pit].y)
        min_y = cloud_filtered->points[*pit].y;
      if(max_y < cloud_filtered->points[*pit].y)
        max_y = cloud_filtered->points[*pit].y;
    }
    //for (std::vector<pcl_po>; cloud_cluster.begin (); )
    //std::cout << "min_x: " << min_x << " min_y: "<<min_y<< " max_x: "<<max_x<< " max_y: "<<max_y <<std::endl;
    //std::cout << "height: " << max_y-min_y << " width: " << max_x-min_x <<std::endl; 
    //std::cout << "The height of the cluster: " << << " Total width: "<< <<std::endl;
    bool reject = false;
    if(z > 6){
      //std::cout << "rejecting cluster due to distance!" <<std::endl; 
      continue;
    }
    if(max_y-min_y < 0.05 || max_y-min_y > 0.25)
      {
        std::cout << "rejecting cluster due to height!" <<std::endl; 
        reject = true;
        //continue;
      }
    if(max_x-min_x < 0.1 || max_x-min_x > 0.4)
      {
        std::cout << "rejecting cluster due to width!" <<std::endl; 
        reject = true;
        //continue;
      }
    std::cout << "min_x: " << min_x << " min_y: "<<min_y<< " max_x: "<<max_x<< " max_y: "<<max_y <<std::endl;
    std::cout << "height: " << max_y-min_y << " width: " << max_x-min_x <<std::endl; 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    if(reject){
      ss << "rejected_cluster_" << j << ".pcd";  
    }else{
      ss << "cloud_cluster_" << j << ".pcd";  
    }
    //ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
*/