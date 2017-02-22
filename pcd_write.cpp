#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/poisson.h>


int main (int argc, char** argv)
{
    std::cout << "Loading file" << std::endl;
  
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile("bun0.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    //* the data should be available in cloud

    // Create the object that the filtered point cloud will be stored in
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Filter in x axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(0.0, 1.0);
    pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered_x);
    
    // Filter in z axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xz(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud_filtered_x);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10, 10);
    pass.filter(*cloud_filtered_xz);

    // Filter in y axis
    // NOTE: This wont work in production since the y coordinate will be different for different angles
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud_filtered_xz);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, 564);
    pass.filter(*cloud_filtered_xyz);

    // Remove points that are far away from other points
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud_filtered_xyz);
    outlier_filter.setRadiusSearch(1.2);
    outlier_filter.setMinNeighborsInRadius(2);
    outlier_filter.filter(*cloud_filtered);
  
    std::cout << "Saving filtered cloud in bun0_filtered.pcd" << std::endl;
    pcl::io::savePCDFile("bun0_filtered.pcd", *cloud_filtered);
  
    std::cout << "Estimating normals" << std::endl;
    // Normal estimation*
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should now contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    std::cout << "Begin poisson reconstruction" << std::endl;
    
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_with_normals);
    pcl::PolygonMesh triangles;
    poisson.reconstruct(triangles);
  
    if (pcl::io::savePolygonFile("mesh.stl", triangles) && pcl::io::savePolygonFile("mesh.vtk", triangles)) {
        std::cout << "Saved file successfully" << std::endl;
    } else {
        std::cout << "Failed to save file" << std::endl;
    }
	      
    // Finish
    return(0);
}
