#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/poisson.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "pcMesher.h"

PcMesher::PcMesher(){

    pointClouds_.clear();
    nClouds_ = 0;

}

PcMesher::~PcMesher(){

}

unsigned int PcMesher::getNClouds(){
    return nClouds_;
}

void PcMesher::estimateNormals(const unsigned int _index){

    // Create the normal estimation class, and pass the input dataset to it
    PointCloud<PointXYZRGBNormal>::Ptr cloud = pointClouds_[_index];
    NormalEstimation<PointXYZRGBNormal, PointXYZRGBNormal> ne;

    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    search::KdTree<PointXYZRGBNormal>::Ptr tree (new search::KdTree<PointXYZRGBNormal> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.103); // <--------------------- IT'S IMPORTANT TO DETERMINE THIS NUMBER PROPERLY

    // Compute the features
    ne.compute (*cloud);

}


void PcMesher::estimateAllNormals(){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){

        estimateNormals(i);

    }
}

void PcMesher::planeSegmentation(){

    PointCloud<PointXYZRGBNormal>::Ptr cloud = pointClouds_[0];
    PointCloud<PointXYZRGBNormal>::Ptr cloud_p (new PointCloud<PointXYZRGBNormal>);
    PointCloud<PointXYZRGBNormal>::Ptr cloud_f (new PointCloud<PointXYZRGBNormal>);


      ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
      PointIndices::Ptr inliers (new PointIndices ());
      // Create the segmentation object
      SACSegmentation<PointXYZRGBNormal> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (SACMODEL_PLANE);
      seg.setMethodType (SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.01);

      // Create the filtering object
      ExtractIndices<PointXYZRGBNormal> extract;

      int i = 0, nr_points = (int) cloud->points.size ();
      // While 30% of the original cloud is still there
      while (cloud->points.size () > 0.3 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          cerr << "Could not estimate a planar model for the given dataset." << endl;
          break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << i << endl;

        stringstream ss;
        ss << "test_" << i << ".ply";
        io::savePLYFile(ss.str(), *cloud_p);

        pointClouds_.push_back(cloud_p);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
      }

      cerr << pointClouds_.size() << endl;



}


void PcMesher::surfaceReconstruction(const unsigned int _index){

    PointCloud<PointXYZRGBNormal>::Ptr cloud = pointClouds_[_index];

    Poisson<PointXYZRGBNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);

    io::savePLYFile("triangles.ply", mesh);

}



void PcMesher::readMesh(string _fileName){

    // This cloud is a temporal one which will be stored in the cloud vector
    PointCloud<PointXYZRGBNormal>::Ptr cloud (new PointCloud<PointXYZRGBNormal>);

    // Cloud file is loaded
    if (io::loadPLYFile<PointXYZRGBNormal>(_fileName, *cloud) == -1){
        string message("Couldn't read file ");
        message.append(_fileName);
        message.append(" \n");
        PCL_ERROR(message.c_str());
        return;
    }

    pointClouds_.push_back(cloud);
    nClouds_ = 1;

}

// All the point clouds in the vector are concatenated and printed into one file
void PcMesher::writeMesh(string _fileName){

    PointCloud<PointXYZRGBNormal> outPointCloud;

    for (unsigned int i = 0; i < pointClouds_.size(); i++){
        outPointCloud += *pointClouds_[i];
    }

    io::savePLYFile(_fileName, outPointCloud);
}



int main (int argc, char *argv[]){

    PcMesher cloud;

    cloud.readMesh(argv[1]);

    cloud.planeSegmentation();
    cloud.estimateAllNormals();
    cloud.surfaceReconstruction(0);


    cloud.writeMesh("test.ply");

    return 0;
}
