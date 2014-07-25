#include <pcl/features/normal_3d.h>

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

    cloud.estimateAllNormals();

    cloud.writeMesh("test.ply");

    return 0;
}
