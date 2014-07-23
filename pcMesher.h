#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

class PcMesher{

public:

    PcMesher();
    ~PcMesher();

    // Estimate the normals of a cloud and store them
    void estimateNormals(PointCloud<PointXYZRGBNormal>::Ptr& _cloud);
    void estimateAllNormals();

    // I/O functions
    void readMesh(string _fileName);
    void writeMesh(string _fileName);

private:

    vector<PointCloud<PointXYZRGBNormal>::Ptr > pointClouds_;


};







#endif
