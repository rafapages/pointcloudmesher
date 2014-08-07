#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <stdio.h>

#include "pointXYZRGBNormalCam.h"

class PcMesher{

public:

    PcMesher();
    ~PcMesher();

    // Get number of clouds
    unsigned int getNClouds();

    // Estimate the normals of a cloud and store them
    void estimateNormals(const unsigned int _index);
    void estimateAllNormals();

    // Plane segmentation
    void planeSegmentation();

    // Cilinder segmentation
    void cilinderSegmentation();

    // Surface reconstruction
    void surfaceReconstruction(const unsigned int _index);
    void allSurfaceReconstruction();


    // I/O functions
    void readMesh(string _fileName);
    void writeOneMesh(const unsigned int _index, string _fileName);
    void writeMesh(string _fileName);

private:

    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > pointClouds_;
    unsigned int nClouds_;

};







#endif
