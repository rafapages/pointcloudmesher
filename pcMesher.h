#ifndef PCMESHER_H
#define PCMESHER_H

#include <iostream>
#include <stdio.h>

#include "pointXYZRGBNormalCam.h"
#include "camera.h"

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
    void readMesh(std::string _fileName);
    void writeOneMesh(const unsigned int _index, std::string _fileName);
    void writeMesh(std::string _fileName);
    void bundlerReader(std::string _filename);

private:

    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr > pointClouds_;
    unsigned int nClouds_;

    std::vector<Camera> cameras_;
    unsigned int nCameras_;

};

#endif
