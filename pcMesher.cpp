#include "pcMesher.h"

PcMesher::PcMesher(){

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

    pointClouds_.push_back(*cloud);

}

// All the point clouds in the vector are concatenated and printed into one file
void PcMesher::writeMesh(string _fileName){

    PointCloud<PointXYZRGBNormal> outPointCloud;

    for (unsigned int i = 0; i < pointClouds_.size(); i++){
        outPointCloud += pointClouds_[i];
    }

    io::savePLYFile(_fileName, outPointCloud);
}



int main (int argc, char *argv[]){
    return 0;
}
