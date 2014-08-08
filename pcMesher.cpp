#define PCL_NO_PRECOMPILE

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

//#include<boost/tokenizer.hpp>


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
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[_index];
    NormalEstimation<PointXYZRGBNormalCam, PointXYZRGBNormalCam> ne;

    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    search::KdTree<PointXYZRGBNormalCam>::Ptr tree (new search::KdTree<PointXYZRGBNormalCam> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (1.03); // <--------------------- IT'S IMPORTANT TO DETERMINE THIS NUMBER PROPERLY

    // Compute the features
    ne.compute (*cloud);

}


void PcMesher::estimateAllNormals(){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){

        estimateNormals(i);

    }
}

void PcMesher::planeSegmentation(){

    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[0];
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud_p (new PointCloud<PointXYZRGBNormalCam>);
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud_f (new PointCloud<PointXYZRGBNormalCam>);


    ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
    PointIndices::Ptr inliers (new PointIndices ());
    // Create the segmentation object
    SACSegmentation<PointXYZRGBNormalCam> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);


    // Create the filtering object
    ExtractIndices<PointXYZRGBNormalCam> extract;

    int i = 0, nr_points = (int) cloud->points.size ();
    // While 30% of the original cloud is still there
    while (cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << i << std::endl;

        std::stringstream ss;
        ss << "test_" << i << ".ply";
        io::savePLYFile(ss.str(), *cloud_p);

        // We create a pointer to a copy of the plane cloud to be able to store properly
        PointCloud<PointXYZRGBNormalCam>::Ptr plane_cloud = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(*cloud_p);

        pointClouds_.push_back(plane_cloud);
        nClouds_++;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }

}


void PcMesher::surfaceReconstruction(const unsigned int _index){

    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[_index];

    Poisson<PointXYZRGBNormalCam> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);

    std::stringstream ss;
    ss << "triangles_" << _index << ".ply";
    io::savePLYFile(ss.str(), mesh);

}



void PcMesher::readMesh(std::string _fileName){

    // This cloud is a temporal one which will be stored in the cloud vector
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud (new PointCloud<PointXYZRGBNormalCam>);

    // Cloud file is loaded
    if (io::loadPLYFile<PointXYZRGBNormalCam>(_fileName, *cloud) == -1){
        std::string message("Couldn't read file ");
        message.append(_fileName);
        message.append(" \n");
        PCL_ERROR(message.c_str());
        return;
    }

    pointClouds_.push_back(cloud);
    nClouds_ = 1;

}

void PcMesher::writeOneMesh(const unsigned int _index, std::string _fileName){

    PointCloud<PointXYZRGBNormalCam> outPointCloud = *pointClouds_[_index];

    io::savePLYFile(_fileName, outPointCloud);

}


// All the point clouds in the vector are concatenated and printed into one file
void PcMesher::writeMesh(std::string _fileName){

    PointCloud<PointXYZRGBNormalCam> outPointCloud;

    for (unsigned int i = 0; i < pointClouds_.size(); i++){
        outPointCloud += *pointClouds_[i];
    }

    io::savePLYFile(_fileName, outPointCloud);
}


void PcMesher::bundlerReader(std::string _filename){

    std::ifstream inputFile(_filename);
    std::string line;

    int nPoints = 0;
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud (new PointCloud<PointXYZRGBNormalCam>);

    if (inputFile.is_open()){

        // We avoid all possible commentaries in the firsts lines
        do {
            std::getline(inputFile, line);
        } while (line.at(0) == '#');

        // First, number of cameras and number of points in the input point cloud
        boost::tokenizer<> tokens(line);
        boost::tokenizer<>::iterator tit = tokens.begin();
        std::stringstream ss;
        ss << *tit;
        ss >> nCameras_;
        ++tit;
        // stringstream is cleared. It could also be ss.str("")
        ss.str(std::string());
        ss.clear();

        ss << *tit;
        ss >> nPoints;

        ss.str(std::string());
        ss.clear();

        // Now we read the camera information
        for (unsigned int i = 0; i < nCameras_; i++){
            Camera camera;
            Eigen::Matrix3f R;
            for (unsigned int cam_line = 0; cam_line < 5; cam_line++){
                std::getline(inputFile, line);

                boost::tokenizer<boost::char_separator<char> > cam_tokens(line, boost::char_separator<char>(" "));
                boost::tokenizer<boost::char_separator<char> >::iterator ctit = cam_tokens.begin();

                // First line has focal lenght and distortion coefficients
                if (cam_line == 0){
                    float f, k1, k2;
                    ss << *ctit;
                    ss >> f;
                    ++ctit;
                    ss.str(std::string());
                    ss.clear();
                    ss << *ctit;
                    ss >> k1;
                    ++ctit;
                    ss.str(std::string());
                    ss.clear();
                    ss << *ctit;
                    ss >> k2;
                    ss.str(std::string());
                    ss.clear();
                    camera.setFocalLenght(f);
                    camera.setDistortionCoefficients(k1,k2);
                }
                // Last line has the translation vector
                else if (cam_line == 4){
                    Eigen::Vector3f t;
                    unsigned int index = 0;
                    for (; ctit != cam_tokens.end(); ++ctit, index++){
                        float value;
                        ss << *ctit;
                        ss >> value;
                        ss.str(std::string());
                        ss.clear();
                        t(index) = value;
                    }
                    camera.setTranslationVector(t);
                }
                // The other lines have the rotation matrix
                else {
                    unsigned int index = 0;
                    for (; ctit != cam_tokens.end(); ++ctit, index++){
                        float value;
                        ss << *ctit;
                        ss >> value;
                        ss.str(std::string());
                        ss.clear();
                        R(cam_line-1, index) = value;
                    }
                }

            }
            std::cerr << R << std::endl;
            camera.setRotationMatrix(R);
            cameras_.push_back(camera);
        }

    } else {
        std::cerr << "Unable to open Bundle file" << std::endl;
    }



}



int main (int argc, char *argv[]){

    PcMesher cloud;

    cloud.bundlerReader(argv[1]);

//    cloud.readMesh(argv[1]);
//    cloud.writeMesh("input.ply");

//    cloud.planeSegmentation();
//    cloud.estimateAllNormals();
////    cloud.surfaceReconstruction(0);

//    for (unsigned int i = 0; i < cloud.getNClouds(); i++){

//        std::stringstream ss;
//        ss << "out_" << i << ".ply";
//        cloud.writeOneMesh(i, ss.str());

////        cloud.surfaceReconstruction(i);
//    }

//    cloud.writeMesh("output.ply");


    return 0;
}
