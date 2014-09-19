#define PCL_NO_PRECOMPILE

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/conversions.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

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

PointCloud<PointXYZRGBNormalCam>::Ptr PcMesher::getPointCloudPtr(unsigned int _index){
    return pointClouds_[_index];
}

void PcMesher::removeOutliers(PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud){

    StatisticalOutlierRemoval<PointXYZRGBNormalCam> sor;
    sor.setInputCloud(_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*_cloud);

}

void PcMesher::removeOutliers(unsigned int _index){

    std::cerr << "Removing outliers of pointcloud " << _index + 1 << "/" << nClouds_ << std::endl;

    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[_index];

    StatisticalOutlierRemoval<PointXYZRGBNormalCam> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

}

void PcMesher::removeAllOutliers(){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){
        this->removeOutliers(i);
    }

}



void PcMesher::estimateNormals(const unsigned int _index){

    std::cerr << "Estimating normals of pointcloud " << _index + 1 << "/" << nClouds_ << std::endl;

    // Create the normal estimation class, and pass the input dataset to it
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[_index];
    NormalEstimation<PointXYZRGBNormalCam, PointXYZRGBNormalCam> ne;

    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    search::KdTree<PointXYZRGBNormalCam>::Ptr tree (new search::KdTree<PointXYZRGBNormalCam> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
//    ne.setRadiusSearch (1.03); // <--------------------- IT'S IMPORTANT TO DETERMINE THIS NUMBER PROPERLY
    ne.setRadiusSearch (0.3); // <--------------------- IT'S IMPORTANT TO DETERMINE THIS NUMBER PROPERLY


    // Compute the features
    ne.compute (*cloud);

}


void PcMesher::estimateAllNormals(){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){

        estimateNormals(i);

    }
}

void PcMesher::fixNormal(const unsigned int _index){

    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[_index];

    for (unsigned int i = 0; i < cloud->points.size(); i++){
        PointXYZRGBNormalCam current = cloud->points[i];
        Eigen::Vector3f cur_pos = current.getArray3fMap();
        Eigen::Vector3f cam_pos = cameras_[current.camera].getCameraPosition();

        Eigen::Vector3f cam_dir = cam_pos - cur_pos;
        Eigen::Vector3f normal  = current.getNormalVector3fMap();

        if (cam_dir.dot(normal) < 0){
            cloud->points[i].normal_x = -current.normal_x;
            cloud->points[i].normal_y = -current.normal_y;
            cloud->points[i].normal_z = -current.normal_z;
        }
    }
}

void PcMesher::fixAllNormals(){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){

        fixNormal(i);

    }
}

void PcMesher::planeSegmentation(){

    std::cerr << "Segmenting planes..." << std::endl;

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
//    seg.setDistanceThreshold (0.01);
    seg.setDistanceThreshold(0.05);


    // Create the filtering object
    ExtractIndices<PointXYZRGBNormalCam> extract;

    int i = 0, nr_points = (int) cloud->points.size ();
    // While 5% of the original cloud is still there
    while (cloud->points.size () > 0.05 * nr_points)
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

void PcMesher::cylinderSegmentation(){

    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[0];
//    PointCloud<Normal>::Ptr normal_cloud (new PointCloud<Normal>);

//    Normal n (cloud->points[0].normal_x, cloud->points[0].normal_y, cloud->points[0].normal_z);

    SACSegmentationFromNormals<PointXYZRGBNormalCam, PointXYZRGBNormalCam> seg;
    ExtractIndices<PointXYZRGBNormalCam> extract;

    ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
    PointIndices::Ptr inliers (new PointIndices ());

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_CYLINDER);
    seg.setMethodType (SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud);


    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, *coefficients);
    std::cerr << "Cylinder coefficients: " << *coefficients << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    pcl::PointCloud<PointXYZRGBNormalCam>::Ptr cloud_cylinder (new pcl::PointCloud<PointXYZRGBNormalCam> ());
    // We create a pointer to a copy of the plane cloud to be able to store properly
    PointCloud<PointXYZRGBNormalCam>::Ptr cyl_cloud = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(*cloud_cylinder);

    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ()) {
      std::cerr << "Can't find the cylindrical component." << std::endl;
    } else {
        pointClouds_.push_back(cyl_cloud);
        nClouds_++;
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

PolygonMesh PcMesher::surfaceReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud){

    Poisson<PointXYZRGBNormalCam> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(_cloud);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);

    return mesh;

}

PolygonMesh PcMesher::deleteWrongVertices(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, PolygonMesh _inputMesh){

    std::cerr << "Cleaning the Poisson mesh" << std::endl;

    // This value needs to be automatized
    float MAXD = 0.1;

    KdTreeFLANN<PointXYZRGBNormalCam> kdtree;
    kdtree.setInputCloud(_cloud);

    // Some data for the output mesh:
    // vector for storing valid polygons
    std::vector<Vertices> validFaces;
    // PointCloud instead of PCLPointCloud2
    PointCloud<PointXYZRGBNormalCam> meshCloud;
    fromPCLPointCloud2 (_inputMesh.cloud, meshCloud);

    // foreach face
    std::vector<Vertices, std::allocator<Vertices> >::iterator face_it; // por qué el allocator?
    for (face_it = _inputMesh.polygons.begin(); face_it != _inputMesh.polygons.end(); ++face_it) {

        bool isInside = true;

        // for each vertex in the face
        for (unsigned int i = 0; i < 3; i++){

            PointXYZRGBNormalCam searchPoint = meshCloud.points[face_it->vertices[i]];

            // We first search for the closest point in the original point cloud to the searchPoint
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            float radius;

            if ( kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
                for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){ // It's just one iteration

                    // Now we search for the K closest points to determine the point cloud density
                    PointXYZRGBNormalCam anchorPoint = _cloud->points[pointIdxNKNSearch[i]];

                    int K = 10;

                    std::vector<int> pointIdx(K);
                    std::vector<float> pointSquaredDistance(K);

                    float sum_distance = 0.0;

                    if (kdtree.nearestKSearch(anchorPoint, K, pointIdx, pointSquaredDistance) > 0){
                        for (int j = 0; j < pointIdx.size(); ++j){
                            sum_distance += sqrt(pointSquaredDistance[j]);
                        }
                    }

                    radius = sum_distance / static_cast<float>(K) * 3.0f;
                }
            }


            // Neighbors within radius search

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

//            float radius = MAXD;

            // if there is no vertex in the input point cloud close to this current vertex of the mesh
            // the whole face is discarded
            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0 ){
                isInside = false;
                break;
            }
        }

        if (isInside) validFaces.push_back(*face_it);
    }


    // Write output polygon mesh
    PolygonMesh outputMesh;
    // Same point cloud as the input mesh
    outputMesh.cloud = _inputMesh.cloud;
    // Only the valid faces
    outputMesh.polygons.clear();
    outputMesh.polygons.insert(outputMesh.polygons.begin(), validFaces.begin(), validFaces.end());

    // Here we delete unused vertices
    PolygonMesh finalOutputMesh;
    surface::SimplificationRemoveUnusedVertices cleaner;
    cleaner.simplify(outputMesh, finalOutputMesh);

    return finalOutputMesh;

}

PolygonMesh PcMesher::decimateMesh(const PolygonMesh& _mesh){

    std::cerr << "Decimating mesh" << std::endl;

    PolygonMesh::Ptr meshPtr = boost::make_shared<PolygonMesh>(_mesh);

    PolygonMesh outputMesh;
    MeshQuadricDecimationVTK decimator;
    decimator.setTargetReductionFactor(0.5);
    decimator.setInputMesh(meshPtr);
    decimator.process(outputMesh);

    return outputMesh;
}

void PcMesher::drawCameras(){

    PointCloud<PointXYZRGBNormalCam>::Ptr camera_cloud (new PointCloud<PointXYZRGBNormalCam>);

    for (unsigned int i = 0; i < cameras_.size(); i++){
        PointXYZRGBNormalCam pos;
        Camera currentCam = cameras_[i];
        Eigen::Vector3f eigpos = currentCam.getCameraPosition();
        pos.x = eigpos(0);
        pos.y = eigpos(1);
        pos.z = eigpos(2);
        // They are painted green, for example
        pos.r = 0;
        pos.g = 255;
        pos.b = 0;
        camera_cloud->push_back(pos);
    }

    pointClouds_.push_back(camera_cloud);
    nClouds_++;

}

void PcMesher::writeCameraSetupFile(std::string _fileName, const int _width, const int _height){

    std::cerr << "Writing camera setup file" << std::endl;

    std::ofstream outputFile(_fileName);

    outputFile << cameras_.size() << " " << _width << " " << _height << "\n";

//    for (unsigned int i = 0; i < cameras_.size(); i++){
    for (unsigned int i = 0; i < cameraOrder_.size(); i++){

        const int currentCam = cameraOrder_[i];

        int mid_width = _width * 0.5;
        int mid_height = _height * 0.5;

        // Intrinsic parameteres
        outputFile << cameras_[currentCam].getFocalLength() << " 0 " << mid_width << " ";
        outputFile << "0 " << cameras_[currentCam].getFocalLength() << " " << mid_height << " ";
        outputFile << "0 0 1 ";

        Eigen::Matrix3f R = cameras_[currentCam].getRotationMatrix();

        // Extrinsic parameters
        outputFile << R(0,0) << " " << R(0,1) << " " << R(0,2) << " ";
        outputFile << -R(1,0) << " " << -R(1,1) << " " << -R(1,2) << " ";
        outputFile << -R(2,0) << " " << -R(2,1) << " " << -R(2,2) << " ";


        Eigen::Vector3f p = cameras_[currentCam].getCameraPosition();
        // Camera position
        outputFile << p(0) << " " << p(1) << " " << p(2) << "\n";

    }

    outputFile.close();

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

    std::cerr << "Exporting point cloud: " << _index + 1 << "/" << nClouds_ << std::endl;

    PointCloud<PointXYZRGBNormalCam> outPointCloud = *pointClouds_[_index];

    io::savePLYFile(_fileName, outPointCloud);

}

PointCloud<PointXYZRGBNormalCam> PcMesher::combinePointClouds(){

    PointCloud<PointXYZRGBNormalCam> outPointCloud;

    for (unsigned int i = 0; i < pointClouds_.size(); i++){
        outPointCloud += *pointClouds_[i];
    }

    return outPointCloud;

}

PointCloud<PointXYZRGBNormalCam> PcMesher::combinePointClouds(std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr> _pointclouds){

    PointCloud<PointXYZRGBNormalCam> outPointCloud;

    for (unsigned int i = 0; i < _pointclouds.size(); i++){
        outPointCloud += *_pointclouds[i];
    }

    return outPointCloud;

}


// All the point clouds in the vector are concatenated and printed into one file
void PcMesher::writeMesh(std::string _fileName){

    io::savePLYFile(_fileName, combinePointClouds());

}

void PcMesher::bundlerPointReader(PointXYZRGBNormalCam &_point, std::ifstream &_stream){

    std::stringstream ss;
    std::string line;

    for (unsigned int point_line = 0; point_line < 3; point_line++){

        std::getline(_stream, line);

        boost::tokenizer<boost::char_separator<char> > point_tokens(line, boost::char_separator<char>(" "));
        boost::tokenizer<boost::char_separator<char> >::iterator ptit = point_tokens.begin();

        // This line has the position of the point
        if (point_line == 0){
            unsigned int index = 0;
            float xyz[3];
            for (; ptit  != point_tokens.end(); ++ptit, index++){
                float value;
                ss << *ptit;
                ss >> value;
                ss.str(std::string());
                ss.clear();
                xyz[index] = value;
            }
            _point.x = xyz[0];
            _point.y = xyz[1];
            _point.z = xyz[2];
        }
        // This line has the color of the point
        else if (point_line == 1){
            unsigned int index = 0;
            unsigned char rgb[3];
            for (; ptit != point_tokens.end(); ++ptit, index++){
                float value;
                ss << *ptit;
                ss >> value;
                ss.str(std::string());
                ss.clear();
                rgb[index] = value;
            }
            _point.r = rgb[0];
            _point.g = rgb[1];
            _point.b = rgb[2];
        }

        // This line sets the camera correspondances with each point in the cloud
        else {
            unsigned int nCam;
            ss << *ptit;
            ss >> nCam;
            ss.str(std::string());
            ss.clear();
            ++ptit;

            if (nCam > 0){
                int cam_value;
                ss << *ptit;
                ss >> cam_value;
                ss.str(std::string());
                ss.clear();
                _point.camera = cam_value;
            } else {
                _point.camera = -1;
            }
        }
    }
}


void PcMesher::bundlerReader(std::string _fileName){

    std::cerr << "Reading Bundler file" << std::endl;

    std::ifstream inputFile(_fileName);
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
            camera.readCamera(inputFile);

            cameras_.push_back(camera);
        }

        // Now we read geometry information
        for (unsigned int i = 0; i < nPoints; i++){

            PointXYZRGBNormalCam point;
            bundlerPointReader(point, inputFile);

            cloud->push_back(point);

        }

        pointClouds_.push_back(cloud);
        nClouds_++;

        inputFile.close();

    } else {
        std::cerr << "Unable to open Bundle file" << std::endl;
    }

}

void PcMesher::nvmCameraReader(std::string _fileName){

    std::cerr << "Reading nvm file" << std::endl;

    cameraOrder_.resize(nCameras_);

    std::ifstream inputFile(_fileName);
    std::string line;

    if (inputFile.is_open()){

        do {
            std::getline(inputFile, line);
            if (line.empty()) std::getline(inputFile, line);
        } while (line.at(0) != 'i');

        boost::tokenizer<> tokens(line);
        boost::tokenizer<>::iterator tit = tokens.begin();
        std::stringstream ss;
        tit++;
        ss << *tit;
        int ncam;
        ss >> ncam;

        cameraOrder_[ncam-1] = 0; // 1 - 1

        for (unsigned int i = 1; i < nCameras_; i++){
            std::getline(inputFile, line);
            boost::tokenizer<> moreTokens(line);
            boost::tokenizer<>::iterator mtit = moreTokens.begin();
            mtit++;
            std::stringstream nss;
            nss << *mtit;
            nss >> ncam;

            cameraOrder_[ncam-1] = i;

        }

        inputFile.close();


    } else {
        std::cerr << "Unable to open Bundle file" << std::endl;
    }
}



int main (int argc, char *argv[]){

    PcMesher cloud;

    cloud.bundlerReader(argv[1]);
    cloud.nvmCameraReader(argv[2]);
    cloud.writeCameraSetupFile("cameras.txt", 2868, 4310);
    cloud.writeMesh("input.ply");

    cloud.removeAllOutliers();

//    cloud.cylinderSegmentation();

    cloud.planeSegmentation();
    cloud.estimateAllNormals();
    cloud.fixAllNormals();


    for (unsigned int i = 0; i < cloud.getNClouds(); i++){

        std::stringstream ss;
        ss << "out_" << i << ".ply";
        cloud.writeOneMesh(i, ss.str());

    }

    //--- I collect the pointclouds that are planes into one single pointcloud
    std::vector<PointCloud<PointXYZRGBNormalCam>::Ptr> pointclouds;
    for (unsigned int i = 1; i < cloud.getNClouds(); i++){
        pointclouds.push_back(cloud.getPointCloudPtr(i));
    }
    //---


//    PointCloud<PointXYZRGBNormalCam> combinedCloud = cloud.combinePointClouds();
    PointCloud<PointXYZRGBNormalCam> combinedCloud = cloud.combinePointClouds(pointclouds);
    PointCloud<PointXYZRGBNormalCam>::Ptr combinedCloudPtr = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(combinedCloud);

    PolygonMesh first_mesh = cloud.surfaceReconstruction(combinedCloudPtr);

    PolygonMesh m = cloud.deleteWrongVertices(combinedCloudPtr, first_mesh);
    PolygonMesh simpleM = cloud.decimateMesh(m);

    io::savePLYFile("limpio.ply", m);
    io::savePLYFile("limpio_decimated.ply", simpleM);


//    cloud.drawCameras();

    cloud.writeMesh("output.ply");

    return 0;
}
