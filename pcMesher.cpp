#define PCL_NO_PRECOMPILE

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/conversions.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/gp3.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

#include <FreeImagePlus.h>

#include "pcMesher.h"

PcMesher::PcMesher(){

    pointClouds_.clear();
    nClouds_ = 0;

    cameras_.clear();
    nCameras_ = 0;

    imageList_.clear();
    camPerVtx_.clear();
}

PcMesher::~PcMesher(){

}

unsigned int PcMesher::getNClouds(){
    return nClouds_;
}

std::vector<std::vector<int> > PcMesher::getCamPerVtx(){
    return camPerVtx_;
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
    PointIndices outliers;

    StatisticalOutlierRemoval<PointXYZRGBNormalCam> sor(true); // Setting this to true we are able to extract indices of deleted points
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    // Indices to outliers
    sor.getRemovedIndices(outliers);
    removeOutliersFromCamPerVtx(outliers);

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
    ne.setRadiusSearch (0.3); // <--------------------- IT'S IMPORTANT TO DETERMINE THIS NUMBER PROPERLY


    // Compute the features
    ne.compute (*cloud);

}

void PcMesher::estimateNormals(const unsigned int _index, const float _radius){

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
    ne.setRadiusSearch (_radius);

    // Compute the features
    ne.compute (*cloud);

}


void PcMesher::estimateAllNormals(){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){

        estimateNormals(i);

    }
}

void PcMesher::estimateAllNormals(const float _radius){

    for (unsigned int i = 0; i < pointClouds_.size(); i++){

        estimateNormals(i, _radius);

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

void PcMesher::segmentPlanes(){

    std::cerr << "Segmenting planes" << std::endl;

    PointCloud<PointXYZRGBNormalCam>::Ptr cloud = pointClouds_[0];
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud_p (new PointCloud<PointXYZRGBNormalCam>);
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud_f (new PointCloud<PointXYZRGBNormalCam>);
    PointCloud<PointXYZRGBNormalCam>::Ptr tempPlaneCloud (new PointCloud<PointXYZRGBNormalCam>);
    PointCloud<PointXYZRGBNormalCam>::Ptr emptyCloud (new PointCloud<PointXYZRGBNormalCam>);

    ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
    PointIndices::Ptr inliers (new PointIndices ());
//    PointIndices lastOutliers;

    // Create the segmentation object
    SACSegmentation<PointXYZRGBNormalCam> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (1000);
//    seg.setDistanceThreshold(0.1);
    seg.setDistanceThreshold(0.03);


    // Create the filtering object
    ExtractIndices<PointXYZRGBNormalCam> extract(true); // set to true to be able to extract the outliers also

    int i = 0;
    const int nr_points = (int) cloud->points.size ();
    // To avoid an infinite loop
    const int max_iter = 10;
    int iter = 0;
    // While 5% of the original cloud is still there
    while (cloud->points.size () > 0.05 * nr_points) // 0.01
    {
        if (iter == max_iter) break;

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0){
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // We get the normal of the estimated plane
        Eigen::Vector3f planeNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        const int inliersOrignalSize = inliers->indices.size();
        // We check if every inlier orientation is the aprox. the same as the plane's
        for (unsigned int j = 0; j < inliersOrignalSize; j++){
            PointXYZRGBNormalCam current = cloud->points[inliers->indices[j]];
            Eigen::Vector3f cur_norm(current.normal_x, current.normal_y, current.normal_z);

            const float cos = planeNormal.dot(cur_norm) / (planeNormal.squaredNorm() * cur_norm.squaredNorm());
            // vectors with normals in an angle bigger than 45º are marked with -1 in the inlier list
            if ( fabs(cos) < 0.707){ // >45º
                inliers->indices[j] = -1;
            }
        }

        // inliers marked as -1 are removed from the list
        inliers->indices.erase(std::remove(inliers->indices.begin(), inliers->indices.end(), -1), inliers->indices.end());

        // if not enough points are left to determine a plane, we move to the following plane
        bool badplane = false;
        if (inliers->indices.size() < 0.008 * nr_points) {
            std::cerr << iter << std::endl;
            std::cerr << inliers->indices.size() << "/" << inliersOrignalSize << std::endl;
            iter++;
            badplane = true;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        // Indices to outliers
//        extract.getRemovedIndices(lastOutliers);

        if (!badplane){ // If we have a valid plane, we save it

            // We create a pointer to a copy of the plane cloud to be able to store properly
            PointCloud<PointXYZRGBNormalCam>::Ptr plane_cloud = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(*cloud_p);

            pointClouds_.push_back(plane_cloud);
            nClouds_++;

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);

            *pointClouds_[0] += *tempPlaneCloud;
            tempPlaneCloud = emptyCloud; // To clear the content of temPlaneCloud

            std::cerr << "PointCloud #" << i+1 << " representing the planar component: " << cloud_p->width * cloud_p->height << " data points. ";
            std::cerr << "Points reimaning: " << cloud->width * cloud->height << std::endl;


            // Write plane in ply file
            std::stringstream ss;
            ss << "out_" << i+1 << ".ply";
            std::cerr << "PointCloud #" << i+1 << " exported." << std::endl;
            io::savePLYFile(ss.str(), *plane_cloud);

            i++;
            iter = 0;

        } else { // In this case, the points are rearranged so we have a different result

            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);

            *pointClouds_[0] += *tempPlaneCloud;
            tempPlaneCloud = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(*cloud_p);

        }

    }

    std::cerr << "Exporting non-planar points" << std::endl;
    io::savePLYFile("outliers.ply", *pointClouds_[0]);

}


void PcMesher::segmentCylinders(){

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
    poisson.setDepth(10);
    poisson.setInputCloud(cloud);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);

    std::stringstream ss;
    ss << "triangles_" << _index << ".ply";
    io::savePLYFile(ss.str(), mesh);

}

PolygonMesh PcMesher::surfaceReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud){

    Poisson<PointXYZRGBNormalCam> poisson;
    poisson.setDepth(10);
    poisson.setInputCloud(_cloud);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);

    return mesh;

}

PolygonMesh PcMesher::greedyReconstruction(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud){

    PolygonMesh triangles;
    GreedyProjectionTriangulation<PointXYZRGBNormalCam> gp3;

    search::KdTree<PointXYZRGBNormalCam>::Ptr tree (new search::KdTree<PointXYZRGBNormalCam> ());

    gp3.setSearchRadius (0.006); // Kinect’s points are typically closer t
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (50);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setInputCloud (_cloud);
    gp3.setSearchMethod (tree);
    gp3.reconstruct (triangles);

    return triangles;

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

                    radius = sum_distance / static_cast<float>(K) * 3.0f; // 3.0f
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
    decimator.setInputMesh(meshPtr);
    decimator.setTargetReductionFactor(0.99f);
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

    for (unsigned int i = 0; i < cameras_.size(); i++){
//    for (unsigned int i = 0; i < cameraOrder_.size(); i++){

//        const int currentCam = cameraOrder_[i];
        const int currentCam = i;


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

void PcMesher::writeCameraSetupFile(std::string _fileName){

    std::cerr << "Writing camera setup file" << std::endl;

    std::ofstream outputFile(_fileName);

    outputFile << cameras_.size() << "\n";

    unsigned int width, height;

    for (unsigned int currentCam = 0; currentCam < cameras_.size(); currentCam++){

        getImageDimensions(imageList_[currentCam], height, width);

        int mid_width = width * 0.5;
        int mid_height = height * 0.5;

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
        // Camera position and image dimensions
        outputFile << p(0) << " " << p(1) << " " << p(2) << " " << width << " " << height << "\n";

    }

    outputFile.close();

}


void PcMesher::getImageDimensions(std::string _imageName, unsigned int &_height, unsigned int &_width){

    fipImage input;
    char * name = new char[_imageName.length()];
    strcpy(name, _imageName.c_str());

    if (!input.load(name, FIF_LOAD_NOPIXELS)){
        std::cerr << "Image couldn't be loaded!" << std::endl;
    }

    _height = input.getHeight();
    _width = input.getWidth();

//    std::cerr << "+" << _imageName << ": " << _width << " " << _height << std::endl;


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

void PcMesher::assignCam2Mesh(const PolygonMesh &_mesh, const PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, const std::string _fileName){

    std::ofstream outputFile(_fileName);

    KdTreeFLANN<PointXYZRGBNormalCam> kdtree;
    kdtree.setInputCloud(_cloud);

    // Obtaining a PointCloud from a PointCloud2
    PointCloud<PointXYZRGBNormalCam> meshCloud;
    fromPCLPointCloud2(_mesh.cloud, meshCloud);
    int nVtx = meshCloud.height * meshCloud.width;

    // for each vertex of the mesh
    for (unsigned int v = 0; v < nVtx; v++){

        PointXYZRGBNormalCam searchPoint = meshCloud.points[v];
        std::vector<int> pointIdxNKNSearch(1); // The closest. Just one.
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
            for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){ // This is stupid, it's just one iteration
                std::vector<int> current = camPerVtx_[pointIdxNKNSearch[i]];
                for (unsigned int j = 0; j < current.size(); j++){
                    outputFile << current[j] << " ";
                }
                outputFile << "\n";
            }
        }
    }

    outputFile.close();

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
        while (line.empty()) std::getline(_stream, line);

        boost::tokenizer<boost::char_separator<char> > point_tokens(line, boost::char_separator<char>(" "));
        boost::tokenizer<boost::char_separator<char> >::iterator ptit = point_tokens.begin();

        // This line has the position of the point
        if (point_line == 0){
            float xyz[3];
            for (unsigned int index = 0; ptit  != point_tokens.end(); ++ptit, index++){
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
            unsigned char rgb[3];
            for (unsigned int index = 0; ptit != point_tokens.end(); ++ptit, index++){
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

            std::vector<int> cam2vtx;

            if (nCam > 0){
                for (unsigned int index = 0; ptit != point_tokens.end(); ++ptit, index++){
                    if (index%4 != 0) continue; // every four values we have the number of the camera, we don't care about the other values.
                    int cam_value;
                    ss << *ptit;
                    ss >> cam_value;
                    ss.str(std::string());
                    ss.clear();
                    if (index == 0){
                        _point.camera = cam_value;
                    }
                    cam2vtx.push_back(cam_value);
                }
            } else {
                _point.camera = -1;
            }
            camPerVtx_.push_back(cam2vtx);
        }
    }
}


void PcMesher::bundlerReader(const std::string _fileName){

    std::cerr << "Reading Bundler file... ";

    std::ifstream inputFile(_fileName);
    std::string line;

    int nPoints = 0;
    PointCloud<PointXYZRGBNormalCam>::Ptr cloud (new PointCloud<PointXYZRGBNormalCam>);

    if (inputFile.is_open()){

        // We avoid all possible comments in the firsts lines
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

        std::cerr << "done!: " << nPoints << " points read." << std::endl;


    } else {
        std::cerr << "Unable to open Bundle file" << std::endl;
    }

}

void PcMesher::readImageList(const std::string _fileName){

    std::cerr << "Reading image list file" << std::endl;

    imageList_.clear();

    std::ifstream inputFile(_fileName);
    std::string line;

    if (inputFile.is_open()){

        while(!inputFile.eof()){
            std::getline(inputFile, line);
            if (line.empty()) continue;
            imageList_.push_back(line);
        }

    } else {
        std::cerr << "Unable to open image list file" << std::endl;
        exit(-1);
    }

    inputFile.close();

}

void PcMesher::exportIndices(PointIndices& _indices, const std::string _fileName){

    std::cerr << "Exporting a txt file with points not included in any plane" << std::endl;

    std::ofstream outputFile(_fileName);

    for (unsigned int i = 0; i < _indices.indices.size(); i++){
        outputFile << _indices.indices[i] << "\n";
    }


    outputFile.close();
}


void PcMesher::exportCamPerVtx(const std::string _fileName){

    std::ofstream outputFile(_fileName);

    for (unsigned int i = 0; i < camPerVtx_.size(); i++){
        const std::vector<int> current = camPerVtx_[i];
        for (unsigned int j = 0; j < current.size(); j++){
            outputFile << current[j] << " ";
        }
        outputFile << "\n";
    }

    outputFile.close();

}



void PcMesher::removeOutliersFromCamPerVtx(PointIndices &_indices){

    // Where there is an outlier, we set an empty vector
    for (unsigned int i = 0; i < _indices.indices.size(); i++){
        camPerVtx_[_indices.indices[i]].clear();
    }

    // std::remove efficiently removes every entry where there is an empty vector
    camPerVtx_.erase(std::remove(camPerVtx_.begin(), camPerVtx_.end(), std::vector<int>(0)), camPerVtx_.end());

}



