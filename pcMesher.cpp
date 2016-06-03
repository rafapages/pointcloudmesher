#define PCL_NO_PRECOMPILE

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>

#include <pcl/geometry/mesh_conversion.h>
#include <pcl/geometry/mesh_io.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/get_boundary.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>


#include <FreeImagePlus.h>

#include <stdlib.h>     /* strtol */

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

unsigned int PcMesher::getNClouds() const {
    return nClouds_;
}

std::vector<std::vector<int> > PcMesher::getCamPerVtx() const {
    return camPerVtx_;
}

PointCloud<PointXYZRGBNormalCam>::Ptr PcMesher::getPointCloudPtr(unsigned int _index){
    return pointClouds_[_index];
}

void PcMesher::clearPointClouds(){
    pointClouds_.clear();
}

Eigen::Vector3f PcMesher::getDimensions(const PointCloud<PointXYZRGBNormalCam>::Ptr &_cloud){

    std::cerr << "Estimating dimensions of point cloud" << std::endl;

    float minx, miny, minz, maxx, maxy, maxz;
    minx = miny = minz = FLT_MAX;
    maxx = maxy = maxz = FLT_MIN;

    for (unsigned int i = 0; i < _cloud->points.size(); i++){
        PointXYZRGBNormalCam current = _cloud->points[i];
        if (current.x < minx){
            minx = current.x;
        }
        if (current.y < miny){
            miny = current.y;
        }
        if (current.z < minz){
            minz = current.z;
        }
        if (current.x > maxx){
            maxx = current.x;
        }
        if (current.y > maxy){
            maxy = current.y;
        }
        if (current.z > maxz){
            maxz = current.z;
        }
    }

    const float dimx = fabs(maxx - minx);
    const float dimy = fabs(maxy - miny);
    const float dimz = fabs(maxz - minz);

    return Eigen::Vector3f(dimx, dimy, dimz);

}

Eigen::Vector3f PcMesher::getDimensions(const unsigned int _index){
    PointCloud<PointXYZRGBNormalCam>::Ptr pointcloud = pointClouds_[_index];
    return this->getDimensions(pointcloud);
}

Eigen::Vector3f PcMesher::getDimensions(const PolygonMesh &_mesh){

    PointCloud<PointXYZRGBNormalCam> meshCloud;
    fromPCLPointCloud2 (_mesh.cloud, meshCloud);
    PointCloud<PointXYZRGBNormalCam>::Ptr meshCloudPtr = boost::make_shared<PointCloud<PointXYZRGBNormalCam> >(meshCloud);
    Eigen::Vector3f dims = this->getDimensions(meshCloudPtr);

    return dims;

}

double PcMesher::computeResolution(const PointCloud<PointXYZRGBNormalCam>::Ptr &_cloud) const {

    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointXYZRGBNormalCam> tree;

    tree.setInputCloud (_cloud);

    for (size_t i = 0; i < _cloud->size (); ++i){
        if (! pcl_isfinite ((*_cloud)[i].x)){
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2){
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }

    if (n_points != 0){
        res /= n_points;
    }
    return res;

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
//    sor.setStddevMulThresh(0.5);
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

void PcMesher::downSample(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud, PointCloud<PointXYZRGBNormalCam>::Ptr _outCloud){

    pcl::PCLPointCloud2::Ptr outcloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr incloud (new pcl::PCLPointCloud2 ());

    toPCLPointCloud2(*_cloud, *incloud);

    double res = computeResolution(_cloud);

    std::cerr << "PointCloud before filtering: " << _cloud->width * _cloud->height << std::endl;
    std::cerr << "Downsampling point cloud..." << std::endl;

    // Create the filtering object
    VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (incloud);
//    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.setLeafSize (10*res, 10*res, 10*res);
    sor.filter (*outcloud);

    // PointCloud instead of PCLPointCloud2
    fromPCLPointCloud2 (*outcloud, *_outCloud);

    std::cerr << "PointCloud after filtering: " << _outCloud->width * _outCloud->height << std::endl;


}

void PcMesher::getPlaneDefinedByCameras(PointXYZRGBNormalCam& _normal) const{

    std::vector<Eigen::Vector3f> cam_pos;
    for (unsigned int i = 0; i < cameras_.size(); i++){
        const Camera c = cameras_[i];
        cam_pos.push_back(c.getCameraPosition());
    }

    Eigen::Vector3f normal;
    fitPlane(cam_pos, normal);

}

void PcMesher::fitPlane(const std::vector<Eigen::Vector3f> _cloud, Eigen::Vector3f& _normal) const{

    // http://stackoverflow.com/questions/1400213/3d-least-squares-plane
    // http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points

    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    float xx, xy, yy, xz, yz, x, y, z;
    xx = xy = yy = xz = yz = x = y = z = 0.0;

    for (unsigned int i = 0; i < _cloud.size(); i++){
        const Eigen::Vector3f current = _cloud[i];
        xx += current(0) * current(0);
        xy += current(0) * current(1);
        yy += current(1) * current(1);
        xz += current(0) * current(2);
        yz += current(1) * current(2);
        x = current(0);
        y = current(1);
        z = current(2);
    }

    A << xx, xy, x,
         xy, yy, y,
         x,   y, (float) _cloud.size();

    b << xz, yz, z;

    std::cerr << "A\n" << A << std::endl;
    std::cerr << "b\n" << b << std::endl;

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

    // Use all neighbors in a sphere of radius <_radius>
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

void PcMesher::segmentPlanes(float _threshold){

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
    seg.setDistanceThreshold(_threshold);
//    seg.setDistanceThreshold(0.03);


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
        if (inliers->indices.size() < 0.01 * nr_points) {
//        if (inliers->indices.size() < 0.005 * nr_points) {
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

    SACSegmentationFromNormals<PointXYZRGBNormalCam, PointXYZRGBNormalCam> seg;
    ExtractIndices<PointXYZRGBNormalCam> extract;

    ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
    PointIndices::Ptr inliers (new PointIndices ());

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_CYLINDER);
    seg.setMethodType (SAC_RANSAC);
//    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.1);
    seg.setRadiusLimits (0.5, 2);
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
        io::savePLYFile("cilindro.ply", *cloud_cylinder);
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



PolygonMesh PcMesher::deleteWrongVertices(PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, PolygonMesh& _inputMesh){

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
                for (size_t ind = 0; ind < pointIdxNKNSearch.size(); ++ind){ // It's just one iteration

                    // Now we search for the K closest points to determine the point cloud density
                    PointXYZRGBNormalCam anchorPoint = _cloud->points[pointIdxNKNSearch[ind]];

                    int K = 10;

                    std::vector<int> pointIdx(K);
                    std::vector<float> pointSquaredDistance(K);

                    float sum_distance = 0.0;


                    if (kdtree.nearestKSearch(anchorPoint, K, pointIdx, pointSquaredDistance) > 0){
                        for (int j = 0; j < pointIdx.size(); ++j){
                            sum_distance += sqrt(pointSquaredDistance[j]);
                        }
                    }

                    radius = sum_distance / static_cast<float>(K) * 10.0f; // 3.0f: the bigger this number, the lower number of holes
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

PolygonMesh PcMesher::decimateMesh(const PolygonMesh& _mesh, float _reduction){

    // _reduction goes from 0 to 1

    std::cerr << "Decimating mesh" << std::endl;

    PolygonMesh::Ptr meshPtr = boost::make_shared<PolygonMesh>(_mesh);

    PolygonMesh outputMesh;
    MeshQuadricDecimationVTK decimator;
    decimator.setInputMesh(meshPtr);
//    decimator.setTargetReductionFactor(0.99f);
    decimator.setTargetReductionFactor(1 - _reduction);
    decimator.process(outputMesh);

    return outputMesh;
}

PolygonMesh PcMesher::smoothMeshLaplacian(const PolygonMesh &_mesh){

    std::cerr << "Applying Laplacian filtering to mesh" << std::endl;

    PolygonMesh::Ptr meshPtr = boost::make_shared<PolygonMesh>(_mesh);

    PolygonMesh outputMesh;
    MeshSmoothingLaplacianVTK smoother;
    smoother.setInputMesh(meshPtr);
    smoother.setNumIter(10);
    smoother.setRelaxationFactor(0.5);
    smoother.process(outputMesh);

    return outputMesh;

}

void PcMesher::detectLargestComponent(Mesh &_inputMesh) const {

    std::vector<bool> visited(_inputMesh.sizeVertices(), false);
    std::vector<std::vector<Mesh::VertexIndex> > components;

    unsigned int nComponents = 0;
    Mesh::VertexIndex vi;

    while (true){

        //find an unvisited vertex
        bool found = false;
        for (unsigned int i = 0; i < _inputMesh.sizeVertices(); i++){
            if(!visited[i]){
                found = true;
                visited[i] = true;
                vi = Mesh::VertexIndex(i);
                break;
            }
        }

        //if none was found -> finished
        if (!found) break;
        nComponents++;

        std::vector<Mesh::VertexIndex> vindices;
        std::vector<Mesh::VertexIndex> componentIndices;
        vindices.push_back(vi);

        while(vindices.size() > 0){
            Mesh::VertexIndex current = vindices.back();
            vindices.pop_back();

            Mesh::VertexAroundVertexCirculator vac = _inputMesh.getVertexAroundVertexCirculator(current);
            const Mesh::VertexAroundVertexCirculator vac_end = vac;

            do {
                if (!visited[vac.getTargetIndex().get()]){
                    visited[vac.getTargetIndex().get()] = true;
                    vindices.push_back(vac.getTargetIndex());
                    componentIndices.push_back(vac.getTargetIndex());
                }

            } while (++vac != vac_end);

        }

        components.push_back(componentIndices);

    }

    std::cerr << "Number of mesh components: " << nComponents << std::endl;

    // In case there is more than one component, we just keep the largest
    if (components.size() != 1){

        int maxIndex = -1;
        unsigned int maxNumber = 0;
        for (unsigned int i = 0; i < components.size(); i++){
            if (components[i].size() > maxNumber){
                maxIndex = i;
                maxNumber = components[i].size();
            }
        }
        std::cerr << "Largest component is number " << maxIndex << " with " << maxNumber << " elements" << std::endl;

        for (unsigned int i = 0; i < components.size(); i++){
            // Vertices which don't belong to largest component are deleted
            if (i != maxIndex){
                const std::vector<Mesh::VertexIndex> curr_comp = components[i];
                for (unsigned int j = 0; j < curr_comp.size(); j++){
                    const Mesh::VertexIndex curr_vi = curr_comp[j];
                    _inputMesh.deleteVertex(curr_vi);
                }
            }
        }
    }

    _inputMesh.cleanUp();

}


bool PcMesher::isMeshOpen(const Mesh& _inputMesh) const {


    bool open = false;
    for (unsigned int i = 0;  i < _inputMesh.sizeVertices(); i++){
        if (_inputMesh.isBoundary(Mesh::VertexIndex(i))){
            open = true;
            break;
        }
    }

    return open;
}

void PcMesher::openHole(Mesh &_inputMesh) const {

    if (isMeshOpen(_inputMesh)){
        std::cerr << "Mesh is already open!" << std::endl;
        return;
    }

    float maxArea = FLT_MIN;
    Mesh::FaceIndex bigface;
    for (unsigned int i = 0; i < _inputMesh.sizeFaces(); i++){
        Mesh::FaceIndex fidx = Mesh::FaceIndex(i);

        Mesh::VertexAroundFaceCirculator        circ = _inputMesh.getVertexAroundFaceCirculator(fidx);
        const Mesh::VertexAroundFaceCirculator  circ_end = circ;

        std::vector<Eigen::Vector3f> vertices;
        do {
            const PointXYZRGBNormalCam current = _inputMesh.getVertexDataCloud()[circ.getTargetIndex().get()];
            vertices.push_back(current.getArray3fMap());
        } while (++circ != circ_end);

        Eigen::Vector3f v1, v2;
        v1 = vertices[1]-vertices[0];
        v2 = vertices[2]-vertices[0];

        Eigen::Vector3f areav = v1.cross(v2);
        float area = areav.norm();

        if (area > maxArea){
            maxArea = area;
            bigface = fidx;
        }
    }

    _inputMesh.deleteFace(bigface);
    _inputMesh.cleanUp();

    pcl::PolygonMesh out;
    pcl::geometry::toFaceVertexMesh(_inputMesh, out);
    io::savePLYFile("prueba3.ply", out);

}


void PcMesher::cleanOpenMesh(const PointCloud<PointXYZRGBNormalCam>::Ptr& _cloud, Mesh &_inputMesh) const {

    if (!isMeshOpen(_inputMesh)){
        std::cerr << "This mesh is not open!" << std::endl;
        return;
    }

    // we acquire a set of boundary halfedges
    std::vector<Mesh::HalfEdgeIndices> boundary;
    pcl::geometry::getBoundBoundaryHalfEdges(_inputMesh, boundary);

    for (unsigned int i = 0; i < boundary.size(); i++){
        Mesh::HalfEdgeIndices b = boundary[i];
        std::cerr << "Part " << i << " has " << b.size() << " boundary helfedges" << std::endl;
    }

    // Save the original boundary sizes
    std::vector<int> nBounds;
    int nParts = boundary.size();
    for (unsigned int i = 0; i < boundary.size(); i++){
        nBounds.push_back(boundary[i].size());
    }

    // Estimate the optimal search radius
    double resolution = computeResolution(_cloud);
//    float radius = 10*resolution;

    float radius = 5*resolution;

    // Search

    bool far = true;
    int it = 0;
   // std::set<Mesh::HalfEdgeIndex> he_dictionary;

    while (far){
        far = false;


        for (unsigned int i = 0; i < boundary.size(); i++){
            Mesh::HalfEdgeIndices b = boundary[i];
            for (unsigned int j = 0; j < b.size(); j++){
                const Mesh::HalfEdgeIndex hei = b[j];


                const Mesh::VertexIndex vei = _inputMesh.getOriginatingVertexIndex(hei);
                if (!vei.isValid()) continue;
                const PointXYZRGBNormalCam current = _inputMesh.getVertexDataCloud()[vei.get()];
                const Mesh::FaceIndex face = _inputMesh.getOppositeFaceIndex(hei);
                //std::cerr << i << "/" << j << std::endl;
                if (!isPointCloseToPointCloud(current, _cloud, radius) && face.isValid()){
                    // We remove all faces containing the current vertex
                    Mesh::FaceAroundVertexCirculator fav = _inputMesh.getFaceAroundVertexCirculator(_inputMesh.getOriginatingVertexIndex(hei));
                    const Mesh::FaceAroundVertexCirculator fav_end = fav;
                    std::vector<Mesh::FaceIndex> toDelete;
                    do {
                        Mesh::FaceIndex fav_face = fav.getTargetIndex();
                        toDelete.push_back(fav_face);
                    } while (++fav != fav_end);

                    for (unsigned int j = 0; j < toDelete.size(); j++){
                        if (toDelete[j].isValid()){
                            _inputMesh.deleteFace(toDelete[j]);
                        }
                    }

                    _inputMesh.deleteFace(face);
                    far = true;
                }
            }
        }


        _inputMesh.cleanUp();
        boundary.clear();
        pcl::geometry::getBoundBoundaryHalfEdges(_inputMesh, boundary);

        // To avoid extra calculations, if a boundary has not changed, it is not
        // iterated again. We keep track of the number of halfedges in each iteration,
        // so if this number hasn't changed, we don't analyze the boundary again.
        //
        if (nParts != boundary.size()){
            // Update nBounds and nParts
            nParts = boundary.size();
            for (unsigned int i = 0; i < boundary.size(); i++){
                nBounds.push_back(boundary[i].size());
            }
        } else {
            std::vector<Mesh::HalfEdgeIndices>::iterator itt = boundary.end()-1;
            for (int i = nParts-1; i >= 0; i--){ // starting from the end helps optimizing deletion
                if (nBounds[i] == boundary[i].size()){
                    boundary.erase(itt);
                } else {
                    // The value in nBounds is updated
                    nBounds[i] = boundary[i].size();
                }
                if (itt != boundary.begin()) // just in case...
                    itt--;
            }
        }


        // TEST: just to see the effect ------------
        for (unsigned int i = 0; i < nBounds.size(); i++){
            std::cerr << nBounds[i] << " ";
        }
        std::cerr << "\n";

        for (unsigned int i = 0; i < boundary.size(); i++){
            std::cerr << "Part " << i << " has " << boundary[i].size() << " boundary helfedges" << std::endl;
        }

        it++;
//        pcl::PolygonMesh test;
//        pcl::geometry::toFaceVertexMesh(_inputMesh, test);
//        std::stringstream ss;
//        ss << it;
//        ss << ".ply";
//        io::savePLYFile(ss.str().c_str(),  test);
        // ----------------------------------------

    }

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

    std::ofstream outputFile(_fileName.c_str());

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

    std::ofstream outputFile(_fileName.c_str());

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



void PcMesher::readPLYCloud(const std::string& _fileName){

    // This cloud is a temporal one which will be stored in the cloud vector
    pointClouds_.clear();
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

void PcMesher::writeOneCloud(const unsigned int _index, std::string _fileName){

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

  std::ofstream outputFile(_fileName.c_str());

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
void PcMesher::writeCloud(std::string _fileName){

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

    std::ifstream inputFile(_fileName.c_str());
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

    std::ifstream inputFile(_fileName.c_str());
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

void PcMesher::readPLYMesh(const std::string _fileName, PolygonMesh &_mesh){

    // Cloud file is loaded
    if (io::loadPolygonFilePLY(_fileName, _mesh) == -1){
        std::string message("Couldn't read file ");
        message.append(_fileName);
        message.append(" \n");
        PCL_ERROR(message.c_str());
        return;
    }
}

void PcMesher::readOBJMesh(const std::string _fileName, PolygonMesh &_mesh){

    // Cloud file is loaded
    if (io::loadPolygonFileOBJ(_fileName, _mesh) == -1){
        std::string message("Couldn't read file ");
        message.append(_fileName);
        message.append(" \n");
        PCL_ERROR(message.c_str());
        return;
    }
}

void PcMesher::writeOBJMesh(const std::string _fileName, PolygonMesh& _mesh){

    std::cerr << "Exporting OBJ mesh" << std::endl;

    std::ofstream outputFile(_fileName.c_str());

    PointCloud<PointXYZRGBNormalCam> meshCloud;
    fromPCLPointCloud2 (_mesh.cloud, meshCloud);

    for (unsigned int i = 0; i < meshCloud.points.size(); i++){
        outputFile << "v " << meshCloud.points[i].x << " " << meshCloud.points[i].y << " " << meshCloud.points[i].z << std::endl;
    }


    // foreach face
    std::vector<Vertices, std::allocator<Vertices> >::iterator face_it;
    for (face_it = _mesh.polygons.begin(); face_it != _mesh.polygons.end(); ++face_it) {
        outputFile << "f " << face_it->vertices[0] + 1 << " " << face_it->vertices[1] + 1 << " " << face_it->vertices[2] + 1 << std::endl;
    }


    outputFile.close();

}


void PcMesher::exportIndices(PointIndices& _indices, const std::string _fileName){

    std::cerr << "Exporting a txt file with points not included in any plane" << std::endl;

    std::ofstream outputFile(_fileName.c_str());

    for (unsigned int i = 0; i < _indices.indices.size(); i++){
        outputFile << _indices.indices[i] << "\n";
    }


    outputFile.close();
}


void PcMesher::exportCamPerVtx(const std::string _fileName){

    std::cerr << "Exported a txt with the list of cameras which 'sees' each vertex" << std::endl;
    std::ofstream outputFile(_fileName.c_str());

    for (unsigned int i = 0; i < camPerVtx_.size(); i++){
        const std::vector<int> current = camPerVtx_[i];
        for (unsigned int j = 0; j < current.size(); j++){
            outputFile << current[j] << " ";
        }
        outputFile << "\n";
    }

    outputFile.close();

}

void PcMesher::deleteWrongCameras(const std::string _fileName){

    std::ifstream cam2del(_fileName.c_str());
    std::vector<int> lineNums;

    // We first read the input file which determines the cameras to delete
    if (cam2del.is_open()){

        std::string line;

        while(!cam2del.eof()){
            std::getline(cam2del, line);
            char * pEnd;
            int num = strtol(line.c_str(), &pEnd,10);
//            if (num == 0) continue;
            if (line.c_str() == pEnd) continue; // Too check if the result is not a number
            lineNums.push_back(num);
        }

    } else {
        cerr << "Unable to open image list file" << endl;
        exit(-1);
    }
    cam2del.close();

    // We remove the cameras from the list

    for (unsigned int i = 0; i < lineNums.size(); i++){
        const int index = lineNums[i];
        imageList_[index] = std::string("");
    }

    imageList_.erase(std::remove(imageList_.begin(), imageList_.end(), std::string("")), imageList_.end());

    for (int i = lineNums.size()-1; i >= 0; i--){
        const int index = lineNums[i];
        cameras_.erase(cameras_.begin()+index);
    }

    // We export a new image-list file

    std::ofstream newImageList("newImageList.txt");

    for (unsigned int i = 0; i < imageList_.size(); i++){
        newImageList << imageList_[i] << endl;
    }

    newImageList.close();
}


void PcMesher::removeOutliersFromCamPerVtx(PointIndices &_indices){

    // Where there is an outlier, we set an empty vector
    for (unsigned int i = 0; i < _indices.indices.size(); i++){
        camPerVtx_[_indices.indices[i]].clear();
    }

    // std::remove efficiently removes every entry where there is an empty vector
    camPerVtx_.erase(std::remove(camPerVtx_.begin(), camPerVtx_.end(), std::vector<int>(0)), camPerVtx_.end());

}


bool PcMesher::isPointCloseToPointCloud(const PointXYZRGBNormalCam &_point, const PointCloud<PointXYZRGBNormalCam>::Ptr _cloud, float _radius) const {


    KdTreeFLANN<PointXYZRGBNormalCam> kdtree;
    kdtree.setInputCloud(_cloud);

    std::vector<int> pointIdx;
    std::vector<float> pointSquaredDistance;

    if (kdtree.radiusSearch(_point, _radius, pointIdx, pointSquaredDistance) > 0){
        return true;
    } else {
        return false;
    }
}



