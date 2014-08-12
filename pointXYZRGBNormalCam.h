#ifndef POINTXYZRGBNORMALCAM_H
#define POINTXYZRGBNORMALCAM_H

#include <pcl/point_types.h>

using namespace pcl;

struct PointXYZRGBNormalCam{
    PCL_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
      struct
      {
        PCL_ADD_UNION_RGB
        float curvature;
      };
      float data_c[4];
    };

    int camera;

    PCL_ADD_EIGEN_MAPS_RGB
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBNormalCam,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (int, camera, camera)
                                   )


#endif
