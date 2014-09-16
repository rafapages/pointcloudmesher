#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include <stdio.h>

#include "pointXYZRGBNormalCam.h"


class Camera{

public:
    Camera();
    ~Camera();

    float getFocalLength();
    Eigen::Matrix3f getRotationMatrix();
    Eigen::Vector3f getTranslationVector();

    void setFocalLength(const float _f);
    void setDistortionCoefficients(const float _k1, const float _k2);
    void setRotationMatrix(const Eigen::Matrix3f _R);
    void setTranslationVector(const Eigen::Vector3f _t);

    Eigen::Vector3f getCameraPosition();
    void getCameraPosition(PointXYZRGBNormalCam& _point);

    void readCamera(std::ifstream& _stream);


private:
    float f_; // Focal length
    float k1_, k2_; // Radial distortion coefficients
    Eigen::Matrix3f R_; // Rotation matrix
    Eigen::Vector3f t_; // Camera translation vector

};



#endif
