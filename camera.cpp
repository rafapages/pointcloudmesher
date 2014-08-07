#include "camera.h"
#include "pointXYZRGBNormalCam.h"

Camera::Camera(){

}

Camera::~Camera(){

}

float Camera::getFocalLenght(){
    return f_;
}

Eigen::Matrix3f Camera::getRotationMatrix(){
    return R_;
}

Eigen::Vector3f Camera::getTranslationVector(){
    return t_;
}

void Camera::setFocalLenght(const float _f){
    f_ = _f;
}

void Camera::setDistortionCoefficients(const float _k1, const float _k2){
    k1_ = _k1;
    k2_ = _k2;
}

void Camera::setRotationMatrix(const Eigen::Matrix3f _R){
    R_ = _R;
}

void Camera::setTranslationVector(const Eigen::Vector3f _t){
    t_ = _t;
}

Eigen::Vector3f Camera::getCameraPosition(){
    return R_* t_;
}
