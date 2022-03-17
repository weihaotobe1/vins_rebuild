#pragma once

#include <eigen3/Eigen/Dense>
#include "estimator.h"
#include "estimatorparameters.h"
#include <fstream>
#include "pgsystem.h"
class pgsystem;
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const double &header, pgsystem* _pgsystem);

//void pubOdometry(const Estimator &estimator, const double &header, pgsystem* _pgsystem);

//void pubTF(const Estimator &estimator, const double &header, pgsystem* _pgsystem);
void pubOdometryTF(const Estimator &estimator, const double &header, pgsystem* _pgsystem);

void pubKeyframe(const Estimator &estimator, pgsystem* _pgsystem);

void pubRelocalization(const Estimator &estimator, pgsystem* _pgsystem);
