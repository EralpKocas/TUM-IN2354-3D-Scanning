#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);

		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean);

		// To apply the pose to point x on shape X in the case of Procrustes, we execute:
		// 1. Translation of a point to the shape Y: x' = x + t
		// 2. Rotation of the point around the mean of shape Y: 
		//    y = R (x' - yMean) + yMean = R (x + t - yMean) + yMean = R x + (R t - R yMean + yMean)
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();

        estimatedPose.block(0, 0, 3, 3) = rotation;
        estimatedPose.block(0 , 3, 3, 1) = -rotation * sourceMean + translation + sourceMean;

        return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		Vector3f mean = Vector3f::Zero();
		int num_points = points.size();
        for(int i = 0; i < num_points; i++){
		    mean += points[i];
        }
		mean /= (float) num_points;
        return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.
		Matrix3f rotation = Matrix3f::Identity();

		Eigen::Matrix<float, 4, 3> sourcePoints_zeroMean = Matrix<float, 4, 3>().setZero();
		Eigen::Matrix<float, 4, 3> targetPoints_zeroMean = Matrix<float, 4, 3>().setZero();

        int num_points = sourcePoints.size();
        for(int i = 0; i < num_points; i++){
            sourcePoints_zeroMean.row(i) = Vector3f(sourcePoints[i].x()-sourceMean.x(), sourcePoints[i].y()-sourceMean.y(), sourcePoints[i].z()-sourceMean.z());
            targetPoints_zeroMean.row(i) = Vector3f(targetPoints[i].x()-targetMean.x(), targetPoints[i].y()-targetMean.y(), targetPoints[i].z()-targetMean.z());
        }
        Matrix3f ccMatrix = targetPoints_zeroMean.transpose() * sourcePoints_zeroMean;
        JacobiSVD<Matrix3f> svd(ccMatrix, ComputeFullU | ComputeFullV);
        rotation = svd.matrixU() * svd.matrixV().transpose();
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean) {
		// TODO: Compute the translation vector from source to target points.
		Vector3f translation = Vector3f::Zero();
		translation = targetMean - sourceMean;
		return translation;
	}
};