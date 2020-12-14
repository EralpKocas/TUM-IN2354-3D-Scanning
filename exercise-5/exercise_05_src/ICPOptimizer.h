#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <flann/flann.hpp>

#include "SimpleMesh.h"
#include "NearestNeighbor.h"
#include "PointCloud.h"
#include "ProcrustesAligner.h"


/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const Vector3f& input, T* output) {
	output[0] = T(input[0]);
	output[1] = T(input[1]);
	output[2] = T(input[2]);
}


/**
 * Pose increment is only an interface to the underlying array (in constructor, no copy
 * of the input array is made).
 * Important: Input array needs to have a size of at least 6.
 */
template <typename T>
class PoseIncrement {
public:
	explicit PoseIncrement(T* const array) : m_array{ array } { }
	
	void setZero() {
		for (int i = 0; i < 6; ++i)
			m_array[i] = T(0);
	}

	T* getData() const {
		return m_array;
	}

	/**
	 * Applies the pose increment onto the input point and produces transformed output point.
	 * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
	 * beforehand).
	 */
	void apply(T* inputPoint, T* outputPoint) const {
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		const T* rotation = m_array;
		const T* translation = m_array + 3;

		T temp[3];
		ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

		outputPoint[0] = temp[0] + translation[0];
		outputPoint[1] = temp[1] + translation[1];
		outputPoint[2] = temp[2] + translation[2];
	}

	/**
	 * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
	 * transformation 4x4 matrix.
	 */
	static Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement) {
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		double* pose = poseIncrement.getData();
		double* rotation = pose;
		double* translation = pose + 3;

		// Convert the rotation from SO3 to matrix notation (with column-major storage).
		double rotationMatrix[9];
		ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

		// Create the 4x4 transformation matrix.
		Matrix4f matrix;
		matrix.setIdentity();
		matrix(0, 0) = float(rotationMatrix[0]);	matrix(0, 1) = float(rotationMatrix[3]);	matrix(0, 2) = float(rotationMatrix[6]);	matrix(0, 3) = float(translation[0]);
		matrix(1, 0) = float(rotationMatrix[1]);	matrix(1, 1) = float(rotationMatrix[4]);	matrix(1, 2) = float(rotationMatrix[7]);	matrix(1, 3) = float(translation[1]);
		matrix(2, 0) = float(rotationMatrix[2]);	matrix(2, 1) = float(rotationMatrix[5]);	matrix(2, 2) = float(rotationMatrix[8]);	matrix(2, 3) = float(translation[2]);
		
		return matrix;
	}

private:
	T* m_array;
};


/**
 * Optimization constraints.
 */
class PointToPointConstraint {
public:
	PointToPointConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) :
		m_sourcePoint{ sourcePoint },
		m_targetPoint{ targetPoint },
		m_weight{ weight }
	{ }

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {

		PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T* const>(pose));

		// TODO: Implement the point-to-point cost function.
		// The resulting 3D residual should be stored in the residuals array. To apply the pose 
		// increment (pose parameters) to the source point, you can use the PoseIncrement class.
		// Important: Ceres automatically squares the cost function.
        T source_res[3] = {T(0), T(0), T(0)};
        fillVector(m_sourcePoint, source_res);

        T target_res[3] = {T(0), T(0), T(0)};
        fillVector(m_targetPoint, target_res);

        T pose_incremented[3] = {T(0), T(0), T(0)};
        poseIncrement.apply(source_res, pose_incremented);

        residuals[0] = T(ceres::sqrt(LAMBDA)) * T(ceres::sqrt(m_weight)) * (pose_incremented[0] - target_res[0]);
		residuals[1] = T(ceres::sqrt(LAMBDA)) * T(ceres::sqrt(m_weight)) * (pose_incremented[1] - target_res[1]);
		residuals[2] = T(ceres::sqrt(LAMBDA)) * T(ceres::sqrt(m_weight)) * (pose_incremented[2] - target_res[2]);
        return true;
	}

	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) {
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
			new PointToPointConstraint(sourcePoint, targetPoint, weight)
		);
	}

protected:
	const Vector3f m_sourcePoint;
	const Vector3f m_targetPoint;
	const float m_weight;
	const float LAMBDA = 0.1f;
};

class PointToPlaneConstraint {
public:
	PointToPlaneConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) :
		m_sourcePoint{ sourcePoint },
		m_targetPoint{ targetPoint },
		m_targetNormal{ targetNormal },
		m_weight{ weight }
	{ }

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {

		PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T* const>(pose));
		
		// TODO: Implement the point-to-plane cost function.
		// The resulting 1D residual should be stored in the residuals array. To apply the pose 
		// increment (pose parameters) to the source point, you can use the PoseIncrement class.
		// Important: Ceres automatically squares the cost function.
		T source_res[3] = {T(0), T(0), T(0)};
        fillVector(m_sourcePoint, source_res);

        T target_res[3] = {T(0), T(0), T(0)};
        fillVector(m_targetPoint, target_res);

        poseIncrement.apply(source_res, source_res);
        T x_dist = T(m_targetNormal.x()) * (source_res[0] - target_res[0]);
        T y_dist = T(m_targetNormal.y()) * (source_res[1] - target_res[1]);
        T z_dist = T(m_targetNormal.z()) * (source_res[2] - target_res[2]);
		//residuals[0] = T(ceres::sqrt(LAMBDA)) * T(ceres::sqrt(m_weight)) * ceres::sqrt(ceres::pow(x_dist, 2) + ceres::pow(y_dist, 2) + ceres::pow(z_dist, 2));
		residuals[0] = T(ceres::sqrt(LAMBDA)) * T(ceres::sqrt(m_weight)) * (x_dist + y_dist + z_dist);
		return true;
	}

	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) {
		return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
			new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight)
		);
	}

protected:
	const Vector3f m_sourcePoint;
	const Vector3f m_targetPoint;
	const Vector3f m_targetNormal;
	const float m_weight;
	const float LAMBDA = 1.0f;
};


/**
 * ICP optimizer - Abstract Base Class, using Ceres for optimization.
 */
class ICPOptimizer {
public:
	ICPOptimizer() : 
		m_bUsePointToPlaneConstraints{ false },
		m_nIterations{ 20 },
		m_nearestNeighborSearch{ std::make_unique<NearestNeighborSearchFlann>() }
	{ }

	void setMatchingMaxDistance(float maxDistance) {
		m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
	}

	void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints) {
		m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
	}

	void setNbOfIterations(unsigned nIterations) {
		m_nIterations = nIterations;
	}

	virtual Matrix4f estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f initialPose = Matrix4f::Identity()) = 0;

protected:
	bool m_bUsePointToPlaneConstraints;
	unsigned m_nIterations;
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;

	std::vector<Vector3f> transformPoints(const std::vector<Vector3f>& sourcePoints, const Matrix4f& pose) {
		std::vector<Vector3f> transformedPoints;
		transformedPoints.reserve(sourcePoints.size());

		const auto rotation = pose.block(0, 0, 3, 3);
		const auto translation = pose.block(0, 3, 3, 1);

		for (const auto& point : sourcePoints) {
			transformedPoints.push_back(rotation * point + translation);
		}

		return transformedPoints;
	}

	std::vector<Vector3f> transformNormals(const std::vector<Vector3f>& sourceNormals, const Matrix4f& pose) {
		std::vector<Vector3f> transformedNormals;
		transformedNormals.reserve(sourceNormals.size());

		const auto rotation = pose.block(0, 0, 3, 3);

		for (const auto& normal : sourceNormals) {
			transformedNormals.push_back(rotation.inverse().transpose() * normal);
		}

		return transformedNormals;
	}

	void pruneCorrespondences(const std::vector<Vector3f>& sourceNormals, const std::vector<Vector3f>& targetNormals, std::vector<Match>& matches) {
		const unsigned nPoints = sourceNormals.size();

		for (unsigned i = 0; i < nPoints; i++) {
			Match& match = matches[i];
			if (match.idx >= 0) {
				const auto& sourceNormal = sourceNormals[i];
				const auto& targetNormal = targetNormals[match.idx];

				// TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60

                double source_mag = ceres::sqrt(ceres::pow(sourceNormal.x(), 2) + ceres::pow(sourceNormal.y(), 2) + ceres::pow(sourceNormal.z(), 2));
				double target_mag = ceres::sqrt(ceres::pow(targetNormal.x(), 2) + ceres::pow(targetNormal.y(), 2) + ceres::pow(targetNormal.z(), 2));

                double cos_similarity = sourceNormal.dot(targetNormal) / (source_mag * target_mag);
                double arccos_rad = ceres::acos(cos_similarity);
                double cos_degree = arccos_rad * 180 / M_PI;
                if (cos_degree > 60){
                    match.idx = -1;
                }
			}
		}
	}
};


/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer {
public:
	CeresICPOptimizer() {}

	virtual Matrix4f estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f initialPose = Matrix4f::Identity()) override {
		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		m_nearestNeighborSearch->buildIndex(target.getPoints());

		// The initial estimate can be given as an argument.
		Matrix4f estimatedPose = initialPose;

		// We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
		// the rotation angle) and 3 parameters for the translation vector. 
		double incrementArray[6];
		auto poseIncrement = PoseIncrement<double>(incrementArray);
		poseIncrement.setZero();

		for (int i = 0; i < m_nIterations; ++i) {
			// Compute the matches.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
			auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

			auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
			pruneCorrespondences(transformedNormals, target.getNormals(), matches);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			// Prepare point-to-point and point-to-plane constraints.
			ceres::Problem problem;
			prepareConstraints(transformedPoints, target.getPoints(), target.getNormals(), matches, poseIncrement, problem);

			// Configure options for the solver.
			ceres::Solver::Options options;
			configureSolver(options);

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cout << summary.BriefReport() << std::endl;
			//std::cout << summary.FullReport() << std::endl;

			// Update the current pose estimate (we always update the pose from the left, using left-increment notation).
			Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
			estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
			poseIncrement.setZero();

			std::cout << "Optimization iteration done." << std::endl;
		}

		return estimatedPose;
	}


private:
	void configureSolver(ceres::Solver::Options& options) {
		// Ceres options.
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.use_nonmonotonic_steps = false;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = 1;
		options.max_num_iterations = 1;
		options.num_threads = 8;
	}

	void prepareConstraints(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals, const std::vector<Match> matches, const PoseIncrement<double>& poseIncrement, ceres::Problem& problem) const {
		const unsigned nPoints = sourcePoints.size();

		for (unsigned i = 0; i < nPoints; ++i) {
			const auto match = matches[i];
			if (match.idx >= 0) {
				const auto& sourcePoint = sourcePoints[i];
				const auto& targetPoint = targetPoints[match.idx];
				const auto& weight = match.weight;

				if (!sourcePoint.allFinite() || !targetPoint.allFinite())
					continue;

				// TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block) 
				// to the Ceres problem.
                PointToPointConstraint p2p_constraint = PointToPointConstraint(sourcePoint, targetPoint, weight);
                problem.AddResidualBlock(
                        p2p_constraint.create(sourcePoint, targetPoint, weight),
                        nullptr, poseIncrement.getData()
                        );

                if (m_bUsePointToPlaneConstraints) {
					const auto& targetNormal = targetNormals[match.idx];

					if (!targetNormal.allFinite())
						continue;

					// TODO: Create a new point-to-plane cost function and add it as constraint (i.e. residual block) 
					// to the Ceres problem.
                    PointToPlaneConstraint p2plane_constraint = PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight);
                    problem.AddResidualBlock(
                            p2plane_constraint.create(sourcePoint, targetPoint, targetNormal, weight),
                            nullptr, poseIncrement.getData()
                    );
					
				}
			}
		}
	}
};


/**
 * ICP optimizer - using linear least-squares for optimization.
 */
class LinearICPOptimizer : public ICPOptimizer {
public:
	LinearICPOptimizer() {}

	virtual Matrix4f estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f initialPose = Matrix4f::Identity()) override {
		// Build the index of the FLANN tree (for fast nearest neighbor lookup).
		m_nearestNeighborSearch->buildIndex(target.getPoints());

		// The initial estimate can be given as an argument.
		Matrix4f estimatedPose = initialPose;

		for (int i = 0; i < m_nIterations; ++i) {
			// Compute the matches.
			std::cout << "Matching points ..." << std::endl;
			clock_t begin = clock();

			auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
			auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

			auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
			pruneCorrespondences(transformedNormals, target.getNormals(), matches);

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			std::vector<Vector3f> sourcePoints;
			std::vector<Vector3f> targetPoints;

			// Add all matches to the sourcePoints and targetPoints vectors,
			// so that sourcePoints[i] matches targetPoints[i].
			for (int j = 0; j < transformedPoints.size(); j++) {
				const auto& match = matches[j];
				if (match.idx >= 0) {
					sourcePoints.push_back(transformedPoints[j]);
					targetPoints.push_back(target.getPoints()[match.idx]);
				}
			}

			// Estimate the new pose
			if (m_bUsePointToPlaneConstraints) {
				estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, target.getNormals()) * estimatedPose;
			}
			else {
				estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
			}

			std::cout << "Optimization iteration done." << std::endl;
		}

		return estimatedPose;
	}

private:
	Matrix4f estimatePosePointToPoint(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ProcrustesAligner procrustAligner;
		Matrix4f estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

		return estimatedPose;
	}

	Matrix4f estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals) {
		const unsigned nPoints = sourcePoints.size();

		// Build the system
		MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
		VectorXf b = VectorXf::Zero(4 * nPoints);

		for (unsigned i = 0; i < nPoints; i++) {
			const auto& s = sourcePoints[i];
			const auto& d = targetPoints[i];
			const auto& n = targetNormals[i];

			// TODO: Add the point-to-plane constraints to the system
            float ai1 = n.z() * s.y() - n.y() * s.z();
            float ai2 = n.x() * s.z() - n.z() * s.x();
            float ai3 = n.y() * s.x() - n.x() - s.y();
            float nix = n.x();
            float niy = n.y();
            float niz = n.z();
			Matrix<float, 1, 6> curr_row;
            curr_row << ai1, ai2, ai3, nix, niy, niz;
            A.block(i*4, 0, 1, 6) = curr_row;
            b[i*4] = n.x() * d.x() + n.y() * d.y() + n.z() * d.z() - n.x() * s.x() - n.y() * s.y() - n.z() * s.z();
			// TODO: Add the point-to-point constraints to the system
            /*Matrix<float, 1, 6> curr_row_x;
            curr_row_x << 0, -1, 1, 1, 0, 0;
            Matrix<float, 1, 6> curr_row_y;
            curr_row_y << 1, 0, -1, 0, 1, 0;
            Matrix<float, 1, 6> curr_row_z;
            curr_row_z << -1, 1, 0, 0, 0, 1;*/

            Matrix<float, 1, 6> curr_row_x;
            curr_row_x << 0, -n.z(), n.y(), n.x(), 0, 0;

            Matrix<float, 1, 6> curr_row_y;
            curr_row_y << n.z(), 0, -n.x(), 0, n.y(), 0;

            Matrix<float, 1, 6> curr_row_z;
            curr_row_z << -n.y(), n.x(), 0, 0, 0, n.z();

            A.block(i*4 + 1, 0, 1, 6) = curr_row_x;
            A.block(i*4 + 2, 0, 1, 6) = curr_row_y;
            A.block(i*4 + 3, 0, 1, 6) = curr_row_z;

            b[i*4 + 1] = n.x() * (d.x() - s.x());
            b[i*4 + 2] = n.y() * (d.y() - s.y());
            b[i*4 + 3] = n.z() * (d.z() - s.z());

            /*b[i*4 + 1] = (d.x() - s.x());
            b[i*4 + 2] = (d.y() - s.y());
            b[i*4 + 3] = (d.z() - s.z());*/
			// TODO: Optionally, apply a higher weight to point-to-plane correspondences

		}

		// TODO: Solve the system
		VectorXf x(6);
        x = (A.transpose() * A).inverse() * A.transpose() * b;

        /*JacobiSVD<Matrix3f> svd(A, ComputeFullU | ComputeFullV);
        const Matrix3f& U = svd.matrixU();
        const Matrix3f& V = svd.matrixV();
        V*svd.singularValues().head().inverse()*U.transpose();
        x = (V * svd.singularValues().inverse() * U.transpose()) * b;
        x = A.completeOrthogonalDecomposition().pseudoInverse() * b;
        x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);*/

        float alpha = x(0), beta = x(1), gamma = x(2);

        // Build the pose matrix
		Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
			                AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
			                AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

        Vector3f translation = x.tail(3);

		// TODO: Build the pose matrix using the rotation and translation matrices
		Matrix4f estimatedPose = Matrix4f::Identity();

        estimatedPose.block(0 , 0, 3, 3) = rotation;
        estimatedPose.block(0 , 3, 3, 1) = translation;

		return estimatedPose;
	}
};
