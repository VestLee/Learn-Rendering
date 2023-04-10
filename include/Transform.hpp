#pragma once
#include "Eigen/Core"
#include <Eigen/Dense>

using namespace Eigen;
namespace LRR//Learn Realtime Rendering
{
	class Transform
	{
	public :
		Transform();

		// Rotate
		static Matrix4f get_Z_rotation_matrix(float rotation_angle);
		static Matrix4f get_Rodrigues_rotation(Vector3f axis, float rotation_angle);
		static Matrix4f get_Quaternions_rotation(Vector3f point, Vector3f axis, float rotation_angle);

		// View Matrix
		// Model Space To View Space
		static Matrix4f get_view_matrix(Vector3f eye_pos);
		// Projection Matrix
		// View Space To Projection Space
		static Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar);

	private:
		 static inline double Degree(double angle) { return angle * EIGEN_PI / 180.0; }
	};
}