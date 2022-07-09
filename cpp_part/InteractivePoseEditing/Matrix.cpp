#include "pch.h"
#include "Matrix.h"

namespace ZMath
{
	ZVector3D ZUnit3D() { return ZVector3D{ 0.0f,0.0f,0.0f }; }
	ZVector4D ZUnit4D() { return ZVector4D{ 0.0f,0.0f,0.0f,1.0f }; }

	ZVector3D ZUnitX3D() { return ZVector3D{ 1.0f,0.0f,0.0f }; }
	ZVector3D ZUnitY3D() { return ZVector3D{ 0.0f,1.0f,0.0f }; }
	ZVector3D ZUnitZ3D() { return ZVector3D{ 0.0f,0.0f,1.0f }; }

	ZVector4D ZUnitX4D() { return ZVector4D{ 1.0f,0.0f,0.0f,1.0f }; }
	ZVector4D ZUnitY4D() { return ZVector4D{ 0.0f,1.0f,0.0f,1.0f }; }
	ZVector4D ZUnitZ4D() { return ZVector4D{ 0.0f,0.0f,1.0f,1.0f }; }


	ZMatrix3D matrix4d_3d(const ZMatrix4D& matrix)
	{
		ZMatrix3D matrix3d = matrix.block(0, 0, 3, 3);
		return matrix3d;
	}

	ZMatrix4D matrix3d_4d(const ZMatrix3D& matrix)
	{
		ZMatrix4D matrix4d = ZMath::ZIdentify4D().matrix();

		matrix4d.block(0, 0, 3, 3) = matrix;

		return matrix4d;
	}

	ZVector3D vector4d_3d(const ZVector4D& vec4)
	{
		return vec4.head(3);
	}

	ZVector4D vector3d_4d(const ZVector3D& vec3)
	{
		ZVector4D vec4;
		vec4.head(3) = vec3;
		vec4(3) = 1.0f;
		return vec4;
	}

	ZTransform3D ZTranslation(ZScalar tx, ZScalar ty, ZScalar tz)
	{
		Eigen::Translation<ZScalar, 3> translation = Eigen::Translation<ZScalar, 3>(tx, ty, tz);
		return ZTransform3D(translation);
	}

	ZTransform3D ZTranslation(const ZVector3D& translation)
	{
		return ZTranslation(translation(0), translation(1), translation(2));
	}

	ZTransform3D ZRotation(ZRadian rx, ZRadian ry, ZRadian rz, RotOrder rot_order)
	{
		switch (rot_order)
		{
		case RotOrder::XYZ:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ()) * Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY()) * Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX());
			return ZTransform3D(q);
		}
		case RotOrder::XZY:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY()) * Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ()) * Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX());
			return ZTransform3D(q);
		}
		case RotOrder::YXZ:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ()) * Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX()) * Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY());
			return ZTransform3D(q);
		}
		case RotOrder::YZX:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX()) * Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ()) * Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY());
			return ZTransform3D(q);
		}
		case RotOrder::ZYX:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX()) * Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY()) * Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ());
			return ZTransform3D(q);
		}
		case RotOrder::ZXY:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY()) * Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX()) * Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ());
			return ZTransform3D(q);
		}
		default:
		{
			Eigen::Quaternion<ZRadian> q;
			q = Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ()) * Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY()) * Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX());
			return ZTransform3D(q);
		}
		}
	}

	ZTransform3D ZRotation(const ZVector3D& rotation, RotOrder rot_order)
	{
		return ZRotation(rotation(0), rotation(1), rotation(2), rot_order);
	}

	ZTransform3D ZScaling(ZScalar sx, ZScalar sy, ZScalar sz)
	{
		Eigen::DiagonalMatrix<ZScalar,3> scaling = Eigen::Scaling(sx, sy, sz);
		return ZTransform3D(scaling);
	}

	ZTransform3D ZScaling(const ZVector3D& scaling)
	{
		return ZScaling(scaling(0), scaling(1), scaling(2));
	}

	ZTransform3D ZRotationX(ZRadian rx)
	{
		Eigen::Quaternion<ZRadian> q;
		q = Eigen::AngleAxis<ZRadian>(rx, ZVector3D::UnitX());
		return ZTransform3D(q);
	}

	ZTransform3D ZRotationY(ZRadian ry)
	{
		Eigen::Quaternion<ZRadian> q;
		q = Eigen::AngleAxis<ZRadian>(ry, ZVector3D::UnitY());
		return ZTransform3D(q);
	}

	ZTransform3D ZRotationZ(ZRadian rz)
	{
		Eigen::Quaternion<ZRadian> q;
		q = Eigen::AngleAxis<ZRadian>(rz, ZVector3D::UnitZ());
		return ZTransform3D(q);
	}

	ZTransform3D ZIdentify4D()
	{
		Eigen::Quaternion<ZRadian> q;
		q = Eigen::AngleAxis<ZRadian>::Identity();
		return ZTransform3D(q);
	}

	ZMatrix4D DerivateX(ZRadian rx)
	{
		ZMatrix4D derivate_m;
		derivate_m << 0, 0, 0, 0,
			0, -sin(rx), -cos(rx), 0,
			0, cos(rx), -sin(rx), 0,
			0, 0, 0, 0;
		return derivate_m;
	}

	ZMatrix4D DerivateY(ZRadian ry)
	{
		ZMatrix4D derivate_m;
		derivate_m << -sin(ry), 0, cos(ry), 0,
			0, 0, 0, 0,
			-cos(ry), 0, -sin(ry), 0,
			0, 0, 0, 0;
		return derivate_m;
	}

	ZMatrix4D DerivateZ(ZRadian rz)
	{
		ZMatrix4D derivate_m;
		derivate_m << -sin(rz), -cos(rz), 0, 0,
			cos(rz), -sin(rz), 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;
		return derivate_m;
	}

	ZAxisAngle rotationMatrixToJointAngle(const ZMatrix4D& matrix, RotOrder rot_order)
	{
		ZVector3D ret;

		ZMatrix3D matrix3d = matrix4d_3d(matrix);

		return rotationMatrixToJointAngle(matrix3d);
	}

	ZAxisAngle rotationMatrixToJointAngle(const ZMatrix3D& matrix, RotOrder rot_order)
	{
		ZVector3D ret;

		ZAxisAngle joint_angle;

		switch (rot_order)
		{
		case RotOrder::XYZ:
		{
			ret = matrix.eulerAngles(2, 1, 0);
			joint_angle.rz = radian2Degree(ret(0));
			joint_angle.ry = radian2Degree(ret(1));
			joint_angle.rx = radian2Degree(ret(2));
			break;
		}
		case RotOrder::XZY:
		{
			ret = matrix.eulerAngles(1, 2, 0);
			joint_angle.ry = radian2Degree(ret(0));
			joint_angle.rz = radian2Degree(ret(1));
			joint_angle.rx = radian2Degree(ret(2));
			break;
		}
		case RotOrder::YXZ:
		{
			ret = matrix.eulerAngles(1, 0, 2);
			joint_angle.ry = radian2Degree(ret(0));
			joint_angle.rx = radian2Degree(ret(1));
			joint_angle.rz = radian2Degree(ret(2));
			break;
		}
		case RotOrder::YZX:
		{
			ret = matrix.eulerAngles(0, 2, 1);
			joint_angle.rx = radian2Degree(ret(0));
			joint_angle.rz = radian2Degree(ret(1));
			joint_angle.ry = radian2Degree(ret(2));
			break;
		}
		case RotOrder::ZYX:
		{
			ret = matrix.eulerAngles(0, 1, 2);
			joint_angle.rx = radian2Degree(ret(0));
			joint_angle.ry = radian2Degree(ret(1));
			joint_angle.rz = radian2Degree(ret(2));
			break;
		}
		case RotOrder::ZXY:
		{
			ret = matrix.eulerAngles(1, 0, 2);
			joint_angle.ry = radian2Degree(ret(0));
			joint_angle.rx = radian2Degree(ret(1));
			joint_angle.rz = radian2Degree(ret(2));
			break;
		}
		}

		//ZAxisAngle joint_angle(radian2Degree(ret(0)), radian2Degree(ret(1)), radian2Degree(ret(2)));

		return joint_angle;
	}

	ZAxisAngle rotationMatrixToJointAngle(const ZTransform3D& matrix, RotOrder rot_order)
	{
		ZVector3D ret;

		ZMatrix3D matrix3d = matrix4d_3d(matrix.matrix());

		return rotationMatrixToJointAngle(matrix3d,rot_order);

	}

	ZAxisAngle flipEuler(ZAxisAngle axis_angle, RotOrder rot_order)
	{
		ZAxisAngle ret_angle;
		switch (rot_order)
		{
		case RotOrder::XYZ:
			ret_angle.rx = axis_angle.rx + 180.0f;
			ret_angle.ry = -axis_angle.ry + 180.0f;
			ret_angle.rz = axis_angle.rz + 180.0f;
			break;
		case RotOrder::XZY:
			ret_angle.rx = axis_angle.rx + 180.0f;
			ret_angle.rz = -axis_angle.rz + 180.0f;
			ret_angle.ry = axis_angle.ry + 180.0f;
			break;
		case RotOrder::YXZ:
			ret_angle.ry = axis_angle.ry + 180.0f;
			ret_angle.rx = -axis_angle.rx + 180.0f;
			ret_angle.rz = axis_angle.rz + 180.0f;
			break;
		case RotOrder::YZX:
			ret_angle.ry = axis_angle.ry + 180.0f;
			ret_angle.rz = -axis_angle.rz + 180.0f;
			ret_angle.rx = axis_angle.rx + 180.0f;
			break;
		case RotOrder::ZXY:
			ret_angle.rz = axis_angle.rz + 180.0f;
			ret_angle.rx = -axis_angle.rx + 180.0f;
			ret_angle.ry = axis_angle.ry + 180.0f;
			break;
		case RotOrder::ZYX:
			ret_angle.rz = axis_angle.rz + 180.0f;
			ret_angle.ry = -axis_angle.ry + 180.0f;
			ret_angle.rx = axis_angle.rx + 180.0f;
			break;
		default:
			break;
		}
		return ret_angle;
	}

	ZVector3D flipEuler(ZVector3D rotation, RotOrder rot_order)
	{
		ZMath::ZVector3D ret_rotation;
		switch (rot_order)
		{
		case RotOrder::XYZ:
			ret_rotation(0) = rotation(0) + M_PI;
			ret_rotation(1) = -rotation(1) + M_PI;
			ret_rotation(2) = rotation(2) + M_PI;
			break;
		case RotOrder::XZY:
			ret_rotation(0) = rotation(0) + M_PI;
			ret_rotation(2) = -rotation(2) + M_PI;
			ret_rotation(1) = rotation(1) + M_PI;
			break;
		case RotOrder::YXZ:
			ret_rotation(1) = rotation(1) + M_PI;
			ret_rotation(0) = -rotation(0) + M_PI;
			ret_rotation(2) = rotation(2) + M_PI;
			break;
		case RotOrder::YZX:
			ret_rotation(1) = rotation(1) + M_PI;
			ret_rotation(2) = -rotation(2) + M_PI;
			ret_rotation(0) = rotation(0) + M_PI;
			break;
		case RotOrder::ZXY:
			ret_rotation(2) = rotation(2) + M_PI;
			ret_rotation(0) = -rotation(0) + M_PI;
			ret_rotation(1) = rotation(1) + M_PI;
			break;
		case RotOrder::ZYX:
			ret_rotation(2) = rotation(2) + M_PI;
			ret_rotation(1) = -rotation(1) + M_PI;
			ret_rotation(0) = rotation(0) + M_PI;
			break;
		default:
			break;
		}
		return ret_rotation;
	}

	ZAxisAngle eulerDiffFilter(ZAxisAngle axis_angle,ZAxisAngle ref_axis_angle)
	{
		ZAxisAngle ret_axis_angle;
		//rx:
		if (axis_angle.rx <= ref_axis_angle.rx)
		{
			while (axis_angle.rx <= ref_axis_angle.rx)
			{
				axis_angle.rx += 360.0f;
			}
			if (abs(axis_angle.rx - ref_axis_angle.rx) >= abs(ref_axis_angle.rx + 360.0f - axis_angle.rx))
			{
				ret_axis_angle.rx = axis_angle.rx - 360.0f;
			}
			else
			{
				ret_axis_angle.rx = axis_angle.rx;
			}

		}
		else
		{
			while (axis_angle.rx >= ref_axis_angle.rx)
			{
				axis_angle.rx -= 360.0f;
			}
			if (abs(axis_angle.rx - ref_axis_angle.rx) >= abs(axis_angle.rx + 360.0f - ref_axis_angle.rx))
			{
				ret_axis_angle.rx = axis_angle.rx + 360.0f;
			}
			else
			{
				ret_axis_angle.rx = axis_angle.rx;
			}

		}
		//ry:
		if (axis_angle.ry <= ref_axis_angle.ry)
		{
			while (axis_angle.ry <= ref_axis_angle.ry)
			{
				axis_angle.ry += 360.0f;
			}
			if (abs(axis_angle.ry - ref_axis_angle.ry) >= abs(ref_axis_angle.ry + 360.0f - axis_angle.ry))
			{
				ret_axis_angle.ry = axis_angle.ry - 360.0f;
			}
			else
			{
				ret_axis_angle.ry = axis_angle.ry;
			}
		}
		else
		{
			while (axis_angle.ry >= ref_axis_angle.ry)
			{
				axis_angle.ry -= 360.0f;
			}
			if (abs(axis_angle.ry - ref_axis_angle.ry) >= abs(axis_angle.ry + 360.0f - ref_axis_angle.ry))
			{
				ret_axis_angle.ry = axis_angle.ry + 360.0f;
			}
			else
			{
				ret_axis_angle.ry = axis_angle.ry;
			}
		}
		//rz:
		if (axis_angle.rz <= ref_axis_angle.rz)
		{
			while (axis_angle.rz <= ref_axis_angle.rz)
			{
				axis_angle.rz += 360.0f;
			}
			if (abs(axis_angle.rz - ref_axis_angle.rz) >= abs(ref_axis_angle.rz + 360.0f - axis_angle.rz))
			{
				ret_axis_angle.rz = axis_angle.rz - 360.0f;
			}
			else
			{
				ret_axis_angle.rz = axis_angle.rz;
			}
		}
		else
		{
			while (axis_angle.rz >= ref_axis_angle.rz)
			{
				axis_angle.rz -= 360.0f;
			}
			if (abs(axis_angle.rz - ref_axis_angle.rz) >= abs(axis_angle.rz + 360.0f - ref_axis_angle.rz))
			{
				ret_axis_angle.rz = axis_angle.rz + 360.0f;
			}
			else
			{
				ret_axis_angle.rz = axis_angle.rz;
			}
		}



		return ret_axis_angle;
	}

	ZVector3D eulerDiffFilter(ZVector3D rotation,ZVector3D ref_rotation)
	{
		ZVector3D ret_rotation;

		for (unsigned int ii = 0; ii < 3; ii++)
		{
			if (rotation(ii) <= ref_rotation(ii))
			{
				while (rotation(ii) <= ref_rotation(ii))
				{
					rotation(ii) += 2.0f * M_PI;
				}
				if (abs(rotation(ii) - ref_rotation(ii)) >= abs(rotation(ii) - 2.0f * M_PI - ref_rotation(ii)))
				{
					ret_rotation(ii) = rotation(ii) - 2.0f * M_PI;
				}
				else
				{
					ret_rotation(ii) = rotation(ii);
				}
			}
			else
			{
				while (rotation(ii) >= ref_rotation(ii))
				{
					rotation(ii) -= 2.0f * M_PI;
				}
				if (abs(rotation(ii) - ref_rotation(ii)) >= abs(rotation(ii) + 2.0f * M_PI - ref_rotation(ii)))
				{
					ret_rotation(ii) = rotation(ii) + 2.0f * M_PI;
				}
				else
				{
					ret_rotation(ii) = rotation(ii);
				}
			}
		}

		return ret_rotation;
	}

	ZAxisAngle eulerFilter(ZAxisAngle axis_angle, ZAxisAngle ref_axis_angle, RotOrder rot_order)
	{
		ZAxisAngle ret_axis_angle;
		axis_angle = eulerDiffFilter(axis_angle, ref_axis_angle);
		ZAxisAngle flip_axis_angle = flipEuler(axis_angle, rot_order);
		flip_axis_angle = eulerDiffFilter(flip_axis_angle, ref_axis_angle);

		ZAxisAngle diff_axis_angle = axis_angle - ref_axis_angle;
		ZScalar diff = sqrt(diff_axis_angle.rx * diff_axis_angle.rx + diff_axis_angle.ry * diff_axis_angle.ry + diff_axis_angle.rz * diff_axis_angle.rz);

		ZAxisAngle diff_flip_axis_angle = flip_axis_angle - ref_axis_angle;
		ZScalar diff_flip = sqrt(diff_flip_axis_angle.rx * diff_flip_axis_angle.rx + diff_flip_axis_angle.ry * diff_flip_axis_angle.ry + diff_flip_axis_angle.rz * diff_flip_axis_angle.rz);

		if (diff <= diff_flip)
		{
			return axis_angle;
		}
		else
		{
			return flip_axis_angle;
		}

	}

	ZVector3D eulerFilter(ZVector3D rotation, ZVector3D ref_rotation, RotOrder rot_order)
	{
		ZVector3D ret_rotation;

		rotation = eulerDiffFilter(rotation, ref_rotation);
		ZVector3D flip_rotation = flipEuler(rotation, rot_order);
		flip_rotation = eulerDiffFilter(flip_rotation, ref_rotation);

		ZScalar diff = (rotation - ref_rotation).norm();
		ZScalar diff_flip = (flip_rotation - ref_rotation).norm();

		if (diff < diff_flip)
		{
			ret_rotation = rotation;
		}
		else
		{
			ret_rotation = flip_rotation;
		}

		return ret_rotation;
	}

	//from : https://zalo.github.io/blog/inverse-kinematics/
	ZQuat ZQuatFromTwoVector(ZVector3D a, ZVector3D b)
	{
		ZScalar norm_u_norm_v = sqrt(a.dot(a) * b.dot(b));
		ZScalar real_part = norm_u_norm_v + a.dot(b);
		ZVector3D w;

		if (real_part < 1.e-6 * norm_u_norm_v)
		{
			/* If u and v are exactly opposite, rotate 180 degrees
		 * around an arbitrary orthogonal axis. Axis normalisation
		 * can happen later, when we normalise the quaternion. */

			real_part = 0.0f;
			w = abs(a(0)) > abs(a(2)) ? ZVector3D{-a(1),a(0),0.0f} : ZVector3D{ 0.0f,-a(2),a(1) };
		}
		else
		{
			w = a.cross(b);
		}

		ZQuat q = ZQuat(real_part, w(0), w(1), w(2));

		return q.normalized();
	}

	ZMatrix3D ZRotationFromTwoVector(ZVector3D a, ZVector3D b)
	{
		ZVector3D v = a.cross(b);
		v = v / v.norm();
		ZMatrix3D v_hat = getSkewMatrixFromVector(v);
		ZScalar cos_tht = a.dot(b) / a.norm() / b.norm();
		ZScalar tht = acos(cos_tht);

		ZMatrix3D R;
		R.setIdentity();

		R = R + v_hat * sin(tht) + v_hat * v_hat * (1 - cos(tht));

		return R;
	}

	ZMatrix3D getSkewMatrixFromVector(ZVector3D a)
	{
		ZMatrix3D R;
		R.setZero();

		R(0, 1) = -a(2); R(0, 2) = a(1);
		R(1, 0) = a(2); R(1, 2) = -a(0);
		R(2, 0) = -a(1); R(2, 1) = a(0);

		return R;
	}

	ZRadian angleBetweenVectors(ZVector3D a, ZVector3D b)
	{
		ZScalar dot_value = a.dot(b);
		ZScalar lenSq1 = a.norm();
		ZScalar lenSq2 = b.norm();

		ZScalar ret = dot_value / sqrt(lenSq1 * lenSq2);
		if (ret > 1.0f) {
			ret = 1.0f;
		}
		else if (ret < -1.0f) {
			ret = -1.0f;
		}

		return acos(ret);
	}

	ZRadian angleBetweenVectors(ZVector2D a, ZVector2D b)
	{
		a = a.normalized();
		b = b.normalized();

		ZScalar ret = a.dot(b);

		return acos(ret);
	}

	ZRadian degree2Radian(ZDegree degree)
	{
		return degree * M_PI / 180.0f;
	}

	ZDegree radian2Degree(ZRadian radian)
	{
		return radian / M_PI * 180.0f;
	}

	void KronekerProduct(ZMatrixXD A, ZMatrixXD B, ZMatrixXD& ret)
	{
		unsigned int a_rows = A.rows();
		unsigned int a_cols = A.cols();
		unsigned int b_rows = B.rows();
		unsigned int b_cols = B.cols();

		if (ret.rows() != a_rows * b_rows)
		{
			throw std::runtime_error("rows not equal" + std::to_string(a_rows * b_rows) + " != " + std::to_string(ret.rows()));
		}
		if (ret.cols() != a_cols * b_cols)
		{
			throw std::runtime_error("cols not equal" + std::to_string(a_cols * b_cols) + " != " + std::to_string(ret.cols()));
		}

		for (unsigned int ii = 0; ii < a_rows; ii++)
		{
			for (unsigned int jj = 0; jj < a_cols; jj++)
			{
				ret.block(ii * b_rows, jj * b_cols, b_rows, b_cols) = A(ii, jj) * B;
			}
		}
	}

	void ZBoundingBox3D::updateX(ZScalar x)
	{
		if (x > bbx.second)
		{
			bbx.second = x;
		}
		if (x < bbx.first)
		{
			bbx.first = x;
		}
	}
	void ZBoundingBox3D::updateY(ZScalar y)
	{
		if (y > bby.second)
		{
			bby.second = y;
		}
		if (y < bby.first)
		{
			bby.first = y;
		}
	}
	void ZBoundingBox3D::updateZ(ZScalar z)
	{
		if (z > bbz.second)
		{
			bbz.second = z;
		}
		if (z < bbz.first)
		{
			bbz.first = z;
		}
	}
	bool ZBoundingBox3D::check(ZVector3D point)
	{
		if (point(0) < bbx.first || point(0) > bbx.second)
		{
			return false;
		}
		if (point(1) < bby.first || point(1) > bby.second)
		{
			return false;
		}
		if (point(1) < bbz.first || point(1) > bbz.second)
		{
			return false;
		}
		return true;
	}
}
