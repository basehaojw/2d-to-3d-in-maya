#pragma once
#include "Type.h"
namespace ZMath
{
	struct ZAxisAngle
	{
		ZAxisAngle()
		{
			rx = 0.0f;
			ry = 0.0f;
			rz = 0.0f;
		}
		ZAxisAngle(ZScalar rx, ZScalar ry, ZScalar rz)
		{
			this->rx = rx;
			this->ry = ry;
			this->rz = rz;
		}
		ZScalar rx;
		ZScalar ry;
		ZScalar rz;

		inline ZAxisAngle operator +(const ZAxisAngle& axis_angle)
		{
			ZAxisAngle ret_axis_angle;
			ret_axis_angle.rx = rx + axis_angle.rx;
			ret_axis_angle.ry = ry + axis_angle.ry;
			ret_axis_angle.rz = rz + axis_angle.rz;
			return ret_axis_angle;
		}
		inline ZAxisAngle operator -(const ZAxisAngle& axis_angle)
		{
			ZAxisAngle ret_axis_angle;
			ret_axis_angle.rx = rx - axis_angle.rx;
			ret_axis_angle.ry = ry - axis_angle.ry;
			ret_axis_angle.rz = rz - axis_angle.rz;
			return ret_axis_angle;
		}
	};

	enum class RotOrder
	{
		XYZ,
		XZY,
		YXZ,
		YZX,
		ZYX,
		ZXY
	};

	using ZTransform3D = Eigen::Transform<ZScalar, 3, Eigen::Affine>;

	using ZMatrix2D = Eigen::Matrix<ZScalar, 2, 2>;

	using ZMatrix3D = Eigen::Matrix<ZScalar, 3, 3>;

	using ZMatrix4D = Eigen::Matrix<ZScalar, 4, 4>;

	using ZMatrixXD = Eigen::Matrix<ZScalar, -1, -1>;

	using ZVector2D = Eigen::Matrix<ZScalar, 2, 1>;

	using ZVector3D = Eigen::Matrix<ZScalar, 3, 1>;

	using ZVector4D = Eigen::Matrix<ZScalar, 4, 1>;

	using ZVectorXD = Eigen::Matrix<ZScalar, -1, 1>;

	using ZQuat = Eigen::Quaternion<ZScalar>;

	/*using ZVector3D_Vec = std::vector<ZVectorXD, Eigen::aligned_allocator<ZVector3D>>;

	using ZVector4D_Vec = std::vector<ZVectorXD, Eigen::aligned_allocator<ZVector4D>>;

	using ZVectorXD_Vec = std::vector<ZVectorXD, Eigen::aligned_allocator<Eigen::VectorXd>>;*/
	using ZVector3D_Vec = std::vector<ZVector3D>;

	using ZVector4D_Vec = std::vector<ZVector4D>;

	using ZVectorXD_Vec = std::vector<ZVectorXD>;


	struct ZBoundingBox3D
	{
		void updateX(ZScalar x);
		void updateY(ZScalar x);
		void updateZ(ZScalar x);

		bool check(ZVector3D point);

		std::pair<ZScalar, ZScalar> bbx = { 0.0f,0.0f };
		std::pair<ZScalar, ZScalar> bby = { 0.0f,0.0f };
		std::pair<ZScalar, ZScalar> bbz = { 0.0f,0.0f };
	};


	ZVector3D ZUnit3D();
	ZVector4D ZUnit4D();

	ZVector3D ZUnitX3D();
	ZVector3D ZUnitY3D();
	ZVector3D ZUnitZ3D();

	ZVector4D ZUnitX4D();
	ZVector4D ZUnitY4D();
	ZVector4D ZUnitZ4D();

	ZMatrix3D matrix4d_3d(const ZMatrix4D& matrix);

	ZMatrix4D matrix3d_4d(const ZMatrix3D& matrix);

	ZVector3D vector4d_3d(const ZVector4D& vec4);
	ZVector4D vector3d_4d(const ZVector3D& vec3);

	ZTransform3D ZTranslation(ZScalar tx, ZScalar ty, ZScalar tz);

	ZTransform3D ZTranslation(const ZVector3D& translation);

	ZTransform3D ZRotation(ZRadian rx, ZRadian ry, ZRadian rz, RotOrder rot_order = RotOrder::XYZ);

	ZTransform3D ZRotation(const ZVector3D& rotation, RotOrder rot_order = RotOrder::XYZ);

	ZTransform3D ZScaling(ZScalar sx, ZScalar sy, ZScalar sz);

	ZTransform3D ZScaling(const ZVector3D& scaling);

	ZTransform3D ZRotationX(ZRadian rx);

	ZTransform3D ZRotationY(ZRadian ry);

	ZTransform3D ZRotationZ(ZRadian rz);

	ZTransform3D ZIdentify4D();

	ZMatrix4D DerivateX(ZRadian rx);

	ZMatrix4D DerivateY(ZRadian ry);

	ZMatrix4D DerivateZ(ZRadian rz);

	ZAxisAngle rotationMatrixToJointAngle(const ZMatrix4D& matrix, RotOrder rot_order = RotOrder::XYZ);

	ZAxisAngle rotationMatrixToJointAngle(const ZMatrix3D& matrix, RotOrder rot_order = RotOrder::XYZ);

	ZAxisAngle rotationMatrixToJointAngle(const ZTransform3D& matrix, RotOrder rot_order = RotOrder::XYZ);

	ZAxisAngle flipEuler(ZAxisAngle axis_angle,RotOrder rot_order);
	ZVector3D flipEuler(ZVector3D rotation,RotOrder rot_order);

	ZAxisAngle eulerDiffFilter(ZAxisAngle axis_angle, ZAxisAngle ref_axis_angle);
	ZVector3D eulerDiffFilter(ZVector3D rotation,ZVector3D ref_rotation);
	ZAxisAngle eulerFilter(ZAxisAngle axis_angle, ZAxisAngle ref_axis_angle, RotOrder rot_order);
	ZVector3D eulerFilter(ZVector3D rotation, ZVector3D ref_rotation, RotOrder rot_order);

	ZQuat ZQuatFromTwoVector(ZVector3D a, ZVector3D b);

	ZMatrix3D ZRotationFromTwoVector(ZVector3D a, ZVector3D b);

	ZMatrix3D getSkewMatrixFromVector(ZVector3D a);

	ZRadian angleBetweenVectors(ZVector3D a, ZVector3D b);

	ZRadian angleBetweenVectors(ZVector2D a, ZVector2D b);

	ZRadian degree2Radian(ZDegree degree);

	ZDegree radian2Degree(ZRadian radian);

	void KronekerProduct(ZMatrixXD A, ZMatrixXD B, ZMatrixXD& ret);


	class ZQuaternion
	{
	public:

		ZQuaternion() {}

		ZQuaternion(ZScalar w, ZScalar x, ZScalar y, ZScalar z)
		{
			q = Eigen::Quaternion<ZScalar>(w, x, y, z);
		}

		ZQuaternion(ZMatrix3D mat)
		{
			q = Eigen::Quaternion<ZScalar>(mat);
		}

		ZScalar w() const {
			return q.w();
		}
		ZScalar x() const {
			return q.x();
		}
		ZScalar y()const {
			return q.y();
		}
		ZScalar z() const {
			return q.z();
		}

		ZQuaternion operator/ (ZScalar scalar) const
		{
			ZScalar w = q.w() / scalar;
			ZScalar x = q.x() / scalar;
			ZScalar y = q.y() / scalar;
			ZScalar z = q.z() / scalar;
			return ZQuaternion(w,x,y,z);
		}

		ZQuaternion operator/= (ZScalar scalar) {
			ZScalar w = q.w() / scalar;
			ZScalar x = q.x() / scalar;
			ZScalar y = q.y() / scalar;
			ZScalar z = q.z() / scalar;
			q = Eigen::Quaternion<ZScalar>(w, x, y, z);
			return *this;
		}
		ZQuaternion operator* (const ZQuaternion& zq) const
		{
			Eigen::Quaternion<ZScalar> eq(zq.w(), zq.x(), zq.y(), zq.z());
			Eigen::Quaternion<ZScalar> ret_eq = q* eq;
			return ZQuaternion(ret_eq.w(), ret_eq.x(), ret_eq.y(), ret_eq.z());
		}

		ZQuaternion operator* (ZScalar scalar) const
		{
			return ZQuaternion(q.w() * scalar,
				q.x() * scalar,
				q.y() * scalar,
				q.z() * scalar);
		}
		
		

		ZQuaternion operator+ (const ZQuaternion& zq) const
		{
			return ZQuaternion(q.w() + zq.w(),
				q.x() + zq.x(),
				q.y() + zq.y(),
				q.z() + zq.z());
		}

		ZQuaternion operator-() const {
			return ZQuaternion(-q.w(), -q.x(), -q.y(), -q.z());
		}

		void setIdentity()
		{
			q.setIdentity();
		}

		ZMatrix3D matrix3d()
		{
			return q.toRotationMatrix();
		}

		ZMatrix4D matrix4d()
		{
			return matrix3d_4d(q.toRotationMatrix());
		}

		ZQuaternion conjugate()
		{
			Eigen::Quaternion<ZScalar> qc = q.conjugate();
			return ZQuaternion(qc.w(), qc.x(), qc.y(), qc.z());
		}

		ZVector3D getVec3()
		{
			return ZVector3D(q.x(), q.y(), q.z());
		}
		ZScalar getReal()
		{
			
			return q.w();
		}

	private:
		Eigen::Quaternion<ZScalar> q;
	};

	static ZQuaternion operator* (ZScalar scalar, ZQuaternion const& zq)
	{
		return ZQuaternion(zq.w() * scalar,
			zq.x() * scalar,
			zq.y() * scalar,
			zq.z() * scalar);
	}

	class ZDualQuaternion
	{
	public:
		ZDualQuaternion()
		{
			qr.setIdentity();
			qd = 0.5f * ZQuaternion(0, 0, 0, 0) * qr;
		}

		ZDualQuaternion(const ZQuaternion& qr, const ZQuaternion& qd)
		{
			this->qr = qr;
			this->qd = qd;
		}

		ZDualQuaternion(const ZQuaternion& qr, const ZVector3D& t)
		{
			this->qr = qr;
			ZQuaternion tmp = ZQuaternion(0, t(0) / 2.0f, t(1) / 2.0f, t(2) / 2.0f);
			this->qd = tmp * qr;
		}

		ZDualQuaternion(const ZMath::ZMatrix4D mat)
		{
			this->qr = ZQuaternion(mat.block(0, 0, 3, 3));
			ZVector3D t = mat.col(3).head<3>();
			ZQuaternion tmp = ZQuaternion(0, t(0), t(1), t(2));
			this->qd = 0.5f * tmp * qr;
		}

		ZDualQuaternion(const ZMath::ZMatrix3D mat)
		{
			this->qr = ZQuaternion(mat);
			ZQuaternion tmp = ZQuaternion(0, 0, 0, 0);
			this->qd = 0.5f * tmp * qr;
		}

		ZDualQuaternion operator+(const ZDualQuaternion& dq) const
		{
			return ZDualQuaternion(qr + dq.qr, qd + dq.qd);
		}
		ZDualQuaternion operator*(const ZDualQuaternion& dq) const
		{
			return ZDualQuaternion(qr * dq.Qr(), qr * dq.Qd() + qd * dq.Qr());
		}

		ZVector3D transformPoint(ZVector3D p)
		{
			ZDualQuaternion zp(ZQuaternion(1, 0, 0, 0), ZQuaternion(0, p(0), p(1), p(2)));

			ZDualQuaternion ret = ZDualQuaternion(qr, qd) * zp * ZDualQuaternion(qr.conjugate(),-qd.conjugate());

			return ZVector3D(ret.Qd().getVec3());
		}

		ZQuaternion Qr() const {
			return qr;
		}
		ZQuaternion Qd() const {
			return qd;
		}

	private:
		ZQuaternion qr;
		ZQuaternion qd;
	};
}