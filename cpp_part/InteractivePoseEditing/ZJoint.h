#pragma once
#include "Type.h"
#include "ZTransform.h"
#include <boost/circular_buffer.hpp>

struct ZJointState
{
	ZName name;
	ZID jid;
	bool inherit_transform;
	ZMath::RotOrder rot_order;
	ZMath::ZVector3D translation;
	ZMath::ZVector3D joint_orient;
	ZMath::ZVector3D rotation;
	ZMath::ZVector3D rotate_axis;
	ZMath::ZVector3D scaling;
	ZITransform3D::Ptr parent;
	ZITransform3D::PtrVec childs;
};

struct Zt_TranslateLimits {
	ZScalar min_limit = std::numeric_limits<ZScalar>::lowest();
	ZScalar max_limit = std::numeric_limits<ZScalar>::max();
};

struct Zt_RotateLimits {
	ZScalar min_limit = -360.0f;
	ZScalar max_limit = 360.0f;
};

class ZJoint : public ZTransform
{
public:

	ZDEFINE_PTR(ZJoint)
	ZDEFINE_VECPTR(ZJoint)

	ZJoint(ZName name);

	virtual ZID getJID();
	virtual void setJID(ZID id);

	virtual void updateMatrix() override;

	virtual ZMath::ZVector3D getJointOrient();
	virtual ZMath::ZVector3D getRotateAxis();

	virtual const ZMath::ZTransform3D& JOMatrix();
	virtual const ZMath::ZTransform3D& ROMatrix();
	virtual const ZMath::ZTransform3D& ISMatrix();

	virtual void setJointOrient(const ZMath::ZVector3D& joint_orient);
	virtual void setRotateAxis(const ZMath::ZVector3D& rotate_axis);

	virtual void captureInitialHistory();
	virtual void restoreFromInitialHistory();

	// 通过 ZITransform3D 继承
	virtual ZObjectType getObjectType() override;

	// 通过 ZIUndoRedo 继承
	virtual ZHistoryRecord captureHistory(std::string history_info) override;

	virtual void restoreFromHistory(const ZHistoryRecord& history_record) override;

	virtual bool get_locked(size_t index) const { return locked[index]; }
	virtual void set_locked(size_t index, bool is_locked) { locked[index] = is_locked; }
	virtual unsigned int getDegrees();

	virtual Zt_TranslateLimits get_x_trans_limit() { return m_x_translate_limit; }
	virtual void set_x_trans_limit(ZScalar minX, ZScalar maxX) { m_x_translate_limit = { minX,maxX }; }
	virtual Zt_TranslateLimits get_y_trans_limit() { return m_y_translate_limit; }
	virtual void set_y_trans_limit(ZScalar minX, ZScalar maxX) { m_y_translate_limit = { minX,maxX }; }
	virtual Zt_TranslateLimits get_z_trans_limit() { return m_z_translate_limit; }
	virtual void set_z_trans_limit(ZScalar minX, ZScalar maxX) { m_z_translate_limit = { minX,maxX }; }

	virtual Zt_RotateLimits get_x_rotate_limit() { return m_x_rotate_limit; }
	virtual void set_x_rotate_limit(ZScalar minX, ZScalar maxX) { m_x_rotate_limit = { minX,maxX }; }
	virtual Zt_RotateLimits get_y_rotate_limit() { return m_y_rotate_limit; }
	virtual void set_y_rotate_limit(ZScalar minX, ZScalar maxX) { m_y_rotate_limit = { minX,maxX }; }
	virtual Zt_RotateLimits get_z_rotate_limit() { return m_z_rotate_limit; }
	virtual void set_z_rotate_limit(ZScalar minX, ZScalar maxX) { m_z_rotate_limit = { minX,maxX }; }


private:

	ZID jid;

	ZHistoryRecord initial_history;

	ZJointState captureState();

	void restoreState(const ZJointState& joint_state);
	
	ZMath::ZVector3D joint_orient;
	ZMath::ZTransform3D JOM;
	ZMath::ZVector3D rotate_axis;
	ZMath::ZTransform3D ROM;
	ZMath::ZTransform3D ISM;

	bool locked[9];
	Zt_TranslateLimits m_x_translate_limit;
	Zt_TranslateLimits m_y_translate_limit;
	Zt_TranslateLimits m_z_translate_limit;

	Zt_RotateLimits m_x_rotate_limit;
	Zt_RotateLimits m_y_rotate_limit;
	Zt_RotateLimits m_z_rotate_limit;

};