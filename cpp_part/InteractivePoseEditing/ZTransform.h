#pragma once
#include "Type.h"
#include "Matrix.h"
#include "Object.h"
#include "ComputeInterface.h"

class ZITransform3D : public ZIObject
{
public:

	ZDEFINE_PTR(ZITransform3D)
	ZDEFINE_VECPTR(ZITransform3D)

	virtual ZName getLongName() = 0;

	virtual ZMath::RotOrder getRotOrder() = 0;

	virtual void setRotOrder(ZMath::RotOrder rot_order) = 0;

	virtual const ZMath::ZVector3D& getTranslation() = 0;

	virtual const ZMath::ZVector3D& getRotate() = 0;

	virtual const ZMath::ZVector3D& getScale() = 0;

	virtual void translate(ZMath::ZVector3D trans) = 0;

	virtual void rotate(ZMath::ZVector3D rot) = 0;

	virtual void scale(ZMath::ZVector3D scaling) = 0;

	virtual const ZMath::ZTransform3D& TMatrix() = 0;

	virtual const ZMath::ZTransform3D& RMatrix() = 0;

	virtual const ZMath::ZTransform3D& SMatrix() = 0;

	virtual void setKey(int key) = 0;

	virtual void updateMatrix() = 0;

	//get local matrix
	virtual const ZMath::ZTransform3D& localMatrix() = 0;

	//get world matrix
	virtual const ZMath::ZTransform3D& worldMatrix() = 0;

	virtual void setComputeUnit(ZIComputeUnit::Ptr compute_unit) = 0;

	virtual ZIComputeUnit::Ptr getComputeUnit() = 0;

	virtual bool inheritParentTransform() = 0;

	virtual void setInheritParentTransform(bool is_inherit) = 0;

	virtual ZITransform3D::Ptr getParent() = 0;

	virtual ZITransform3D::PtrVec getChilds() = 0;

	virtual void setParent(ZITransform3D::Ptr parent) = 0;

	virtual void appendChild(ZITransform3D::Ptr child) = 0;

	virtual void updateDirtyState() = 0;

};


struct ZHistoryState
{
	boost::any value;
	std::string history_info;
};

typedef std::pair<ZID, ZHistoryState> ZHistoryRecord;

class ZIUndoRedo
{
public:
	virtual ZHistoryRecord captureHistory(std::string history_info) = 0;
	virtual void restoreFromHistory(const ZHistoryRecord& history_record) = 0;
};

struct ZTransformState
{
	ZName name;
	bool inherit_transform;
	ZMath::RotOrder rot_order;
	ZMath::ZVector3D translation;
	ZMath::ZVector3D rotation;
	ZMath::ZVector3D scaling;
	ZITransform3D::Ptr parent;
	ZITransform3D::PtrVec childs;
};

enum class InterpolationType
{
	Fasten,
	Linear
};

struct ZAnimationFrame
{
	bool is_key_frame = false;
	bool is_valid = false;
	int key;
	int previous_key = -1;
	int next_key = -1;
	ZScalar value;

	friend ZAnimationFrame operator +(const ZAnimationFrame& a, const ZAnimationFrame& b)
	{
		ZAnimationFrame frame;
		frame.value = a.value + b.value;
		return frame;
	}
	friend ZAnimationFrame operator -(const ZAnimationFrame& a, const ZAnimationFrame& b)
	{
		ZAnimationFrame frame;
		frame.value = a.value - b.value;
		return frame;
	}
	friend ZAnimationFrame operator *(ZScalar scalar, const ZAnimationFrame& b)
	{
		ZAnimationFrame frame;
		frame.value = scalar * b.value;
		return frame;
	}
	friend ZAnimationFrame operator *(const ZAnimationFrame& a, ZScalar scalar)
	{
		ZAnimationFrame frame;
		frame.value = scalar * a.value;
		return frame;
	}
	friend ZAnimationFrame operator /(const ZAnimationFrame& a, ZScalar rate)
	{
		ZAnimationFrame frame;
		frame.value = a.value / rate;
		return frame;
	}
};

struct ZAnimationSequence
{
	std::vector<ZAnimationFrame> sequence = {};

	int min_key = std::numeric_limits<int>::max();
	int max_key = std::numeric_limits<int>::min();
	InterpolationType interpolation_type = InterpolationType::Linear;
};

class ZIAnimationSequence
{
public:
	virtual void setInterpolationType(InterpolationType type, int flags) = 0;

	virtual std::vector<ZAnimationFrame> getFrame(int key)const = 0;
	virtual ZAnimationFrame getFrame(int key, int flags)const = 0;

	virtual bool setKeyFrame(int key,const ZAnimationFrame& frame,int flags) = 0;

	virtual std::pair<int, int> getFrameRange()const = 0;

	virtual bool deleteKeyFrame(int key, int flags) = 0;

	virtual void clearKeyFrame(int flags) = 0;

	virtual void clearALLKeyFrame() = 0;
};

class ZTransformAnimationSequence : public ZIAnimationSequence
{
public:

	ZTransformAnimationSequence() = default;

	const ZAnimationSequence& getAnimationSequence(int flags) const;

	// Inherited via ZIAnimationSequence
	virtual void setInterpolationType(InterpolationType type, int flags) override;
	virtual std::vector<ZAnimationFrame> getFrame(int key) const override;
	virtual ZAnimationFrame getFrame(int key, int flags ) const override;
	virtual bool setKeyFrame(int key, const ZAnimationFrame& frame, int flags) override;
	virtual std::pair<int, int> getFrameRange()const override;
	virtual bool deleteKeyFrame(int key, int flags) override;
	virtual void clearKeyFrame(int flags) override;
	virtual void clearALLKeyFrame() override;

	std::array< ZAnimationSequence, 9> transform_sequence; // tx,ty,tz,rx,ry,rz,sx,sy,sz

	int min_key = std::numeric_limits<int>::max();
	int max_key = std::numeric_limits<int>::min();



private:
	ZAnimationFrame getInterpolationFrame(const ZAnimationFrame& first, const ZAnimationFrame& secound, int key,int flags) const;
	int binarySearchKey(int key,int flags) const;
};

class ZTransform : public ZITransform3D , public ZIUndoRedo , public ZTransformAnimationSequence
{
public:
	ZDEFINE_PTR(ZTransform)
	ZDEFINE_VECPTR(ZTransform)

	ZTransform();
	ZTransform(ZName name);
	virtual ~ZTransform() = default;

	

	// 通过 ZITransform3D 继承
	virtual ZName getName() override;
	virtual void setName(ZName name) override;
	virtual ZName getLongName() override;
	virtual ZID getID() override;
	virtual ZObjectType getObjectType() override;
	virtual ZMath::RotOrder getRotOrder() override;
	virtual void setRotOrder(ZMath::RotOrder rot_order) override;
	virtual const ZMath::ZVector3D& getTranslation() override;
	virtual const ZMath::ZVector3D& getRotate() override;
	virtual const ZMath::ZVector3D& getScale() override;
	virtual void translate(ZMath::ZVector3D trans) override;
	virtual void rotate(ZMath::ZVector3D rot) override;
	virtual void scale(ZMath::ZVector3D scaling) override;
	virtual const ZMath::ZTransform3D& TMatrix() override;
	virtual const ZMath::ZTransform3D& RMatrix() override;
	virtual const ZMath::ZTransform3D& SMatrix() override;
	virtual void updateMatrix() override;
	virtual void setKey(int key) override;
	virtual const ZMath::ZTransform3D& localMatrix() override;
	virtual const ZMath::ZTransform3D& worldMatrix() override;
	virtual void setComputeUnit(ZIComputeUnit::Ptr compute_unit) override;
	virtual ZIComputeUnit::Ptr getComputeUnit() override;
	virtual bool inheritParentTransform() override;
	virtual void setInheritParentTransform(bool is_inherit) override;
	virtual std::shared_ptr<ZITransform3D> getParent() override;
	virtual std::vector<std::shared_ptr<ZITransform3D>> getChilds() override;
	virtual void setParent(ZITransform3D::Ptr parent) override;
	virtual void appendChild(ZITransform3D::Ptr child) override;
	virtual void updateDirtyState() override;

	// 通过 ZIUndoRedo 继承
	virtual ZHistoryRecord captureHistory(std::string history_info) override;
	virtual void restoreFromHistory(const ZHistoryRecord& history_record) override;

protected:
	ZName name;
	ZID zid;

	int cur_key;

	ZMath::RotOrder rot_order;
	bool inherit_parent_transform;
	bool dirty;

	ZMath::ZVector3D translation; // 位移
	ZMath::ZTransform3D TM;

	ZMath::ZVector3D rotation; // 旋转
	ZMath::ZTransform3D RM;

	ZMath::ZVector3D scaling; // 缩放
	ZMath::ZTransform3D SM;

	ZMath::ZTransform3D world_matrix; //世界矩阵

	ZMath::ZTransform3D local_matrix; // 变换的local矩阵

	ZIComputeUnit::Ptr compute_unit;

	ZITransform3D::Ptr parent;

	ZITransform3D::PtrVec childs;

	

	//ZTransformAnimationSequence animation_sequence;

private:
	void init();
	ZTransformState captureState();
	void restoreState(const ZTransformState& state);


};
