#pragma once
#include "Type.h"
#include "Matrix.h"
#include "Object.h"

class ZIComputeUnit : public ZIObject
{
public:
	ZDEFINE_PTR(ZIComputeUnit)
	ZDEFINE_VECPTR(ZIComputeUnit)

	virtual void appendComputeUnit(ZIComputeUnit::Ptr compute_unit) = 0;

	virtual bool compute() = 0;

	virtual ZIComputeUnit::PtrVec getSubComputeUnit() = 0;

	virtual bool bindZIObject(ZIObject::PtrVec object) = 0;
};


class ZComputeUnit : public ZIComputeUnit
{
public:

	ZDEFINE_PTR(ZComputeUnit)
	ZDEFINE_VECPTR(ZComputeUnit)

	ZComputeUnit();
	ZComputeUnit(ZName name);

	// Inherited via ZIComputeUnit
	virtual void setName(ZName name) override;
	virtual ZName getName() override;
	virtual ZID getID() override;
	virtual ZObjectType getObjectType() override;

	virtual void appendComputeUnit(ZIComputeUnit::Ptr compute_unit) override;

	virtual bool compute() override;
	virtual ZIComputeUnit::PtrVec getSubComputeUnit() override;
	virtual bool bindZIObject(ZIObject::PtrVec object_vec) override;

protected:
	
	ZName name;
	ZID zid;
	ZIComputeUnit::PtrVec sub_compute_unit;

private:
	void init();

};