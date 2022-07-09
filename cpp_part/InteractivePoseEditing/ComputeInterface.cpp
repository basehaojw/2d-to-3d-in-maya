#include "pch.h"
#include "ComputeInterface.h"

ZComputeUnit::ZComputeUnit()
{
	init();
}

ZComputeUnit::ZComputeUnit(ZName name)
{
	this->name = name;
	init();
}

void ZComputeUnit::setName(ZName name)
{
	this->name = name;
}

ZName ZComputeUnit::getName()
{
	return name;
}

ZID ZComputeUnit::getID()
{
	return zid;
}

ZObjectType ZComputeUnit::getObjectType()
{
	return ZObjectType::CommonCompute;
}

void ZComputeUnit::appendComputeUnit(ZIComputeUnit::Ptr compute_unit)
{
	sub_compute_unit.emplace_back(compute_unit);
}

bool ZComputeUnit::compute()
{
	return true;
}

std::vector<ZIComputeUnit::Ptr> ZComputeUnit::getSubComputeUnit()
{
	return sub_compute_unit;
}

bool ZComputeUnit::bindZIObject(ZIObject::PtrVec object_vec)
{
	return false;
}

void ZComputeUnit::init()
{
	zid = ZIDAllocator::get_ZIDAllocator()->bind(this);
}
