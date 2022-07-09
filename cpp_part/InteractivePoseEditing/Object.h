#pragma once
#include "Type.h"

enum class ZObjectType
{
	Transform,
	Mesh,
	Skeleton,
	Joint,
	Camera,
	ImagePlane,
	CommonCompute,
	SkeletonFK,
	MotionRetargeting£¬
};

class ZIObject : public std::enable_shared_from_this<ZIObject>
{
public:

	ZDEFINE_PTR(ZIObject)
	ZDEFINE_VECPTR(ZIObject)

	virtual void setName(ZName name) = 0;

	virtual ZName getName() = 0;

	virtual ZID getID() = 0;

	virtual ZObjectType getObjectType() = 0;
};

//µ¥ÀýµÄID·ÖÅäÆ÷
class ZIDAllocator {

	static ZIDAllocator* allocator;
	std::map<ZID, ZIObject*> ZIDMap;

	ZIDAllocator() { cur_id = 0; }

	ZID cur_id;
	
public:
	static ZIDAllocator* get_ZIDAllocator() {

		if (!allocator) {
			allocator = new ZIDAllocator();
		}

		return allocator;
	}

	ZID bind(ZIObject* obj)
	{
		if (ZIDMap.find(cur_id) == ZIDMap.end()) {
			if (cur_id > std::numeric_limits<unsigned int>::max()) {
				throw std::logic_error("ID Allocator is full");
			}
			ZIDMap[cur_id] = obj;
			return cur_id++;
		}
		return -1;
	}

	ZIObject* find(ZID zid)
	{
		if (ZIDMap.find(zid) != ZIDMap.end()) {
			return ZIDMap.at(zid);
		}
		return nullptr;
	}

	void release(ZID zid)
	{
		std::map<ZID, ZIObject*>::iterator it = ZIDMap.find(zid);
		if (it != ZIDMap.end()) {
			ZIDMap.erase(it);
		}
	}
};
