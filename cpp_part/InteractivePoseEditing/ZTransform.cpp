#include "pch.h"
#include "ZTransform.h"

using namespace ZMath;

ZIDAllocator* ZIDAllocator::allocator;

ZTransform::ZTransform()
{
	init();
}

ZTransform::ZTransform(ZName name)
{
	this->name = name;
	init();
}

const ZAnimationSequence& ZTransformAnimationSequence::getAnimationSequence(int flags) const
{
	return transform_sequence[flags];
}

ZName ZTransform::getName()
{
	return name;
}

void ZTransform::setName(ZName name)
{
	this->name = name;
}

ZName ZTransform::getLongName()
{
	if (parent)
	{
		ZName longName = parent->getLongName() + "|" + name;

		return longName;
	}
	else
	{
		return name;
	}
}

ZID ZTransform::getID()
{
	return zid;
}

ZObjectType ZTransform::getObjectType()
{
	return ZObjectType::Transform;
}

ZMath::RotOrder ZTransform::getRotOrder()
{
	return rot_order;
}

void ZTransform::setRotOrder(ZMath::RotOrder rot_order)
{
	this->rot_order = rot_order;
}

const ZMath::ZVector3D& ZTransform::getTranslation()
{
	return translation;
}

const ZMath::ZVector3D& ZTransform::getRotate()
{
	return rotation;
}

const ZMath::ZVector3D& ZTransform::getScale()
{
	return scaling;
}

void ZTransform::translate(ZMath::ZVector3D trans)
{
	this->translation = trans;
	TM = ZTranslation(translation);
	updateDirtyState();
}

void ZTransform::rotate(ZMath::ZVector3D rot)
{
	this->rotation = rot;
	RM = ZRotation(rotation, rot_order);
	updateDirtyState();
}

void ZTransform::scale(ZMath::ZVector3D scaling)
{
	this->scaling = scaling;
	SM = ZScaling(this->scaling);
	updateDirtyState();
}

const ZMath::ZTransform3D& ZTransform::TMatrix()
{
	return TM;
}

const ZMath::ZTransform3D& ZTransform::RMatrix()
{
	return RM;
}

const ZMath::ZTransform3D& ZTransform::SMatrix()
{
	return SM;
}

void ZTransform::updateMatrix()
{
	ZTransform3D parent_matrix = ZIdentify4D();
	if (parent)
	{
		parent_matrix = parent->worldMatrix();
	}
	local_matrix = TM * RM * SM;
	world_matrix = parent_matrix * local_matrix;
	dirty = false;

	for (auto& child : childs)
	{
		child->updateMatrix();
	}
	
}

void ZTransform::setKey(int key)
{
	cur_key = key;

	std::vector<ZAnimationFrame> frame_vec = getFrame(key);

	ZVectorXD trs = ZVectorXD::Zero(9);
	trs.head(3) = translation;
	trs.segment<3>(3) = rotation;
	trs.tail(3) = scaling;

	for (unsigned int ii = 0; ii < 9; ii++)
	{
		if (frame_vec[ii].is_valid)
		{
			trs(ii) = frame_vec[ii].value;
		}
	}

	translate(trs.head(3));
	rotate(trs.segment<3>(3));
	scale(trs.tail(3));
	dirty = true;
}

const ZMath::ZTransform3D& ZTransform::localMatrix()
{
	if (dirty) updateMatrix();
	return local_matrix;
}

const ZMath::ZTransform3D& ZTransform::worldMatrix()
{
	if (dirty) updateMatrix();
	return world_matrix;
}

void ZTransform::setComputeUnit(ZIComputeUnit::Ptr compute_unit)
{
	this->compute_unit = compute_unit;
}

ZIComputeUnit::Ptr ZTransform::getComputeUnit()
{
	return compute_unit;
}

bool ZTransform::inheritParentTransform()
{
	return inherit_parent_transform;
}

void ZTransform::setInheritParentTransform(bool is_inherit)
{
	inherit_parent_transform = is_inherit;
}

ZITransform3D::Ptr ZTransform::getParent()
{
	return parent;
}

std::vector<ZITransform3D::Ptr> ZTransform::getChilds()
{
	return childs;
}

void ZTransform::setParent(ZITransform3D::Ptr parent)
{
	this->parent = parent;
	dirty = true;
}
void ZTransform::appendChild(ZITransform3D::Ptr child)
{
	this->childs.emplace_back(child);
	dirty = true;
}

ZHistoryRecord ZTransform::captureHistory(std::string history_info)
{
	ZHistoryRecord history_record;
	ZTransformState state = captureState();
	history_record.second.history_info = history_info;
	history_record.second.value = state;
	return history_record;
}

void ZTransform::restoreFromHistory(const ZHistoryRecord& history_record)
{
	ZTransformState joint_state = boost::any_cast<ZTransformState>(history_record.second.value);

	restoreState(joint_state);
}

void ZTransform::updateDirtyState()
{
	dirty = true;
	for (const auto child : childs)
	{
		child->updateDirtyState();
	}
}

void ZTransform::init()
{
	rot_order = ZMath::RotOrder::XYZ;
	inherit_parent_transform = true;
	dirty = false;
	parent = nullptr;
	compute_unit = nullptr;
	zid = ZIDAllocator::get_ZIDAllocator()->bind(this);

	translation = ZMath::ZUnit3D();
	rotation = ZMath::ZUnit3D();
	scaling = ZMath::ZVector3D{ 1.0f,1.0f,1.0f };

	TM = ZMath::ZIdentify4D();
	RM = ZMath::ZIdentify4D();
	SM = ZMath::ZIdentify4D();
	world_matrix = ZMath::ZIdentify4D();
	local_matrix = ZMath::ZIdentify4D();
}

ZTransformState ZTransform::captureState()
{
	ZTransformState state;
	state.name = name;
	state.inherit_transform = inherit_parent_transform;
	state.rot_order = rot_order;
	state.translation = translation;
	state.rotation = rotation;
	state.scaling = scaling;
	state.childs = childs;
	state.parent = parent;

	return state;
}

void ZTransform::restoreState(const ZTransformState& state)
{
	name = state.name;
	inherit_parent_transform = state.inherit_transform;
	rot_order = state.rot_order;
	translate(state.translation);
	rotate(state.rotation);
	scale(state.scaling);
	childs = state.childs;
	parent = state.parent;

	updateMatrix();
}

void ZTransformAnimationSequence::setInterpolationType(InterpolationType type,int flags)
{
	transform_sequence[flags].interpolation_type = type;
}

std::vector<ZAnimationFrame> ZTransformAnimationSequence::getFrame(int key) const
{
	std::vector<ZAnimationFrame> ret;
	for (unsigned int ii = 0; ii < transform_sequence.size(); ii++)
	{
		ret.emplace_back(this->getFrame(key, ii));
	}

	return ret;
}

ZAnimationFrame ZTransformAnimationSequence::getFrame(int key, int flags) const
{
	if (key < transform_sequence[flags].min_key || key > transform_sequence[flags].max_key)
	{
		return ZAnimationFrame();
	}
	
	int n = binarySearchKey(key,flags);
	if (transform_sequence[flags].sequence[n].key == key)
	{
		return transform_sequence[flags].sequence[n];
	}

	return getInterpolationFrame(transform_sequence[flags].sequence[n], transform_sequence[flags].sequence[n + 1], key,flags);

}

//添加关键帧
bool ZTransformAnimationSequence::setKeyFrame(int key, const ZAnimationFrame& frame,int flags)
{
	ZAnimationFrame key_frame = frame;
	key_frame.is_key_frame = true;
	key_frame.is_valid = true;
	key_frame.key = key;
	key_frame.previous_key = -1;
	key_frame.next_key = -1;

	auto updateMinKey = [&](int min) { if (min_key > min) { min_key = min; }};
	auto updateMaxKey = [&](int max) { if (max_key < max) { max_key = max; }};

	if (transform_sequence[flags].sequence.empty())
	{
		transform_sequence[flags].sequence.emplace_back(key_frame);
		transform_sequence[flags].min_key = key;
		transform_sequence[flags].max_key = key;
		updateMinKey(key);
		updateMaxKey(key);
		return true;
	}
	else if (key < transform_sequence[flags].min_key || key > transform_sequence[flags].max_key)
	{
		if (key < transform_sequence[flags].min_key)
		{
			transform_sequence[flags].min_key = key;
			updateMinKey(key);
			key_frame.next_key = transform_sequence[flags].sequence.begin()->key;
			transform_sequence[flags].sequence.begin()->previous_key = key_frame.key;
			transform_sequence[flags].sequence.insert(transform_sequence[flags].sequence.begin(), key_frame);
		}
		if (key > transform_sequence[flags].max_key)
		{
			transform_sequence[flags].max_key = key;
			updateMaxKey(key);
			key_frame.previous_key = std::prev(transform_sequence[flags].sequence.end())->key;
			std::prev(transform_sequence[flags].sequence.end())->next_key = key_frame.key;
			transform_sequence[flags].sequence.insert(transform_sequence[flags].sequence.end(), key_frame);
		}
		return true;
	}
	else
	{
		int n = binarySearchKey(key,flags);
		if (transform_sequence[flags].sequence[n].key == key)
		{
			key_frame.previous_key = transform_sequence[flags].sequence[n].previous_key;
			key_frame.next_key = transform_sequence[flags].sequence[n].next_key;
			transform_sequence[flags].sequence[n] = key_frame;
		}
		else
		{
			key_frame.previous_key = transform_sequence[flags].sequence[n].key;
			key_frame.next_key = transform_sequence[flags].sequence[n + 1].key;
			transform_sequence[flags].sequence.insert(transform_sequence[flags].sequence.begin() + n, key_frame);
		}

		return true;
	}
}

std::pair<int, int> ZTransformAnimationSequence::getFrameRange() const
{
	return std::pair<int, int>(min_key, max_key);
}

bool ZTransformAnimationSequence::deleteKeyFrame(int key, int flags)
{
	if (transform_sequence[flags].sequence.empty())
	{
		return false;
	}

	int n = binarySearchKey(key,flags);

	if (transform_sequence[flags].sequence[n].key != key)
	{
		return false;
	}

	if (n == 0)
	{
		if (transform_sequence[flags].sequence.size() != 1)
		{
			transform_sequence[flags].sequence[1].previous_key = -1;
			
		}
		transform_sequence[flags].sequence.erase(transform_sequence[flags].sequence.begin());
		return true;
	}

	if (n == transform_sequence[flags].sequence.size() - 1)
	{
		if (transform_sequence[flags].sequence.size() != 1)
		{
			std::prev((std::prev(transform_sequence[flags].sequence.end())))->next_key = -1;
		}
		transform_sequence[flags].sequence.erase(std::prev(transform_sequence[flags].sequence.end()));
		return true;
	}

	transform_sequence[flags].sequence[n - 1].next_key = transform_sequence[flags].sequence[n + 1].key;
	transform_sequence[flags].sequence[n + 1].previous_key = transform_sequence[flags].sequence[n - 1].key;

	transform_sequence[flags].sequence.erase(transform_sequence[flags].sequence.begin() + n);

	return true;
}

void ZTransformAnimationSequence::clearKeyFrame(int flags)
{
	transform_sequence[flags].sequence.clear();
}

void ZTransformAnimationSequence::clearALLKeyFrame()
{
	for (unsigned int ii = 0; ii < transform_sequence.size(); ii++)
	{
		transform_sequence[ii].sequence.clear();
	}
}

//根据插值方式获得插值帧
ZAnimationFrame ZTransformAnimationSequence::getInterpolationFrame(const ZAnimationFrame& first, const ZAnimationFrame& second,int key, int flags) const
{
	ZAnimationFrame ret_frame;
	ret_frame.is_key_frame = false;
	ret_frame.is_valid = true;

	if (key < first.key || second.key < first.key)
	{
		return first;
	}
	if (key > second.key)
	{
		return second;
	}

	switch (transform_sequence[flags].interpolation_type)
	{
	case InterpolationType::Fasten:
		ret_frame = first;
		break;
	case InterpolationType::Linear:
		ZScalar rate = static_cast<ZScalar>(key - first.key) / static_cast<ZScalar>(second.key - first.key);
		ret_frame = first + rate * (second - first);
		break;
	}

	return ret_frame;
}

int ZTransformAnimationSequence::binarySearchKey(int key,int flags) const
{

	if (transform_sequence[flags].sequence.empty())
	{
		return -1;
	}

	if (key <= transform_sequence[flags].min_key || transform_sequence[flags].sequence.size() == 1)
	{
		return 0;
	}
	if (key >= transform_sequence[flags].max_key)
	{
		return transform_sequence[flags].sequence.size() - 1;
	}
	
	size_t left = 0, right = (transform_sequence[flags].sequence.size()) - 1;

	/*if(key > 200000)
		std::cout<< transform_sequence[flags].sequence.size() << std::endl;*/
		
	while (left < right)
	{
		if (key >= transform_sequence[flags].sequence[left].key && key < transform_sequence[flags].sequence[left + 1].key)
		{
			return left;
		}

		unsigned int mid = (left + right) / 2;
		if (key == transform_sequence[flags].sequence[mid].key)
		{
			return mid;
		}
		else if(key < transform_sequence[flags].sequence[mid].key)
		{
			right = mid;
		}
		else
		{
			left = mid;
		}
	}

	

	return left;
}
