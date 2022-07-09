#pragma once
#include "pch.h"
#include "framework.h"

#define OUT

using ZInteger = int;

using ZScalar = double;

using ZDegree = double;

using ZRadian = double;

using ZID = unsigned int;

#define ZDEFINE_PTR(className) \
	typedef std::shared_ptr<className> Ptr; \
	typedef std::shared_ptr<const className> ConstPtr;

#define ZDEFINE_VECPTR(className) \
	typedef std::vector<std::shared_ptr<className>> PtrVec; \
	typedef std::vector<std::shared_ptr<const className>> ConstPtrVec;

#define ZDEFINE_MAPPTR(type,className) \
	typedef std::map<type,std::shared_ptr<className>> PtrMap; \
	typedef std::map<type,std::shared_ptr<const className>> ConstPtrMap;

#ifndef M_PI
const ZScalar M_PI = 3.14159265358979323846f;  /* pi */
#endif // !_INC_MATH


const ZScalar double_toleration = 1e-7f;

#define SLOT_HANDLE

inline bool zt_isZero(double x) { return abs(x) < double_toleration; }

class ZName {
public:
	inline ZName() = default;
	inline ZName(std::string t_name) { m_name = t_name; }
	inline ZName(const char* t_name) { m_name = t_name; }
	inline void set_name(std::string& t_name) { m_name = t_name; }
	inline void set_name(const std::string& t_name) { m_name = t_name; }
	inline std::string get_str() { return m_name; }
	inline std::string get_str() const { return m_name; }
	inline size_t size() { return m_name.size(); }
	inline size_t size() const { return m_name.size(); }

	inline ZName& operator =(const ZName& t_name) { m_name = t_name.get_str(); return *this; }
	inline ZName operator +(const ZName& t_name) { ZName new_name; new_name.m_name = t_name.get_str() + m_name; return new_name; }
	//inline ZName operator +(const ZName& first, const ZName& second) { std::string tmp_name; tmp_name = first.m_name + second.m_name; return tmp_name; }
	inline bool operator<(const ZName& b) const { return this->m_name < b.m_name; }
	inline bool operator==(const ZName& b) const { return this->m_name == b.m_name; }
	inline bool operator!=(const ZName& b) const { return this->m_name != b.m_name; }
	friend inline std::ostream& operator <<(std::ostream& out, ZName& t_name) { out << t_name.m_name; return out; }
private:
	std::string m_name;
};

class ZPath {
public:

	inline ZPath() {}

	inline ZPath(std::string t_path) { m_path = t_path; }
	inline ZPath(const char* t_name) { m_path = t_name; }

	inline void set_path(std::string& t_path) { m_path = t_path; }
	inline void set_path(const std::string& t_path) { m_path = t_path; }
	inline std::string get_str() { return m_path.string(); }
	inline std::string get_str() const { return m_path.string(); }
	inline size_t size() { return m_path.size(); }
	inline size_t size() const { return m_path.size(); }

	inline bool is_exist() { return boost::filesystem::exists(m_path); }
	inline bool is_exist() const { return boost::filesystem::exists(m_path); }
	inline bool is_directory() { return boost::filesystem::is_directory(m_path); }
	inline bool is_directory() const { return boost::filesystem::is_directory(m_path); }
	inline std::string get_extensions() const { return boost::filesystem::extension(m_path); }
	inline std::string get_filename() const { return m_path.filename().string(); }
	inline std::string get_stem() const { return m_path.stem().string(); }
	friend inline std::ostream& operator <<(std::ostream& out, ZPath& t_path) { out << t_path; return out; }
private:
	boost::filesystem::path m_path;

};