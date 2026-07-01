#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "CSpace.h"

namespace bp = boost::property_tree;

CSpaceConfig* CSpaceConfig::instance = nullptr;

CSpaceConfig* CSpaceConfig::get_instance()
{
	if(instance == nullptr)
		instance = new CSpaceConfig();
	return instance;
}

CSpaceConfig::CSpaceConfig()
	:top(),
     bottom(read_bottom()),  //各パラメータの最小値を読み込む
	 range(read_range()), //各パラメータの離散間隔を読み込む
	 symangle(read_symangle()) //対象物の回転対称角度を読み込む
{
	bp::ptree pt;
	read_ini("config/SpaceConfig.ini", pt);
	int x, y;
	boost::optional<int> carrier = pt.get_optional<int>("top.x");
	x = carrier.get();
	carrier = pt.get_optional<int>("top.y");
	y = carrier.get();
	
	top = State3D(x, y, symangle);
	numx = (top.x - bottom.x)/range.x + 1;
	numy = (top.y - bottom.y)/range.y + 1;
	numth = (top.th - bottom.th)/range.z + 1;
}


int read_symangle()
{
	bp::ptree pb, pt;
	read_ini("config/ProblemDefine.ini", pb);
	read_ini("config/ObjectParameter.ini", pt);
	boost::optional<int> carrier = pb.get_optional<int>("object.shape");
	int flag = carrier.get();
	if(flag == 1){
		carrier = pt.get_optional<int>("Rectangle.symmetry");
		return carrier.get();
	}
	else if(flag == 2){
		carrier = pt.get_optional<int>("LShape.symmetry");
		int th = carrier.get();
		return th;
	}
	else if(flag == 3){
		carrier = pt.get_optional<int>("Triangle.symmetry");
		int th = carrier.get();
		return th;
	}
	else if(flag == 4){
		carrier = pt.get_optional<int>("TShape.symmetry");
		int th = carrier.get();
		return th;
	}
	
	return 360;
}

State3D read_top()
{
	bp::ptree pt;
	read_ini("config/SpaceConfig.ini", pt);
	int x, y, th;
	boost::optional<int> carrier = pt.get_optional<int>("top.x");
	x = carrier.get();
	carrier = pt.get_optional<int>("top.y");
	y = carrier.get();
	carrier = pt.get_optional<int>("top.th");
	th = carrier.get();

	return State3D(x, y, th);
}

State3D read_bottom()
{
	bp::ptree pt;
	read_ini("config/SpaceConfig.ini", pt);
	int x, y, th;
	boost::optional<int> carrier = pt.get_optional<int>("bottom.x");
	x = carrier.get();
	carrier = pt.get_optional<int>("bottom.y");
	y = carrier.get();
	carrier = pt.get_optional<int>("bottom.th");
	th = carrier.get();

	return State3D(x, y, th);
}


Vector3D<int> read_range()
{
	bp::ptree pt;
	read_ini("config/SpaceConfig.ini", pt);
	int x, y, z;
	boost::optional<int> carrier = pt.get_optional<int>("range.x");
	x = carrier.get();
	carrier = pt.get_optional<int>("range.y");
	y = carrier.get();
	carrier = pt.get_optional<int>("range.th");
	z = carrier.get();

	return Vector3D<int>(x, y, z);
}
