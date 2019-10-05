#include "utils.h"

namespace utils
{

}

// --------------------------------------------------------------------------------
// private method defintions for ::utils
// --------------------------------------------------------------------------------


// --------------------------------------------------------------------------------
// public method defintions for ::utils
// --------------------------------------------------------------------------------

std::string
utils::exec_path()
{

	//auto dir = std::filesystem::current_path().string();
	
	char result[PATH_MAX];
	auto count = readlink("/proc/self/exe", result, PATH_MAX);

	return std::string(result, (count > 0) ? count : 0);
}