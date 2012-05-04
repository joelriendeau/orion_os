#include "dynamic_memory.hpp"

#include <list>
#include "boost/any.hpp"

int main(int argc, char* argv[])
{
    boost::any a;
    std::list<boost::any> many;

    a = 10;
    many.push_back(a);

	return 0;
}