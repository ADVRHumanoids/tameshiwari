#include <tameshiwari/talker.h>
#include <iostream>

namespace tameshiwari
{

Talker::Talker(std::string name):
    _name(name)
{

}

void Talker::say_hello()
{
    std::cout << "Ciao " << _name << "!" << std::endl;
}

    
}
