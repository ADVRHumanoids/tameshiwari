#ifndef __TAMESHIWARI_TALKER_H__
#define __TAMESHIWARI_TALKER_H__

#include <string>

namespace tameshiwari
{

    class Talker
    {
    
    public:
    
        Talker(std::string name);
	
	    void say_hello();
	    
	private:
	
	    std::string _name;
	
	};

}

#endif
