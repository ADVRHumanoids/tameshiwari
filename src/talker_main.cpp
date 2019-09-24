#include <tameshiwari/talker.h>

int main(int argc, char ** argv)
{
    tameshiwari::Talker talker("Paul");
    talker.say_hello();

    return 0;
}
