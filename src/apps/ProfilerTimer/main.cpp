#include "../../core/profilertimer.h"

#define BIG_NUM 1000000000
int main (int argc, char **argv) {

    ProfilerTimer pt;
    ProfilerTimer ptIdentity("Test");

    for (int i = 0; i < BIG_NUM; ++i);

    return 0;
}
