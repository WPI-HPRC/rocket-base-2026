#include <State.hpp>
#include <stdio.h>

void mainDescentInit(StateData *data) {
    printf("Main Descent Started\n");
}

StateID mainDescentLoop (StateData* data, Context* ctx) {
   return RECOVERY; 
}