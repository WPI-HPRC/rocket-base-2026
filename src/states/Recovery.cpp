#include <State.hpp>
#include <stdio.h>

void recoveryInit(StateData *data) {
    printf("Recovery Started\n");
}

StateID recoveryLoop (StateData* data, Context* ctx) {
   return RECOVERY; 
}