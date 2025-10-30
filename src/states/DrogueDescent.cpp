#include <State.hpp>
#include <stdio.h>

void drogueDescentInit(StateData *data) {
    printf("Drogue Started\n");
}

StateID drogueDescentLoop (StateData* data, Context* ctx) {
   return MAIN_DESCENT; 
}