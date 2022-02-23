#include <stdint.h>

void variableEepromLoad();
int32_t getVariableValue();
void setVarValue(int32_t newValue, int32_t *value);
void runSetVarValue(uint8_t id);
void freeVariables();