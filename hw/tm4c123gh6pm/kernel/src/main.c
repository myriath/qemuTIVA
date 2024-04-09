#include "main.h"

int a = 0;

void fault_test(void)
{
    a++;
    ExitQEMU();
}

/**
 * Main function. Write your code here
*/
int main() 
{
    IntRegister(-13, fault_test);
    __asm__("b 0x30000000");
    return 0;
}
