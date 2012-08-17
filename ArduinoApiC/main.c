#include "zoSmsMaster.h"

#define SUPERMODIFIED_NODE_ID	0x04

int main(void)
{
	zoSmsMasterInitI2c();
	zoCommandStart(SUPERMODIFIED_NODE_ID);
	
	while(1);
		

	return 1;
}