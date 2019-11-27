#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "hx711.hpp"

int main()
{
	try
	{
		using namespace hx711;

		THX711 hx711("/dev/spidev0.0");

		hx711::DEBUG = true;

		while(true)
		{
			try
			{
				const double v = hx711.GetSensorValueFloat();
				printf("val = %lf\n", v);
			}
			catch(const char* err)
			{
				fprintf(stderr, "WARN: %s\n", err);
			}
			usleep(200000);
		}

		return 0;
	}
	catch(const char* err)
	{
		perror(err);
		return 1;
	}
	return 2;
}
