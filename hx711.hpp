#pragma once

#include <stdint.h>

namespace hx711
{
	struct TFileDescriptor
	{
		public:
			int fd;

			inline operator int() const { return this->fd; }

			TFileDescriptor(const int fd = -1);
			TFileDescriptor(const TFileDescriptor&);
			TFileDescriptor(TFileDescriptor&&);
			~TFileDescriptor();
	};

	// this is a pretty simple wrapper driver for the SPI bus
	// all it does it handle the open/close and data transfer calls to the kernel
	class THX711
	{
		private:
			THX711(const THX711&);

		protected:
			TFileDescriptor fd;

		public:
			uint32_t GetSensorValue();
			double GetSensorValueFloat();

			THX711(const char* const spidev);
			~THX711();
	};
}
