#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <endian.h>
#include "hx711.hpp"

#define SYSERR(expr) (([&](){ const auto r = ((expr)); if( (long)r == -1L ) { perror(#expr); throw #expr; } else return r; })())

namespace hx711
{
	TFileDescriptor::TFileDescriptor(const int fd) : fd(fd)
	{
	}

	TFileDescriptor::TFileDescriptor(const TFileDescriptor& other) : fd(SYSERR(dup(other.fd)))
	{
	}

	TFileDescriptor::TFileDescriptor(TFileDescriptor&& other) : fd(other.fd)
	{
		other.fd = -1;
	}

	TFileDescriptor::~TFileDescriptor()
	{
		if(this->fd != -1)
			if(close(this->fd) == -1)
				perror("TFileDescriptor::~TFileDescriptor(): failed to close fd in destructor");
	}

	void Hexdump(const void* const buffer, const unsigned sz)
	{
		for(unsigned i = 0; i < sz; i++)
			printf("%02hhx ", ((const unsigned char*)buffer)[i]);
		putchar('\n');
	}

	static uint32_t Discard2Bit(uint64_t in)
	{
		uint32_t out = 0;
		for(unsigned i = 0; i < 32; i++)
		{
			out <<= 1;
			out |= (((in & 0x8000000000000000) != 0) ? 1 : 0);
			in <<= 2;
		}
		return out;
	}

	static const uint64_t hz_clock = 1000UL * 1000UL;

	uint32_t THX711::GetSensorValue()
	{
		const unsigned n_reset_bytes = (hz_clock / 10000UL) * 2;
		const unsigned n_data_bytes = 6;
		char buffer[n_reset_bytes + n_data_bytes];

// 		fprintf(stderr, "DEBUG: hz_clock = %u KHz\n", hz_clock / 1000UL);
// 		fprintf(stderr, "DEBUG: n_reset_bytes = %u\n", n_reset_bytes);

		// 0x00 => HIGH => reset
		memset(buffer, 0x00, n_reset_bytes);

		// 0xaa => clock signal (10101010) => data transfer
		memset(buffer + n_reset_bytes, 0xaa, n_data_bytes);

		struct spi_ioc_transfer xfer_cmd;
		memset(&xfer_cmd, 0, sizeof(xfer_cmd));
		xfer_cmd.tx_buf = (size_t)buffer;
		xfer_cmd.rx_buf = (size_t)buffer;
		xfer_cmd.len = sizeof(buffer);
		xfer_cmd.delay_usecs = 0;
		xfer_cmd.speed_hz = hz_clock;
		xfer_cmd.bits_per_word = 8;
		SYSERR(ioctl(this->fd, SPI_IOC_MESSAGE(1), &xfer_cmd));

		if(buffer[n_reset_bytes - 1] == 0xff)
			throw "HX711::GetSensorValue(): no data available yet";

		uint64_t raw = 0;
		uint32_t out = 0;
		memcpy(&raw, buffer + n_reset_bytes, 6);
		out = Discard2Bit(raw);
 		out = be32toh(out);

		return out;
	}

	double THX711::GetSensorValueFloat()
	{
		const double v = this->GetSensorValue();
		return v / (double)UINT32_MAX;
	}

	THX711::THX711(const char* const spidev) : fd(SYSERR(open(spidev, O_RDWR | O_CLOEXEC | O_SYNC)))
	{
	}

	THX711::~THX711()
	{
	}
}

