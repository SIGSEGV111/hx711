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
	bool DEBUG = false;

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

	void Hexdump(const char* const title, const void* const buffer, const unsigned sz)
	{
		fprintf(stderr, "DEBUG: %s: ", title);
		for(unsigned i = 0; i < sz; i++)
			fprintf(stderr, "%02hhx ", ((const unsigned char*)buffer)[i]);
		fputc('\n', stderr);
	}

	static uint32_t Discard2Bit(uint64_t in)
	{
		uint32_t out = 0;
		for(unsigned i = 0; i < 32; i++)
		{
			const bool b1 = ((in & 0x8000000000000000) != 0);
			const bool b2 = ((in & 0x4000000000000000) != 0);
			if(b1 != b2)
				throw "Discard2Bit(): communication error detected (b1 != b2)";

			out <<= 1;
			out |= (b1 ? 1 : 0);
			in <<= 2;
		}
		return out;
	}

	static constexpr unsigned long CalculateResetBytes(const unsigned long hz_clock_spi, const unsigned long us_reset_time)
	{
		unsigned long t = (hz_clock_spi * us_reset_time) / 1000000UL / 8UL;
		if(t * 1000000UL * 8UL != hz_clock_spi * us_reset_time)
			t++;
		return t;
	}

	// the HX711 will reset once the clock is HIGH for more than 60µs
	// here we calculate how many 0x00 bytes (which lead to HIGH level on the SPI data PIN)
	// we have to sent, in order for the HX711 to reset
	// this depends on the clock speed, more ticks mean more bytes to send
	// so high clock speeds really don't buy us anything as we have to wait for a fixed time
	// and we only waste memory for large buffers on high clock speeds
	// so essentially we want the lowest possible clock speed, which does not cause troubles
	// since for the purpose of data transfer the effective clock speed is half the SPI clock (0xAA pattern)
	// and the lowest allowed effective clock speed is 20 KHz, we cannot go below 40 KHz SPI clock
	// we choose to wait for 100µs instead of only 60µs (makes the math simpler and adds to reliability)
	// the clock speed of 160KHz is choose so that it leads exactly to 2 reset-bytes
	// combined with the 6 data-bytes, this makes a nice 8 bytes buffer ;-)
	static const unsigned long HZ_CLOCK_SPI = 160UL * 1000UL;
	static const unsigned long US_RESET_TIME = 100;
	static const unsigned long N_RESET_BYTES = CalculateResetBytes(HZ_CLOCK_SPI, US_RESET_TIME);

	uint32_t THX711::GetSensorValue()
	{
		// we have to send 10101010 patterns as data, we need two bits per effective clock cycle
		// since we need to pull 24 bits (3 bytes) from the chip, we have to allocate 48 bits (6 bytes) for the clock pattern
		const unsigned n_data_bytes = 6;

		// we send reset and clock in once go from one buffer
		char buffer[N_RESET_BYTES + n_data_bytes];

		if(DEBUG /* see hx711.hpp */)
		{
			const unsigned hz_clock_data = HZ_CLOCK_SPI / 2UL;
			fprintf(stderr, "DEBUG: HZ_CLOCK_SPI   = %u KHz\n", HZ_CLOCK_SPI / 1000UL);
			fprintf(stderr, "DEBUG: US_RESET_TIME  = %u µs\n", US_RESET_TIME);
			fprintf(stderr, "DEBUG: N_RESET_BYTES  = %u bytes\n", N_RESET_BYTES);
			fprintf(stderr, "DEBUG: hz_clock_data  = %u KHz\n", hz_clock_data / 1000UL);
			fprintf(stderr, "DEBUG: n_data_bytes   = %u bytes\n", n_data_bytes);
			fprintf(stderr, "DEBUG: sizeof(buffer) = %u bytes\n", sizeof(buffer));
		}

		// generate the reset pattern: N_RESET_BYTES * 0x00 (HIGH) => reset
		memset(buffer, 0x00, N_RESET_BYTES);

		// generate the data clock pattern: n_data_bytes * 0xaa (clock signal; 10101010) => data transfer
		memset(buffer + N_RESET_BYTES, 0xaa, n_data_bytes);

		if(DEBUG) Hexdump("tx-buffer", buffer, sizeof(buffer));

		struct spi_ioc_transfer xfer_cmd;
		memset(&xfer_cmd, 0, sizeof(xfer_cmd));
		xfer_cmd.tx_buf = (size_t)buffer;
		xfer_cmd.rx_buf = (size_t)buffer;
		xfer_cmd.len = sizeof(buffer);
		xfer_cmd.delay_usecs = 0;
		xfer_cmd.speed_hz = HZ_CLOCK_SPI;
		xfer_cmd.bits_per_word = 8;
		SYSERR(ioctl(this->fd, SPI_IOC_MESSAGE(1), &xfer_cmd));

		if(DEBUG) Hexdump("rx-buffer", buffer, sizeof(buffer));

		if(buffer[N_RESET_BYTES - 1] == 0xff)
			throw "HX711::GetSensorValue(): no data available yet";

		uint64_t raw = 0;
		uint32_t out = 0;
		memcpy(&raw, buffer + N_RESET_BYTES, 6);
		out = Discard2Bit(raw);
		if(DEBUG) Hexdump("after discard", &out, sizeof(out));
 		out = be32toh(out);
		if(DEBUG) Hexdump("host endianity", &out, sizeof(out));
		if(DEBUG) fprintf(stderr, "DEBUG: sensor-value = %u\n", out);

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

