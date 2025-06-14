/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file cubered_io.cpp
 *
 * CubeRed IO Module - PX4IO simulation for CubeRed Secondary
 *
 * This module simulates PX4IO functionality on the CubeRed Secondary
 * by providing low-latency serial communication on ttyS4.
 */

#include "stm32_uart.h"
#include "cubered_io.h"


#undef PX4_DEBUG
#define PX4_DEBUG PX4_INFO


// Use the CRC function from protocol.h

// Configuration page data (simulating PX4IO)
static const uint16_t config_page[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= PX4IO_PROTOCOL_VERSION,
	[PX4IO_P_CONFIG_HARDWARE_VERSION]	= 2,	// Simulate PX4IOv2
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= 5,	// Bootloader version
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= PX4IO_MAX_TRANSFER_LEN,
	[PX4IO_P_CONFIG_CONTROL_COUNT]		= PX4IO_PROTOCOL_MAX_CONTROL_COUNT,
	[PX4IO_P_CONFIG_ACTUATOR_COUNT]		= 8,	// Actuator outputs
	[PX4IO_P_CONFIG_RC_INPUT_COUNT]		= 18,	// RC input channels
	[PX4IO_P_CONFIG_ADC_INPUT_COUNT]	= 3,	// ADC inputs
};

CuberedIO::CuberedIO()
{
}

CuberedIO::~CuberedIO()
{
	if (_serial_fd >= 0) {
		::close(_serial_fd);
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

int CuberedIO::task_spawn(int argc, char *argv[])
{
	CuberedIO *instance = new CuberedIO();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("cubered_io",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_MAX - 5, // High priority for low latency
					      2048,
					      (px4_main_t) &CuberedIO::run_trampoline,
					      (char *const *)argv);

		if (_task_id < 0) {
			PX4_ERR("task start failed");
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}

		return PX4_OK;

	} else {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}
}

int CuberedIO::run_trampoline(int argc, char *argv[])
{
	CuberedIO *instance = reinterpret_cast<CuberedIO *>(_object.load());

	if (instance) {
		if (instance->init()) {
			instance->run();
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
	}

	return 0;
}

bool CuberedIO::init()
{
	if (init_serial() != PX4_OK) {
		PX4_ERR("Failed to initialize serial port");
		return false;
	}

	PX4_INFO("CuberedIO initialized on %s", DEVICE_NAME);
	return true;
}

int CuberedIO::init_serial()
{
	// Open serial port
	_serial_fd = ::open(DEVICE_NAME, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		PX4_ERR("Failed to open %s: %s", DEVICE_NAME, strerror(errno));
		return PX4_ERROR;
	}

	// Configure serial port
	termios uart_config;

	if (tcgetattr(_serial_fd, &uart_config) < 0) {
		PX4_ERR("Failed to get termios config: %s", strerror(errno));
		::close(_serial_fd);
		_serial_fd = -1;
		return PX4_ERROR;
	}

	// Set baud rate
	if (cfsetispeed(&uart_config, BAUDRATE) < 0 || cfsetospeed(&uart_config, BAUDRATE) < 0) {
		PX4_ERR("Failed to set baudrate: %s", strerror(errno));
		::close(_serial_fd);
		_serial_fd = -1;
		return PX4_ERROR;
	}

	// After cfsetispeed/cfsetospeed
	printf("Secondary: Configured for B1000000\n");
	// If you can access the hardware registers:
	#define PX4IO_SERIAL_BASE STM32_UART7_BASE
	#define REG(_x)   (*(volatile uint32_t *)(PX4IO_SERIAL_BASE + (_x)))
	#define rBRR    REG(STM32_USART_BRR_OFFSET)
	printf("Secondary: Actual BRR register = 0x%lx\n", rBRR);
	// Read the UART control register directly

	uint32_t cr1 = getreg32(STM32_USART3_CR1);
	printf("Primary USART3 CR1: 0x%08lx\n", cr1);
	printf("Primary USART3 OVER8 bit: %s\n", (cr1 & USART_CR1_OVER8) ? "8x oversampling" : "16x oversampling");

	printf("USART3 CR1: 0x%08lx\n", cr1);
	printf("Bit 21 value: %s\n", (cr1 & (1<<21)) ? "SET (8x)" : "CLEAR (16x)");
	printf("USART_CR1_OVER8 constant: 0x%08x\n", USART_CR1_OVER8);

	cr1 = getreg32(STM32_UART7_CR1);
	printf("Primary UART7 CR1: 0x%08lx\n", cr1);
	printf("Primary UART7 OVER8 bit: %s\n", (cr1 & USART_CR1_OVER8) ? "8x oversampling" : "16x oversampling");

	printf("UART7 CR1: 0x%08lx\n", cr1);
	printf("Bit 21 value: %s\n", (cr1 & (1<<21)) ? "SET (8x)" : "CLEAR (16x)");
	printf("USART_CR1_OVER8 constant: 0x%08x\n", USART_CR1_OVER8);

	uint32_t console_brr = getreg32(STM32_USART3_BRR);
	printf("Console UART BRR: 0x%lx (%ld)\n", console_brr, console_brr);

	// Check USART234578 clock source (includes both USART3 and UART7)
	uint32_t d2ccip2r = getreg32(STM32_RCC_D2CCIP2R);
	uint32_t usart_sel = (d2ccip2r & RCC_D2CCIP2R_USART234578SEL_MASK) >> RCC_D2CCIP2R_USART234578SEL_SHIFT;
	printf("USART234578 clock source: %lu\n", usart_sel);

// Compare ALL registers between USART3 and UART7
printf("USART3 CR1: 0x%08lx\n", getreg32(STM32_USART3_CR1));
printf("USART3 CR2: 0x%08lx\n", getreg32(STM32_USART3_CR2));
printf("USART3 CR3: 0x%08lx\n", getreg32(STM32_USART3_CR3));
printf("USART3 BRR: 0x%08lx\n", getreg32(STM32_USART3_BRR));

printf("UART7 CR1: 0x%08lx\n", getreg32(STM32_UART7_CR1));
printf("UART7 CR2: 0x%08lx\n", getreg32(STM32_UART7_CR2));
printf("UART7 CR3: 0x%08lx\n", getreg32(STM32_UART7_CR3));
printf("UART7 BRR: 0x%08lx\n", getreg32(STM32_UART7_BRR));

	// Configure for raw mode
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_oflag &= ~OPOST;
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_cflag &= ~(CSIZE | PARENB);
	uart_config.c_cflag |= CS8;

	// Set timeouts for non-blocking read
	uart_config.c_cc[VMIN] = 6;
	uart_config.c_cc[VTIME] = 1;

	if (tcsetattr(_serial_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("Failed to set termios config: %s", strerror(errno));
		::close(_serial_fd);
		_serial_fd = -1;
		return PX4_ERROR;
	}

	// Flush any existing data
	tcflush(_serial_fd, TCIOFLUSH);

	return PX4_OK;
}

void CuberedIO::run()
{
	PX4_INFO("CuberedIO task started - simulating PX4IO on CubeRed Secondary");

	while (!should_exit()) {
		perf_begin(_loop_perf);
		perf_count(_loop_interval_perf);

		poll_and_process();

		perf_end(_loop_perf);
	}

	PX4_INFO("CuberedIO task exiting");
}

void CuberedIO::poll_and_process()
{
	IOPacket packet {};
	size_t bytes_read = 0;

	while (bytes_read < sizeof(IOPacket)) {
		pollfd fds[1];
		fds[0].fd = _serial_fd;
		fds[0].events = POLLIN;

		int ret = poll(fds, 1, POLL_TIMEOUT_MS);

		if (ret > 0) {
			if (fds[0].revents & POLLIN) {
				// Data available to read from CubeRed Primary
				int read_ret = ::read(_serial_fd, (reinterpret_cast<uint8_t*>(&packet)) + bytes_read, sizeof(IOPacket) - bytes_read);

				if (read_ret > 0) {
					printf("read from %d to %d\n", bytes_read, bytes_read + read_ret);

					//printf("packet bytes: ");
					//for (unsigned i = 0; i < sizeof(IOPacket); i++) {
					//	printf("%02x ", ((uint8_t *)&packet)[i]);
					//}
					//printf("\n");
					bytes_read += read_ret;

					// Check if packet has valid CRC
					if (validate_crc(packet)) {
						printf("CRC OK at %d\n", bytes_read);
						process_received_data(packet);
						return;
					} else {
						printf("CRC failed at %d\n", bytes_read);
						// Continue loop to read more data
					}
				} else if (read_ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
					PX4_ERR("Read error: %s", strerror(errno));
					return;
				}
			}
		} else if (ret < 0 && errno != EINTR) {
			PX4_ERR("Poll error: %s", strerror(errno));
			return;
		} else {
			// Timeout, start fresh.
			return;
		}
	}
}

void CuberedIO::process_received_data(IOPacket &packet)
{
	uint8_t code = PKT_CODE(packet);
	//uint8_t count = PKT_COUNT(packet);

	if (code == PKT_CODE_READ) {
		handle_read_request(packet);
	} else if (code == PKT_CODE_WRITE) {
		handle_write_request(packet);
	} else {
		PX4_DEBUG("Unknown packet code: 0x%02x", code);
		send_error_response();
	}

	//PX4_DEBUG("Received packet: page=%u, offset=%u, code=0x%02x, count=%u",
	//	 packet.page, packet.offset, code, count);

}

void CuberedIO::send_response(IOPacket &packet)
{
	if (_serial_fd >= 0) {
		size_t packet_size = PKT_SIZE(packet);
		ssize_t bytes_written = ::write(_serial_fd, &packet, packet_size);

		if (bytes_written != (ssize_t)packet_size) {
			if (bytes_written < 0) {
				PX4_ERR("Write error: %s", strerror(errno));
			} else {
				PX4_WARN("Partial write: %zd of %zu bytes", bytes_written, packet_size);
			}
		} else {
			//PX4_DEBUG("Sent %zu bytes to CubeRed Primary", packet_size);
		}
	}
}

int CuberedIO::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CubeRed IO Module - PX4IO simulation for CubeRed Secondary.

This module simulates PX4IO functionality on the CubeRed Secondary by providing
low-latency serial communication on ttyS4 with the CubeRed Primary.

The module runs as a dedicated task (not in work queue) to provide low-latency
polling and response on the serial port, mimicking the behavior of a real PX4IO
coprocessor.

### Implementation
The module runs in its own high-priority task and uses poll() with a short timeout
to achieve low latency communication with the CubeRed Primary. It processes
incoming commands and responds according to the PX4IO protocol.

### Examples
Start the cubered-io module:
$ cubered_io start

Stop the cubered-io module:
$ cubered_io stop

Check status:
$ cubered_io status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cubered_io", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool CuberedIO::validate_crc(IOPacket &packet)
{
	uint8_t expected_crc = packet.crc;
	packet.crc = 0;
	uint8_t calculated_crc = calculate_crc(packet);
	packet.crc = expected_crc;

	return expected_crc == calculated_crc;
}

uint8_t CuberedIO::calculate_crc(IOPacket &packet)
{
	return crc_packet(&packet);
}

void CuberedIO::handle_read_request(IOPacket &packet)
{
	//PX4_DEBUG("Read request: page=%u, offset=%u, count=%u",
	//	 packet.page, packet.offset, PKT_COUNT(packet));

	// Handle config page reads (version info)
	if (packet.page == PX4IO_PAGE_CONFIG) {
		uint8_t offset = packet.offset;
		uint8_t count = PKT_COUNT(packet);

		// Prepare response packet
		IOPacket response;
		memset(&response, 0, sizeof(response));
		response.count_code = count | PKT_CODE_SUCCESS;
		response.page = packet.page;
		response.offset = packet.offset;

		// Copy requested config data
		for (int i = 0; i < count && (offset + i) < 8; i++) {
			response.regs[i] = config_page[offset + i];
		}

		//PX4_DEBUG("Sending config data: version=%u, hw_version=%u",
		//	 config_page[PX4IO_P_CONFIG_PROTOCOL_VERSION],
		//	 config_page[PX4IO_P_CONFIG_HARDWARE_VERSION]);

		send_packet(response);
	} else {
		// Unsupported page
		PX4_DEBUG("Unsupported page: %u", packet.page);
		send_error_response();
	}
}

void CuberedIO::handle_write_request(IOPacket &packet)
{
	PX4_DEBUG("Write request: page=%u, offset=%u, count=%u",
		 packet.page, packet.offset, PKT_COUNT(packet));

	// For now, just acknowledge write requests
	// TODO: Implement actual register writes as needed
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = 0 | PKT_CODE_SUCCESS;
	response.page = packet.page;
	response.offset = packet.offset;

	send_packet(response);
}

void CuberedIO::send_corrupt_response()
{
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = 0 | PKT_CODE_CORRUPT;
	send_packet(response);
}

void CuberedIO::send_error_response()
{
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = 0 | PKT_CODE_ERROR;
	send_packet(response);
}

void CuberedIO::send_packet(IOPacket &packet)
{
	// Calculate and set CRC
	packet.crc = calculate_crc(packet);

	// Calculate packet size
	size_t packet_size = 4 + (PKT_COUNT(packet) * 2); // header + registers

	PX4_DEBUG("Send: page=%u, offset=%u, count=%u, size=%zu",
		 packet.page, packet.offset, PKT_COUNT(packet), packet_size);

	send_response(packet);
}

extern "C" __EXPORT int cubered_io_main(int argc, char *argv[])
{
	return CuberedIO::main(argc, argv);
}
