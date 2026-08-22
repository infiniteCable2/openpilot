#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>


#define TIMEOUT 0
#define SPI_BUF_SIZE 2048U
#define SPI_HEADER_SIZE 7U
#define SPI_TRANSACTION_ID_SIZE 8U
#define SPI_RESPONSE_FRAME_SIZE 4U
#define SPI_BUFFER_RESERVE 0x40U
#define SPI_REALIGN_FRAME_SIZE (SPI_HEADER_SIZE + 1U)
#define SPI_MAX_DATA_SIZE (SPI_BUF_SIZE - SPI_BUFFER_RESERVE)
#define SPI_RECOVERY_TRANSFER_SIZE (SPI_BUF_SIZE / 2U)

// ICSP layout contract: this C++ host intentionally uses a smaller local
// buffer than the firmware/Python transport. That is wire-compatible because
// each transfer advertises its window, provided every local frame fits and
// recovery ends on a NACK/header boundary. Framing changes require a protocol
// version bump and the SPI protocol stress suite.
static_assert(SPI_BUF_SIZE > SPI_BUFFER_RESERVE, "SPI buffer must exceed its protocol reserve");
static_assert((SPI_TRANSACTION_ID_SIZE + SPI_MAX_DATA_SIZE + 1U) <= SPI_BUF_SIZE,
              "SPI request frame exceeds the host buffer");
static_assert((SPI_MAX_DATA_SIZE + SPI_RESPONSE_FRAME_SIZE + SPI_HEADER_SIZE) <= SPI_BUF_SIZE,
              "SPI response and pipelined header exceed the host buffer");
static_assert((SPI_RECOVERY_TRANSFER_SIZE % SPI_REALIGN_FRAME_SIZE) == 0U,
              "SPI recovery transfer must end on a NACK/header boundary");


class PandaSpiHandle {
public:
  std::string hw_serial;
  std::atomic<bool> connected = true;
  std::atomic<bool> comms_healthy = true;

  PandaSpiHandle(std::string serial);
  ~PandaSpiHandle();

  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup();

  static std::vector<std::string> list();

private:
  int spi_fd = -1;
  uint8_t tx_buf[SPI_BUF_SIZE];
  uint8_t rx_buf[SPI_BUF_SIZE];
  inline static std::recursive_mutex hw_lock;

  struct __attribute__((packed)) spi_header {
    uint8_t sync;
    uint8_t endpoint;
    uint16_t tx_len;
    uint16_t max_rx_len;
  };
  static_assert((sizeof(spi_header) + 1U) == SPI_HEADER_SIZE, "SPI header layout mismatch");

  int wait_for_ack(uint8_t ack, uint8_t tx, unsigned int timeout, unsigned int length);
  int bulk_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len, unsigned int timeout);
  int spi_transfer(uint64_t transaction_id, uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len,
                   uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int spi_transfer_retry(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int lltransfer(struct spi_ioc_transfer &t);

  spi_header header;
  uint32_t xfer_count = 0;
  uint64_t next_transaction_id;
};
