// sh2_hal_linux.cpp 
#include "sh2_hal_linux.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef BNO085_I2C_ADDR
#define BNO085_I2C_ADDR 0x4A
#endif
#ifndef I2C_BUS
#define I2C_BUS "/dev/i2c-1"
#endif

static int hal_open(sh2_Hal_t *self) {
    linux_hal_t *ctx = reinterpret_cast<linux_hal_t*>(self);
    if (!ctx) return -1;

    if (ctx->fd >= 0) return 0; // already open

    const char *bus = ctx->i2c_bus ? ctx->i2c_bus : I2C_BUS;
    ctx->fd = open(bus, O_RDWR);
    if (ctx->fd < 0) {
        std::perror("open i2c bus");
        return -1;
    }

    int addr = ctx->i2c_addr ? ctx->i2c_addr : BNO085_I2C_ADDR;
    if (ioctl(ctx->fd, I2C_SLAVE, addr) < 0) {
        std::perror("ioctl I2C_SLAVE");
        close(ctx->fd);
        ctx->fd = -1;
        return -1;
    }

    // small delay to allow device to settle
    usleep(10000);
    return 0;
}

static void hal_close(sh2_Hal_t *self) {
    linux_hal_t *ctx = reinterpret_cast<linux_hal_t*>(self);
    if (!ctx) return;
    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }
}

/* HAL read: the CEVA SH2 library expects read() to return a full SHTP transfer
 * when available, or 0 if none. For typical I2C SHTP, the device provides a
 * length header followed by the payload. We'll perform a blocking read of
 * the first 2-4 bytes then the remainder. We keep this minimal — it works
 * for the BNO08x breakouts that present SHTP over I2C.
 */
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    linux_hal_t *ctx = reinterpret_cast<linux_hal_t*>(self);
    if (!ctx || ctx->fd < 0) return 0;

    // Read the first 4 bytes (SHTP header contains 2-byte length + other fields)
    std::cerr << "[HAL] Attempting read...\n";
    ssize_t n = read(ctx->fd, pBuffer, 4);
    if (n <= 0) {
        std::cerr << "[HAL] read returned " << n << " errno=" << errno << "\n";
        return 0;
    }
    if (n != 4) {
        // try to read remaining bytes of header (rare)
        ssize_t rem = 4 - n;
        ssize_t r2 = read(ctx->fd, pBuffer + n, rem);
        if (r2 != rem) return 0;
    }
    

    uint16_t pktLen = (uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8);
    if (pktLen == 0 || pktLen > (int)len) {
        return 0;
    }

    int remaining = pktLen - 4;
    if (remaining > 0) {
        ssize_t r = read(ctx->fd, pBuffer + 4, remaining);
        if (r != remaining) return 0;
    }

    uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (t_us) *t_us = (uint32_t)(now & 0xFFFFFFFF);

    return pktLen;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    linux_hal_t *ctx = reinterpret_cast<linux_hal_t*>(self);
    if (!ctx || ctx->fd < 0) return 0;

    ssize_t w = write(ctx->fd, pBuffer, len);
    if (w < 0) {
        std::perror("i2c write");
        return 0;
    }
    return (int)w;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    (void)self;
    uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return (uint32_t)(now & 0xFFFFFFFF);
}

void sh2_hal_linux_init(linux_hal_t *ctx) {
    if (!ctx) return;
    std::memset(ctx, 0, sizeof(*ctx));
    ctx->fd = -1;
    ctx->i2c_bus = I2C_BUS;
    ctx->i2c_addr = BNO085_I2C_ADDR;

    ctx->hal.open = hal_open;
    ctx->hal.close = hal_close;
    ctx->hal.read = hal_read;
    ctx->hal.write = hal_write;
    ctx->hal.getTimeUs = hal_getTimeUs;
}
