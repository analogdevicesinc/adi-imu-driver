# %%
import spidev
from time import sleep

class ImuSpiDriver:
    def __init__(self, bus, device):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 4000000
        self.spi.mode = 3
        self.spi.bits_per_word = 8
        self.cur_page = 0x00
        self.spi_deb_print = False
        self.set_page(0x00)
        print("Product ID = ", self.reg_read(0x007E))

    def trx(self, buf, xfer_len, num_transfers, repeat_tx = False):
        if (len(buf) % xfer_len != 0):
            print("Length of input should be multiple of transfer length (Array length {} != Xfer length {}".format(len(buf), xfer_len))
        cur_idx = 0
        num_xfer = 0
        out = []
        for i in range(num_transfers):
            if self.spi_deb_print:
                print("SPI TX: {}".format([hex(x) for x in buf[cur_idx : cur_idx + xfer_len]]))
            recv = self.spi.xfer(buf[cur_idx : cur_idx + xfer_len])
            if self.spi_deb_print:
                print("SPI RX: {}".format([hex(x) for x in recv]))
            out += recv
            if (not repeat_tx):
                cur_idx += xfer_len
        return out

    def set_page(self, pg_id):
        self.trx([0x80, 0x00 | pg_id], 2, 1)

    def reg_read(self, reg_addr):
        pg_id = (reg_addr & 0xFF00) >> 8
        regadd = reg_addr & 0xFF
        if (pg_id != self.cur_page):
            self.trx([0x80, 0x00 | pg_id], 2, 1)
            self.cur_page = pg_id
        reg_val = self.trx([regadd, 0x00, 0x00, 0x00], 2, 2)
        return (reg_val[-2] << 8 | reg_val[-1])

    def reg_write(self, reg_addr, val):
        pg_id = (reg_addr & 0xFF00) >> 8
        regadd = reg_addr & 0xFF
        if (pg_id != self.cur_page):
            self.trx([0x80, 0x00 | pg_id], 2, 1)
            self.cur_page = pg_id
        return self.trx([0x80 | regadd, val & 0xFF, 0x80 | (regadd + 1), ((val >> 8) & 0xFF)], 2, 2)

# %%
imu = ImuSpiDriver(1, 0)

# %%
# BUFFER BOARD SOFT RESET
# imu.reg_write(0xFD16, 0x8000)
# sleep(1)
# print("IMU BUF SOFT Reset")

# %%
# Clear buffer
imu.reg_write(0xFD16, 0x0001)
sleep(1)
print("IMU BUF BUFFER cleared")

# %%
# Clear Fault
# imu.reg_write(0xFD16, 0x0002)
# sleep(1)
# print("IMU BUF Fault cleared")

# %%
# IMU spi config 
imu.reg_write(0xFD10, 0x0805)
print("IMU SPI Config = ", hex(imu.reg_read(0xFD10)))

# %%
# Set IMU data rate
outputRate = 2000
decRate = int((4000 / outputRate) - 1);
imu.reg_write(0x030C, decRate)
print("IMU Data Rate = ", 4000 / (imu.reg_read(0x030C) + 1))

# %%
# Set IMU DIO config
imu.reg_write(0x0306, 0x000C)
print("IMU DIO Config = ", imu.reg_read(0x0306))

# %%
# Set IMU page back to 0 for operation
imu.reg_write(0x0000, 0x0000)
print("IMU page was reset to 0")

# %%
# IMU DIO Input config 
imu.reg_write(0xFD08, 0x0011)
print("IMU DIO input Config = ", hex(imu.reg_read(0xFD08)))

# %%
# IMU DIO Output config 
imu.reg_write(0xFD0A, 0x0020)
print("IMU DIO output Config = ", hex(imu.reg_read(0xFD0A)))

# %%
# Set buffer config settings
en_imu_burst = 1
en_user_burst = 1

buf_config = 0x0
if (en_imu_burst):
    buf_config |= 0x2
if (en_user_burst):
    buf_config |= 0x4

imu.reg_write(0xFD02, buf_config)
print("IMU BUF config = ", hex(imu.reg_read(0xFD02)))

# %%
# Save buffer config settings to flash and reset the board
imu.reg_write(0xFD16, 0x0008)
print("Saved buffer config to flash")
sleep(0.1)
imu.reg_write(0xFD16, 0x8000)
print("Sent reset command to buffer board")
sleep(0.1)

# %%
# Check that the buffer board is still responding after a reset
print("Buffer board firmware rev = ", imu.reg_read(0xFD28))

# %%
# IMU BUF Config 
imu.spi_deb_print = False
if (en_imu_burst):
    buf_length = 40 # bytes
    imu.reg_write(0xFE12, 0x7C00)
    imu.reg_write(0xFE14, 0x0000)
    imu.reg_write(0xFE16, 0x0000)
    imu.reg_write(0xFE18, 0x0000)
    imu.reg_write(0xFE1A, 0x0000)
    imu.reg_write(0xFE1C, 0x0000)
    imu.reg_write(0xFE1E, 0x0000)
    imu.reg_write(0xFE20, 0x0000)
    imu.reg_write(0xFE22, 0x0000)
    imu.reg_write(0xFE24, 0x0000)
    imu.reg_write(0xFE26, 0x0000)
    imu.reg_write(0xFE28, 0x0000)
    imu.reg_write(0xFE2A, 0x0000)
    imu.reg_write(0xFE2C, 0x0000)
    imu.reg_write(0xFE2E, 0x0000)
    imu.reg_write(0xFE30, 0x0000)
    imu.reg_write(0xFE32, 0x0000)
    imu.reg_write(0xFE34, 0x0000)
    imu.reg_write(0xFE36, 0x0000)
    imu.reg_write(0xFE38, 0x0000)
    print("IMU BUF Write[0] = ", hex(imu.reg_read(0xFE12)))
    print("IMU BUF Write[1] = ", hex(imu.reg_read(0xFE14)))
    print("IMU BUF Write[2] = ", hex(imu.reg_read(0xFE16)))
    print("IMU BUF Write[3] = ", hex(imu.reg_read(0xFE18)))
    print("IMU BUF Write[4] = ", hex(imu.reg_read(0xFE1A)))
    print("IMU BUF Write[5] = ", hex(imu.reg_read(0xFE1C)))
    print("IMU BUF Write[6] = ", hex(imu.reg_read(0xFE1E)))
    print("IMU BUF Write[7] = ", hex(imu.reg_read(0xFE20)))
    print("IMU BUF Write[8] = ", hex(imu.reg_read(0xFE22)))
    print("IMU BUF Write[9] = ", hex(imu.reg_read(0xFE24)))
    print("IMU BUF Write[10] = ", hex(imu.reg_read(0xFE26)))
    print("IMU BUF Write[11] = ", hex(imu.reg_read(0xFE28)))
    print("IMU BUF Write[12] = ", hex(imu.reg_read(0xFE2A)))
    print("IMU BUF Write[13] = ", hex(imu.reg_read(0xFE2C)))
    print("IMU BUF Write[14] = ", hex(imu.reg_read(0xFE2E)))
    print("IMU BUF Write[15] = ", hex(imu.reg_read(0xFE30)))
    print("IMU BUF Write[16] = ", hex(imu.reg_read(0xFE32)))
    print("IMU BUF Write[17] = ", hex(imu.reg_read(0xFE34)))
    print("IMU BUF Write[18] = ", hex(imu.reg_read(0xFE36)))
    print("IMU BUF Write[19] = ", hex(imu.reg_read(0xFE38)))
    # write buf length
    imu.reg_write(0xFD04, buf_length)
    print("IMU BUF length = ", hex(imu.reg_read(0xFD04)))
else:
    # IMU BUF Config 
    buf_length = 10 # bytes
    imu.reg_write(0xFE12, 0x8000)
    imu.reg_write(0xFE14, 0x0400)
    imu.reg_write(0xFE16, 0x0E00)
    imu.reg_write(0xFE18, 0x0400)
    imu.reg_write(0xFE1A, 0x0000)
    imu.reg_write(0xFE1C, 0x0000)
    imu.reg_write(0xFE1E, 0x0000)
    imu.reg_write(0xFE20, 0x0000)
    imu.reg_write(0xFE22, 0x0000)
    imu.reg_write(0xFE24, 0x0000)
    imu.reg_write(0xFE26, 0x0000)
    print("IMU BUF Write[0] = ", hex(imu.reg_read(0xFE12)))
    print("IMU BUF Write[1] = ", hex(imu.reg_read(0xFE14)))
    print("IMU BUF Write[2] = ", hex(imu.reg_read(0xFE16)))
    print("IMU BUF Write[3] = ", hex(imu.reg_read(0xFE18)))
    print("IMU BUF Write[4] = ", hex(imu.reg_read(0xFE1A)))
    print("IMU BUF Write[5] = ", hex(imu.reg_read(0xFE1C)))
    print("IMU BUF Write[6] = ", hex(imu.reg_read(0xFE1E)))
    print("IMU BUF Write[7] = ", hex(imu.reg_read(0xFE20)))
    print("IMU BUF Write[8] = ", hex(imu.reg_read(0xFE22)))
    print("IMU BUF Write[9] = ", hex(imu.reg_read(0xFE24)))
    print("IMU BUF Write[10] = ", hex(imu.reg_read(0xFE26)))
    # write buf length
    imu.reg_write(0xFD04, buf_length)
    print("IMU BUF length = ", hex(imu.reg_read(0xFD04)))
imu.spi_deb_print = False

imu.set_page(0x00)

# %%
imu.set_page(0xFF)
print("Started capture")

# %%
# Read buffer
if (en_user_burst):
    buf_fetch = [0x06, 0x00]
    for i in range( int((buf_length + 10) / 2) ):
        buf_fetch += [0x00, 0x00]
    imu.spi_deb_print = False
    print(['{0:02X}'.format(int(x)) for x in buf_fetch])
    buf_data_length = buf_length + 12
    # for n in range(1000):
    #     print(imu.reg_read(0xFF04))
    for n in range(20):
        out = ['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch[:buf_data_length], buf_data_length, 1, True)]
        for i in range(1):
            print(out[buf_data_length*i: buf_data_length* (i+1)])
    imu.spi_deb_print = False
else:
    buf_fetch = [0x06, 0x00]
    for i in range(int(buf_length / 2)):
        buf_fetch += [0x12 + i*2, 0x00]
    buf_fetch += [0x00, 0x00]
    buf_data_length = buf_length + 2 + 2 # +2 for buf retrieve, +2 for dummy read for last BUF_DATA reg
    imu.spi_deb_print = False
    print(['{0:02X}'.format(int(x)) for x in buf_fetch])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    print(['{0:02X}'.format(int(x)) for x in imu.trx(buf_fetch, 2, int(buf_data_length/2), False)])
    imu.spi_deb_print = False

# %%
imu.set_page(0xFD)
print("Stopped capture")

# %%
imu.spi.close()