# *****************************************************************************
# * | File        :	  Pico_ePaper-2.13.py
# * | Author      :   Waveshare team
# * | Function    :   Electronic paper driver
# * | Info        :
# *----------------
# * | This version:   V1.0
# * | Date        :   2021-03-16
# # | Info        :   python demo
# -----------------------------------------------------------------------------
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

from machine import Pin, SPI
import framebuf
import utime
import sys
import ujson
import uselect
import math

lut_full_update = [
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,  # LUT0: BB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,  # LUT1: BW:     VS 0 ~7
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,  # LUT2: WB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,  # LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # LUT4: VCOM:   VS 0 ~7

    0x03, 0x03, 0x00, 0x00, 0x02,  # TP0 A~D RP0
    0x09, 0x09, 0x00, 0x00, 0x02,  # TP1 A~D RP1
    0x03, 0x03, 0x00, 0x00, 0x02,  # TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP6 A~D RP6

    0x15, 0x41, 0xA8, 0x32, 0x30, 0x0A,
]

lut_partial_update = [  # 20 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # LUT0: BB:     VS 0 ~7
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # LUT1: BW:     VS 0 ~7
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # LUT2: WB:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # LUT4: VCOM:   VS 0 ~7

    0x0A, 0x00, 0x00, 0x00, 0x00,  # TP0 A~D RP0
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP1 A~D RP1
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00,  # TP6 A~D RP6

    0x15, 0x41, 0xA8, 0x32, 0x30, 0x0A,
]

EPD_WIDTH = 128  # 122
EPD_HEIGHT = 250

RST_PIN = 12
DC_PIN = 8
CS_PIN = 9
BUSY_PIN = 13

FULL_UPDATE = 0
PART_UPDATE = 1


class EPD_2in13(framebuf.FrameBuffer):
    def __init__(self):
        self.reset_pin = Pin(RST_PIN, Pin.OUT)

        self.busy_pin = Pin(BUSY_PIN, Pin.IN, Pin.PULL_UP)
        self.cs_pin = Pin(CS_PIN, Pin.OUT)
        self.width = EPD_WIDTH
        self.height = EPD_HEIGHT

        self.full_lut = lut_full_update
        self.partial_lut = lut_partial_update

        self.full_update = FULL_UPDATE
        self.part_update = PART_UPDATE

        self.spi = SPI(1)
        self.spi.init(baudrate=4000_000)
        self.dc_pin = Pin(DC_PIN, Pin.OUT)

        self.buffer = bytearray(self.height * self.width // 8)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_HLSB)
        self.init(FULL_UPDATE)

    def digital_write(self, pin, value):
        pin.value(value)

    def digital_read(self, pin):
        return pin.value()

    def delay_ms(self, delaytime):
        utime.sleep(delaytime / 1000.0)

    def spi_writebyte(self, data):
        self.spi.write(bytearray(data))

    def module_exit(self):
        self.digital_write(self.reset_pin, 0)

    # Hardware reset
    def reset(self):
        self.digital_write(self.reset_pin, 1)
        self.delay_ms(50)
        self.digital_write(self.reset_pin, 0)
        self.delay_ms(2)
        self.digital_write(self.reset_pin, 1)
        self.delay_ms(50)

    def send_command(self, command):
        self.digital_write(self.dc_pin, 0)
        self.digital_write(self.cs_pin, 0)
        self.spi_writebyte([command])
        self.digital_write(self.cs_pin, 1)

    def send_data(self, data):
        self.digital_write(self.dc_pin, 1)
        self.digital_write(self.cs_pin, 0)
        self.spi_writebyte([data])
        self.digital_write(self.cs_pin, 1)

    def ReadBusy(self):
        while (self.digital_read(self.busy_pin) == 1):  # 0: idle, 1: busy
            self.delay_ms(10)

    def TurnOnDisplay(self):
        self.send_command(0x22)
        self.send_data(0xC7)
        self.send_command(0x20)
        self.ReadBusy()

    def TurnOnDisplayPart(self):
        self.send_command(0x22)
        self.send_data(0x0c)
        self.send_command(0x20)
        self.ReadBusy()

    def init(self, update):
        print('init')
        self.reset()
        if (update == self.full_update):
            self.ReadBusy()
            self.send_command(0x12)  # soft reset
            self.ReadBusy()

            self.send_command(0x74)  # set analog block control
            self.send_data(0x54)
            self.send_command(0x7E)  # set digital block control
            self.send_data(0x3B)

            self.send_command(0x01)  # Driver output control
            self.send_data(0x27)
            self.send_data(0x01)
            self.send_data(0x01)

            self.send_command(0x11)  # data entry mode
            self.send_data(0x01)

            self.send_command(0x44)  # set Ram-X address start/end position
            self.send_data(0x00)
            self.send_data(0x0F)  # 0x0C-->(15+1)*8=128

            self.send_command(0x45)  # set Ram-Y address start/end position
            self.send_data(0x27)  # 0xF9-->(249+1)=250
            self.send_data(0x01)
            self.send_data(0x2e)
            self.send_data(0x00)

            self.send_command(0x3C)  # BorderWavefrom
            self.send_data(0x03)

            self.send_command(0x2C)  # VCOM Voltage
            self.send_data(0x55)  #

            self.send_command(0x03)
            self.send_data(self.full_lut[70])

            self.send_command(0x04)  #
            self.send_data(self.full_lut[71])
            self.send_data(self.full_lut[72])
            self.send_data(self.full_lut[73])

            self.send_command(0x3A)  # Dummy Line
            self.send_data(self.full_lut[74])
            self.send_command(0x3B)  # Gate time
            self.send_data(self.full_lut[75])

            self.send_command(0x32)
            for count in range(70):
                self.send_data(self.full_lut[count])

            self.send_command(0x4E)  # set RAM x address count to 0
            self.send_data(0x00)
            self.send_command(0x4F)  # set RAM y address count to 0X127
            self.send_data(0x0)
            self.send_data(0x00)
            self.ReadBusy()
        else:
            self.send_command(0x2C)  # VCOM Voltage
            self.send_data(0x26)

            self.ReadBusy()

            self.send_command(0x32)
            for count in range(70):
                self.send_data(self.partial_lut[count])

            self.send_command(0x37)
            self.send_data(0x00)
            self.send_data(0x00)
            self.send_data(0x00)
            self.send_data(0x00)
            self.send_data(0x40)
            self.send_data(0x00)
            self.send_data(0x00)

            self.send_command(0x22)
            self.send_data(0xC0)
            self.send_command(0x20)
            self.ReadBusy()

            self.send_command(0x3C)  # BorderWavefrom
            self.send_data(0x01)
        return 0

    def display(self, image):
        self.send_command(0x24)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(image[i + j * int(self.width / 8)])
        self.TurnOnDisplay()

    def displayPartial(self, image):
        self.send_command(0x24)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(image[i + j * int(self.width / 8)])

        self.send_command(0x26)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(~image[i + j * int(self.width / 8)])
        self.TurnOnDisplayPart()

    def displayPartBaseImage(self, image):
        self.send_command(0x24)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(image[i + j * int(self.width / 8)])

        self.send_command(0x26)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(image[i + j * int(self.width / 8)])
        self.TurnOnDisplay()

    def Clear(self, color):
        self.send_command(0x24)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(color)
        self.send_command(0x26)
        for j in range(0, self.height):
            for i in range(0, int(self.width / 8)):
                self.send_data(color)

        self.TurnOnDisplay()

    def sleep(self):
        self.send_command(0x10)  # enter deep sleep
        self.send_data(0x03)
        self.delay_ms(2000)
        self.module_exit()


class RowWriter:
    def __init__(self, epd, start_top=2, row_height=12, bottom_margin=2):
        self.last_top = start_top
        self.current_row = 0
        self.bottom_margin = bottom_margin
        self.row_height = row_height
        self.epd = epd

    def write_line(self, text, underline=False, additional_bottom_margin=0, colour=0x00, center=False):
        line_top = self.last_top
        text = self.pad(text) if center else text
        self.epd.text(text, 0, line_top, colour)
        self.last_top = self.last_top + self.row_height
        if underline:
            self.draw_horizontal_line(128)
        self.last_top = self.last_top + self.bottom_margin + additional_bottom_margin

    def pad(self, text):
        if len(text) < 15:
            pre_pad = math.floor((15 - len(text)) / 2)
            post_pad = 15 - len(text) - pre_pad
            return " " * pre_pad + text + " " * post_pad
        return text

    def draw_horizontal_line(self, width=128):
        self.epd.hline(0, self.last_top, width, 0x00)
        self.last_top += 1


class Time:
    def __init__(self, time_delta=0):
        self.time_delta = time_delta

    def now(self):
        return utime.localtime(utime.time() + self.time_delta)

    def now_unix(self):
        return utime.mktime(self.now())

    def secs_from_midnight(self):
        midnight_date_time = self.now()
        return utime.mktime(self.now()) - utime.mktime(
            (midnight_date_time[0], midnight_date_time[1], midnight_date_time[2], 0, 0, 0, 0, 0))

    def hours_mins(self):
        date_time_now = self.now()
        return "{:02d}:{:02d}".format(date_time_now[3], date_time_now[4])

    def set_delta(self, time_string):
        parts = tuple(map(int, time_string.split(' ')))
        synchronised_time = utime.mktime(parts)
        self.time_delta = synchronised_time - int(utime.time())

    def print_time(self, prefix=""):
        date_time_now = self.now()
        print(prefix + "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(date_time_now[0], date_time_now[1],
                                                                          date_time_now[2],
                                                                          date_time_now[3],
                                                                          date_time_now[4], date_time_now[5]))


nextMeetingSecs = 0


def serial_input_poll():
    spoll = uselect.poll()  # Set up an input polling object.
    spoll.register(sys.stdin, uselect.POLLIN)  # Register polling object.
    kbch = sys.stdin.readline() if spoll.poll(0) else None
    spoll.unregister(sys.stdin)
    if kbch is None:
        return kbch
    return kbch.strip()


def clear(passes=10):
    for i in range(passes):
        epd = EPD_2in13()
        epd.Clear(0xff)
        epd.fill(0xff)
        epd.display(epd.buffer)
        epd.Clear(0x00)
        epd.fill(0x00)
        epd.display(epd.buffer)

        utime.sleep(1)


def start():
    global nextMeetingSecs
    epd = EPD_2in13()
    epd.Clear(0xff)
    epd.fill(0xff)
    local_time = Time()

    while True:
        data = serial_input_poll()
        if data is None:
            epd.init(epd.part_update)
            now = local_time.secs_from_midnight()
            minutes_till_meeting = int(min((nextMeetingSecs - now) / 60, 60))
            percent = minutes_till_meeting / 60
            max_width = 82
            bar_width = int(max_width * percent)
            epd.fill_rect(0, 242, 128, 10, 0xff)
            epd.fill_rect(0, 242, bar_width, 10, 0x00)
            epd.text(local_time.hours_mins(), 82, 242, 0x00)
            epd.displayPartial(epd.buffer)
            utime.sleep(1)
        else:
            epd.fill(0xff)
            lines = ujson.loads(data)
            if 'timeSync' in lines:
                local_time.set_delta(lines["timeSync"] + " 0 0")
                local_time.print_time("Syncing clock ")

            if 'dates' in lines:
                last_top = 0
                row_writer = RowWriter(epd, bottom_margin=0)

                for idx, lineObj in enumerate(lines["dates"]):
                    if idx == 0:
                        nextMeetingSecs = int(lineObj["startSecsFromMidnight"])

                    draw_line = False
                    line = lineObj["summary"]
                    if lineObj["startingSoon"]:
                        # Time goes on it's own line and it has an underline
                        row_writer.write_line(lineObj["startTime"], center=True)
                        draw_line = True
                    else:
                        line = lineObj["startTime"] + " " + line

                    while len(line) > 0:
                        line = line.strip()
                        is_last_line = len(line) <= 15
                        add_bottom_margin = 4 if is_last_line else 0
                        row_writer.write_line(line[:15], additional_bottom_margin=add_bottom_margin,
                                              underline=is_last_line and draw_line)
                        line = line[15:]
                epd.Clear(0xff)
                epd.display(epd.buffer)
                epd.sleep()


if __name__ == '__main__':
    clear(10)
    start()
