#include "ST7735X_kbv.h"
#include "serial_kbv.h"

static uint8_t done_reset;

ST7735X_kbv::ST7735X_kbv(int w, int h):Adafruit_GFX(w, h)
{
}

void ST7735X_kbv::reset(void)
{
	INIT();
    CS_IDLE;
    RESET_IDLE;
    wait_ms(50);
    RESET_ACTIVE;
    wait_ms(100);
    RESET_IDLE;
    wait_ms(100);
	done_reset = 1;
}
/*
void ST7735X_kbv::WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}
*/
void ST7735X_kbv::pushCommand(uint16_t cmd, uint8_t * block, int8_t N)
{
    uint16_t color;
    CS_ACTIVE;
    WriteCmd(cmd);
    CD_DATA;
	write8_block(block, N);
    CS_IDLE;
}

static uint32_t readRegister(uint8_t reg)
{
    uint32_t ret;
	uint8_t bits = 8;
	if (reg == 4) bits = 25;
	if (reg == 9) bits = 33;
    CS_ACTIVE;
    WriteCmd(reg);
    CD_DATA;                    //
	SDIO_INMODE();
    ret = readbits(bits);
    CS_IDLE;
	SDIO_OUTMODE();
    return ret;	
}

uint16_t ST7735X_kbv::readReg(uint16_t reg)
{
    return readRegister(reg);
}

uint32_t ST7735X_kbv::readReg32(uint16_t reg)
{
	return readRegister(reg);
}

uint16_t ST7735X_kbv::readID(void)
{
	if (!done_reset) reset();
    return readRegister(4) >> 8;	
}

#define ST7735X_NOP     0x00
#define ST7735X_SWRESET 0x01
#define ST7735X_RDDID   0x04
#define ST7735X_RDDST   0x09

#define ST7735X_SLPIN   0x10
#define ST7735X_SLPOUT  0x11
#define ST7735X_PTLON   0x12
#define ST7735X_NORON   0x13

#define ST7735X_INVOFF  0x20
#define ST7735X_INVON   0x21
#define ST7735X_DISPOFF 0x28
#define ST7735X_DISPON  0x29
#define ST7735X_CASET   0x2A
#define ST7735X_RASET   0x2B
#define ST7735X_RAMWR   0x2C
#define ST7735X_RAMRD   0x2E

#define ST7735X_PTLAR   0x30
#define ST7735X_COLMOD  0x3A
#define ST7735X_MADCTL  0x36

#define ST7735X_FRMCTR1 0xB1
#define ST7735X_FRMCTR2 0xB2
#define ST7735X_FRMCTR3 0xB3
#define ST7735X_INVCTR  0xB4
#define ST7735X_DISSET5 0xB6

#define ST7735X_PWCTR1  0xC0
#define ST7735X_PWCTR2  0xC1
#define ST7735X_PWCTR3  0xC2
#define ST7735X_PWCTR4  0xC3
#define ST7735X_PWCTR5  0xC4
#define ST7735X_VMCTR1  0xC5

#define ST7735X_RDID1   0xDA
#define ST7735X_RDID2   0xDB
#define ST7735X_RDID3   0xDC
#define ST7735X_RDID4   0xDD

#define ST7735X_PWCTR6  0xFC

#define ST7735X_GMCTRP1 0xE0
#define ST7735X_GMCTRN1 0xE1

#define ILI9163_NOP     0x00
#define ILI9163_SWRESET 0x01
#define ILI9163_RDDID   0x04
#define ILI9163_RDDST   0x09

#define ILI9163_SLPIN   0x10
#define ILI9163_SLPOUT  0x11
#define ILI9163_PTLON   0x12
#define ILI9163_NORON   0x13

#define ILI9163_INVOFF  0x20
#define CMD_GAMMASET 	0x26//Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
#define ILI9163_INVON   0x21
#define ILI9163_DISPOFF 0x28
#define ILI9163_DISPON  0x29
#define ILI9163_CASET   0x2A
#define ILI9163_RASET   0x2B
#define ILI9163_RAMWR   0x2C
#define ILI9163_RAMRD   0x2E

#define ILI9163_PTLAR   0x30
#define CMD_VSCLLDEF	0x33//Vertical Scroll Definition
#define ILI9163_COLMOD  0x3A
#define ILI9163_MADCTL  0x36
#define CMD_VSSTADRS	0x37//Vertical Scrolling Start address

#define ILI9163_FRMCTR1 0xB1
#define ILI9163_FRMCTR2 0xB2
#define ILI9163_FRMCTR3 0xB3
#define ILI9163_INVCTR  0xB4
#define CMD_DFUNCTR 	0xB6//Display Fuction set 5
#define CMD_SDRVDIR 	0xB7//Source Driver Direction Control
#define CMD_GDRVDIR 	0xB8//Gate Driver Direction Control

#define ILI9163_PWCTR1  0xC0
#define ILI9163_PWCTR2  0xC1
#define ILI9163_PWCTR3  0xC2
#define ILI9163_PWCTR4  0xC3
#define ILI9163_PWCTR5  0xC4
#define ILI9163_VMCTR1  0xC5
#define CMD_VCOMCTR2  	0xC6//VCOM_Control 2
#define CMD_VCOMOFFS  	0xC7//VCOM Offset Control

#define ILI9163_RDID1   0xDA
#define ILI9163_RDID2   0xDB
#define ILI9163_RDID3   0xDC
#define ILI9163_RDID4   0xDD

//#define ILI9163_PWCTR6  0xFC

#define ILI9163_GMCTRP1 0xE0
#define ILI9163_GMCTRN1 0xE1
#define CMD_GAMRSEL		0xF2//GAM_R_SEL

int16_t ST7735X_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
	uint8_t r, g, b;
	int16_t n = w * h;    // we are NEVER going to read > 32k pixels at once
	uint8_t colmod = 0x66;
	pushCommand(ST7735X_COLMOD, &colmod, 1);
	setAddrWindow(x, y, x + w - 1, y + h - 1);
	CS_ACTIVE;
	WriteCmd(ST7735X_RAMRD);
	CD_DATA;
	SDIO_INMODE();        // do this while CS is Active

	r = readbits(8 + _is7735);	  //(8) for ILI9163, (9) for ST7735
	while (n-- > 0) {
		r = readbits(8);
		g = readbits(8);
		b = readbits(8);
		*block++ = color565(r, g, b);
	}
	CS_IDLE;
	SDIO_OUTMODE();      //do this when CS is Idle
    setAddrWindow(0, 0, width() - 1, height() - 1);
	colmod = 0x05;
	pushCommand(ST7735X_COLMOD, &colmod, 1);
    return 0;
}

void ST7735X_kbv::setRotation(uint8_t r)
{
    uint8_t mac = 0x00;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
    case 0:
        mac = 0x48;
        break;
    case 1:        //LANDSCAPE 90 degrees
        mac = 0x28;
        break;
    case 2:
        mac = 0x98;
        break;
    case 3:
        mac = 0xF8;
        break;
    }
	mac ^= (_lcd_xor);
    pushCommand(ST7735X_MADCTL, &mac, 1);
}

void ST7735X_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
	if (rotation == 0) y += __OFFSET;
	if (rotation == 1) x += __OFFSET;
    spibuf[0] = x >> 8;
    spibuf[1] = x;
	pushCommand(ST7735X_CASET, spibuf, 2);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
	pushCommand(ST7735X_RASET, spibuf, 2);
    spibuf[0] = color >> 8;
    spibuf[1] = color;
	pushCommand(ST7735X_RAMWR, spibuf, 2);
}

void ST7735X_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
	if (rotation == 0) y += __OFFSET, y1 += __OFFSET;
	if (rotation == 1) x += __OFFSET, x1 += __OFFSET;
    spibuf[0] = x >> 8;
    spibuf[1] = x;
    spibuf[2] = x1 >> 8;
    spibuf[3] = x1;
	pushCommand(ST7735X_CASET, spibuf, 4);
    spibuf[0] = y >> 8;
    spibuf[1] = y;
    spibuf[2] = y1 >> 8;
    spibuf[3] = y1;
	pushCommand(ST7735X_RASET, spibuf, 4);
}

void ST7735X_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(ST7735X_RAMWR);
    CD_DATA;
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    while (h-- > 0) {
        write16_N(color, w);
    }
    CS_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

void ST7735X_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ST7735X_RAMWR);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
        write16(color);
    }
    CS_IDLE;
}

void ST7735X_kbv::pushColors(uint8_t * block, int16_t n, bool first)
{
    uint16_t color;
	uint8_t h, l;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ST7735X_RAMWR);
    }
    CD_DATA;
    while (n-- > 0) {
        h = (*block++);
        l = (*block++);
        color = (h << 8) | l;
        write16(color);
    }
    CS_IDLE;
}

void ST7735X_kbv::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
    uint16_t color;
	uint8_t h, l;
	CS_ACTIVE;
    if (first) {
        WriteCmd(ST7735X_RAMWR);
    }
    CD_DATA;
    while (n-- > 0) {
        l = pgm_read_byte(block++);
        h = pgm_read_byte(block++);
        color = (bigend) ? (l << 8 ) | h : (h << 8) | l;
		write16(color);
    }
    CS_IDLE;
}

void ST7735X_kbv::invertDisplay(boolean i)
{
    pushCommand(i ? ST7735X_INVON : ST7735X_INVOFF, NULL, 0);
}

void ST7735X_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    if (rotation == 0 || rotation == 1) top += __OFFSET;
    int16_t bfa = HEIGHT + __OFFSET - top - scrollines;  // bottom fixed area
    int16_t vsp;
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    spibuf[0] = top>>8;
	spibuf[1] = top;
	spibuf[2] = scrollines>>8;
	spibuf[3] = scrollines;
	spibuf[4] = bfa>>8;
	spibuf[5] = bfa;
	pushCommand(0x33, spibuf, 6);
    spibuf[0] = vsp>>8;
	spibuf[1] = vsp;
	pushCommand(0x37, spibuf, 2);
}

#define TFTLCD_DELAY 0xFF

const uint8_t PROGMEM table7735S[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    (ST7735X_SWRESET), 0,        // software reset
    TFTLCD_DELAY, 50,
    (ST7735X_SLPOUT), 0,         //Sleep exit
    TFTLCD_DELAY, 250,
    //ST7735XR Frame Rate
    (ST7735X_FRMCTR1), 3, 0x01, 0x2C, 0x2D,
    (ST7735X_FRMCTR2), 3, 0x01, 0x2C, 0x2D,
    (ST7735X_FRMCTR3), 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
    (ST7735X_INVCTR), 1, 0x07,   //Column inversion
    //ST7735XR Power Sequence
    (ST7735X_PWCTR1), 3, 0xA2, 0x02, 0x84,
    (ST7735X_PWCTR2), 1, 0xC5,
    (ST7735X_PWCTR3), 2, 0x0A, 0x00,
    (ST7735X_PWCTR4), 2, 0x8A, 0x2A,
    (ST7735X_PWCTR5), 2, 0x8A, 0xEE,
    (ST7735X_VMCTR1), 1, 0x0E,   //VCOM
    (ST7735X_INVOFF), 0, //no inversion
    //(ST7735X_MADCTL), 1, 0xC8, //MX, MY, RGB mode
    (ST7735X_MADCTL), 1, 0x00,   //MX, MY, RGB mode
/*
    //ST7735XR Gamma Sequence
    (ST7735X_GMCTRP1), 16,
    0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f, 0x1b, 0x23, 0x37,
    0x00, 0x07, 0x02, 0x10,
    (ST7735X_GMCTRN1), 16,
    0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30, 0x30, 0x39, 0x3f,
    0x00, 0x07, 0x03, 0x10,
*/
    (ST7735X_CASET), 4, 0x00, 0x00, 0x00, 0x7f,
    (ST7735X_RASET), 4, 0x00, 0x00, 0x00, 0x9f,
    (0xF0), 1, 0x00,            //Enable test command [01]
    (0xF6), 1, 0x00,            //Disable ram power save mode

    (ST7735X_COLMOD), 1, 0x65,   //65k mode
    (ST7735X_DISPON), 0,         //Display on
};

const uint8_t PROGMEM table9101[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    (ST7735X_SWRESET), 0,        // software reset
    TFTLCD_DELAY, 50,
    (ST7735X_SLPOUT), 0,         //Sleep exit
    TFTLCD_DELAY, 250,
/*
    //ST7735XR Frame Rate
    (ST7735X_FRMCTR1), 3, 0x01, 0x2C, 0x2D,
    (ST7735X_FRMCTR2), 3, 0x01, 0x2C, 0x2D,
    (ST7735X_FRMCTR3), 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
    (ST7735X_INVCTR), 1, 0x07,   //Column inversion
    //ST7735XR Power Sequence
    (ST7735X_PWCTR1), 3, 0xA2, 0x02, 0x84,
    (ST7735X_PWCTR2), 1, 0xC5,
    (ST7735X_PWCTR3), 2, 0x0A, 0x00,
    (ST7735X_PWCTR4), 2, 0x8A, 0x2A,
    (ST7735X_PWCTR5), 2, 0x8A, 0xEE,
    (ST7735X_VMCTR1), 1, 0x0E,   //VCOM
    (ST7735X_INVOFF), 0, //no inversion
    //(ST7735X_MADCTL), 1, 0xC8, //MX, MY, RGB mode
    (ST7735X_MADCTL), 1, 0x00,   //MX, MY, RGB mode
    //ST7735XR Gamma Sequence
    (ST7735X_GMCTRP1), 16,
    0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f, 0x1b, 0x23, 0x37,
    0x00, 0x07, 0x02, 0x10,
    (ST7735X_GMCTRN1), 16,
    0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30, 0x30, 0x39, 0x3f,
    0x00, 0x07, 0x03, 0x10,

    (ST7735X_CASET), 4, 0x00, 0x00, 0x00, 0x7f,
    (ST7735X_RASET), 4, 0x00, 0x00, 0x00, 0x9f,
*/
//    (0xB0), 2, 0x00, 0xF0,           //RAMCTRL (B0h): RAM Control on ST7789V
//    (0xF6), 1, 0x01, 0x00, 0x00,     //Interface Control (F6h) on ILI9341
    (ST7735X_COLMOD), 1, 0x05,   //65k mode
    (ST7735X_DISPON), 0,         //Display on
};

const uint8_t PROGMEM table9163C[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    (ILI9163_SWRESET), 0,        // software reset
    TFTLCD_DELAY, 250,
    TFTLCD_DELAY, 250,
    (ILI9163_SLPOUT), 0,         //Sleep exit
    TFTLCD_DELAY, 5,
    (ILI9163_COLMOD), 1, 0x05,   //65k mode
    TFTLCD_DELAY, 5,
    (CMD_GAMMASET), 1, 0x04,
    TFTLCD_DELAY, 1,
    (CMD_GAMRSEL), 1, 0x00,      //.kbv don't select user GAMMA
    TFTLCD_DELAY, 1,
    (ILI9163_NORON), 0,         //Normal
//    (CMD_DFUNCTR), 2, 0xFF, 0x06, //Display Function set 5 ??NL
//#if __OFFSET == 0
//    (CMD_SDRVDIR), 1, 0x01,
    (CMD_GDRVDIR), 1, 0x01,
//#else
//    (CMD_SDRVDIR), 1, 0x00,
//    (CMD_GDRVDIR), 1, 0x00,
//#endif
	(ILI9163_GMCTRP1), 16,
    0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f, 0x1b, 0x23, 0x37,
    0x00, 0x07, 0x02, 0x10,
    (ILI9163_GMCTRN1), 16,
    0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30, 0x30, 0x39, 0x3f,
    0x00, 0x07, 0x03, 0x10,
    (ILI9163_FRMCTR1), 2, 0x08, 0x02,
    (ILI9163_INVCTR), 1, 0x07,   //Column inversion
    TFTLCD_DELAY, 1,
    (ILI9163_PWCTR1), 2, 0x0A, 0x02,
    TFTLCD_DELAY, 1,
    (ILI9163_PWCTR2), 1, 0x02,
    TFTLCD_DELAY, 1,
    (ILI9163_VMCTR1), 2, 0x50, 0x63,   //VCOM (#99)
    TFTLCD_DELAY, 1,
    (CMD_VCOMOFFS), 1, 0,
    TFTLCD_DELAY, 1,
//	(CMD_VSCLLDEF), 6, 0, __OFFSET, 0, 128 + __OFFSET, 0, 0,
	(CMD_VSCLLDEF), 6, 0, 0, 0, 128 + 0, 0, 0,
    (ILI9163_MADCTL), 1, 0x00,   //MX, MY, RGB mode
    (ILI9163_PTLAR), 4, 0x00, 0x00, 0x00, 0x7f,
    (ILI9163_CASET), 4, 0x00, 0x00, 0x00, 0x7f,
    (ILI9163_RASET), 4, 0x00, 0x00, 0x00, 0x7f,
//    (0xF0), 1, 0x01,            //Enable test command
//    (0xF6), 1, 0x00,            //Disable ram power save mode
    (ILI9163_DISPON), 0,         //Display on
};

void ST7735X_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
	_lcd_xor = 0x00;
    uint8_t *p = (uint8_t *) table7735S;
    int16_t size = sizeof(table7735S);
    reset();
    switch(ID) {
	case 0x0091:    //wot readID()
	case 0x9101:
	    __OFFSET = 32;
	    _lcd_xor = 0x80;
		p = (uint8_t *) table9101;
		size = sizeof(table9101);
		break;
	case 0x5480:     //wot readID()
	case 0x9162:
	    __OFFSET = 32;
		_lcd_xor = 0xD0;
		goto common_9163;
	case 0x9163: 
	    _lcd_xor = 0x00;
common_9163:
		p = (uint8_t *) table9163C;
		size = sizeof(table9163C);
		break;
	case 0x7734:
	    __OFFSET = 32;
	    _lcd_xor = 0x90;
		goto common_7735;
	case 0x7C89:
	case 0x7735:
	default: 
		_lcd_xor = 0x98;
common_7735:
	    _is7735 = 1;
		p = (uint8_t *) table7735S;
		size = sizeof(table7735S);
		break;
    }
	while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY) {
            delay(len);
            len = 0;
        } else {
            CS_ACTIVE;
            WriteCmd(cmd);
            CD_DATA;
            for (uint8_t d = 0; d < len; d++) {
                uint8_t x = pgm_read_byte(p++);
                xchg8(x);
            }
            CS_IDLE;
        }
        size -= len + 2;
    }
    setRotation(0);             //PORTRAIT
}
