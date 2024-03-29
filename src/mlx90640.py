from enum import Enum
import struct

import numpy as np

class ReadingPattern(Enum):
    TV_INTERLEAVED = 0,
    CHESS_PATTERN = 1

class MLX90640:
    def __init__(self, i2cbus = None, dev = 0x33, emissivity = 1.0):
        if i2cbus is None:
            raise ValueError("The used i2c bus has to be specified!")
        if dev is None:
            raise ValueError("The used device address has to be specified!")

        self._i2c = i2cbus
        self._adr = dev 
        self._rows = 24
        self._cols = 32
        self._npixels = self._rows * self._cols

        self.frameCallback = []

        self._emissivity = emissivity

        # Quickly verify communication is working ...
        d1 = self._i2c_read_word(0x240F)
        if d1 is None:
            raise ValueError("Communication error")
        d1 = d1[0]
        if (d1 & 0xFF) != self._adr:
            raise ValueError("Stored I2C address from EEPROM mismatches our used address ... suspecting communication error")

        # Registers (shadowed)
        self._reg__osc_trim = None
        self._reg__ana_trim = None
        self._reg__conf_reg = None
        self._reg__id = ( None, None, None )
        self._reg__cr1 = None
        self._reg__cr2 = None
        self._reg__i2c_conf = None
        self._reg__i2c_addr = None

        # Calibration parameter
        self._Kvdd = None
        self._Vdd25 = None
        self._Vptat25 = None
        self._Ktptat = None
        self._Kvptat = None
        self._alphaptat = None
        self._alphaptatee = None
        self._Vptat = None
        self._Vbe = None
        self._dV = None
        self._Ksta = None

        self._resolutioncorr = None
        self._Vdd = None
        self._KsTo = ( None, None, None, None )
        self._alphacorr = ( None, None, None, None )
        self._alphaCP = ( None, None )
        self._offCP = ( None, None )
        self._KvCP = None
        self._KtaCP = None
        self._resolution_EE = None
        self._KGain = None
        self._tgc = None

        self._occ_row = np.empty((self._rows,)) # [ 0 ] * self._rows
        self._occ_col = np.empty((self._cols,)) # [ 0 ] * self._cols
        self._acc_row = np.empty((self._rows,)) # [ 0 ] * self._rows
        self._acc_col = np.empty((self._cols,)) # [ 0 ] * self._cols

        self._offset_pixel = np.empty((self._rows, self._cols))
        self._alpha_pixel = np.empty((self._rows, self._cols))
        self._Kta_pixel = np.empty((self._rows, self._cols))
        self._Kv_pixel = np.empty((self._rows, self._cols))

        self._subpagePattern = np.empty((self._rows,self._cols))
        for iPixelH in range(int(self._cols * self._rows / 2)):
            iPixel = iPixelH * 2
            iCol = iPixel % self._cols
            iRow = int(iPixel / self._cols)
            self._subpagePattern[iRow,iCol] = 0.0
            iPixel = iPixelH * 2 + 1
            iCol = iPixel % self._cols
            iRow = int(iPixel / self._cols)
            self._subpagePattern[iRow,iCol] = 1.0

        # Load and process EEPROM
        self._load_eeprom()
        self._shadow_init()



    def __enter__(self):
        return self
    def __exit__(self, type, value, tb):
        return

    # The open and close methods are just provided to mimic the interface
    # of the simplepycam V4L wrapper
    def open(self):
        pass
    def close(self):
        pass
    def streamOn(self):
        pass
    def streamOff(self):
        pass

    # The following wrappers are used to mimic the behavior of the simplepycam
    # V4L wrapper:

    def stream(self):
        while True:
            frame = self._fetch_frame_sync()
            abrt = True
            for cb in self.frameCallback:
                abrt = abrt and cb(self, frame)
            if not abrt:
                break
    def nextFrame(self):
        frame = self._fetch_frame_sync()
        return frame



    def _get_id(self):
        return self._reg__id

    def _get_refresh_rate(self):
        r = self._i2c_read_word(0x800D)
        if r is None:
            raise ValueError("Communication error")
        r = ((r[0]) >> 7) & 0x07
        return r

    def _set_refresh_rate(self, newRate):
        rate = newRate & 0x07

        cr1 = self._i2c_read_word(0x800D)
        if cr1 is None:
            raise ValueError("Communication error")
        cr1 = (cr1[0] & 0xFC7F) | (rate << 7)
        return self._i2c_write_word(0x800D, cr1)

    def _get_overwrite_enable(self):
        r = self._i2c_read_word(0x8000)
        if r & 0x10:
            return False
        else:
            return True

    def _set_overwrite_enable(self, en):
        r = self._i2c_read_word(0x8000)
        r = r & 0xFFEF
        if en:
            r = r | 0x10
        return self._i2c_write_word(0x8000, r)

    def _is_new_data_avail(self, clearFlag = False):
        r = self._i2c_read_word(0x8000)
        if r & 0x08:
            if clearFlag:
                r = r & 0xFFF7
                self._i2c_write_word(0x8000, r)
            return True
        return False

    def fetch_frame(self):
        return self._fetch_frame_sync()

    def _fetch_frame_sync(self):
        rawframe = self._fetch_raw_frame_sync()
        self._update_ram()
        to = self._process_frame(rawframe)
        return to

    def _fetch_raw_frame_sync(self):
        framedata = np.empty((self._rows, self._cols))
        fetchedFrames = 0

        # Wait till data ready flag ...
        for _ in range(2):
            while True:
                r = self._i2c_read_word(0x8000)[0]
                if r & 0x08:
                    r = r & 0xFFF7
                    self._i2c_write_word(0x8000, r)
                    break
            fetchedSubpage = r & 0x01

            # Checkerboard fetch
            for iPixelH in range(int(self._cols * self._rows / 2)):
                iPixel = iPixelH * 2 + fetchedSubpage

                iCol = iPixel % self._cols
                iRow = int(iPixel / self._cols)
                framedata[iRow,iCol] = self._i2c_read_word(0x400 + iPixel)[0]
                if framedata[iRow,iCol] > 32766:
                    framedata[iRow,iCol] = framedata[iRow,iCol] - 65536

        return framedata

    def _process_frame(
            self,
            rawdata,
            Ta0 = 25.0,
            VddV0 = 3.3
    ):
        # Calculation of Vdd25, Kvdd, KVPtat, KTPtat, dV, VPtat25, Ta, Kgain happens in update_ram ...

        # Gain compensation (11.2.2.5.1)
        pix_gain = rawdata * self._KGain

        # IR data compensation (offset, VDD and Ta)
        pix_os = pix_gain - self._offset_pixel * (1.0 + self._Kta_pixel * (self._Ta - Ta0)) * (1 + self._Kv_pixel * (self._Vdd - VddV0))

        # IR data emissivity compensation (11.2.2.5.4)
        VIR = pix_os / self._emissivity

        # Compensation pixel calculations (11.2.2.6.1)
        pix_gain_CP_SP0 = self._i2c_read_word(0x0708)[0]
        if pix_gain_CP_SP0 > 32767:
            pix_gain_CP_SP0 = pix_gain_CP_SP0 - 65536
        pix_gain_CP_SP0 = pix_gain_CP_SP0 * self._KGain

        pix_gain_CP_SP1 = self._i2c_read_word(0x0728)[0]
        if pix_gain_CP_SP1 > 32767:
            pix_gain_CP_SP1 = pix_gain_CP_SP1 - 65536
        pix_gain_CP_SP1 = pix_gain_CP_SP1 * self._KGain

        # Compensating offset, Ta and Vdd of CP pixel (11.2.2.6.2)
        pix_OS_CP_SP0 = pix_gain_CP_SP0 - self._offCP[0] * (1 + self._KtaCP * (self._Ta - Ta0))*(1 + self._KvCP * (self._Vdd - VddV0))
        pix_OS_CP_SP1 = pix_gain_CP_SP1 - self._offCP[1] * (1 + self._KtaCP * (self._Ta - Ta0))*(1 + self._KvCP * (self._Vdd - VddV0))

        # IR gradient compensation (11.2.2.7)
        # Note pattern just merly selects subpage 0 or 1 for the given pixels ...
        VIR_compensated = VIR - self._tgc * ((1 - self._subpagePattern) * pix_OS_CP_SP0 + self._subpagePattern * pix_OS_CP_SP1)

        # Sensitivity normalization (11.2.2.8)
        alpha_comp = (self._alpha_pixel - self._tgc * ((1 - self._subpagePattern) * self._alphaCP[0] + self._subpagePattern * self._alphaCP[1])) * (1 + self._Ksta * (self._Ta - Ta0))

        # First calculate To for basic range (0 to CT3) (11.2.2.9)
        Tr = self._Ta - 8 # This is just an assumption, see manual; Reflected IR is not directly accessible ...
        TaK4 = (self._Ta + 273.15)**4.0
        TrK4 = (Tr + 273.15)**4.0
        Ta_r = TrK4 - (TrK4 - TaK4) / (self._emissivity)
        Sx = self._KsTo[1] * np.power(np.power(alpha_comp, 3) * VIR_compensated + np.power(alpha_comp, 4) * Ta_r, 0.25)
        To = np.power(VIR_compensated / (alpha_comp * (1 - self._KsTo[1] * 273.15) + Sx) + Ta_r, 0.25) - 273.15

        return To




    def _load_eeprom(self):
        self._eeprom = []
        for adr in range(0x2400, 0x2740):
            self._eeprom.append(self._i2c_read_word(adr)[0])

        self._reg__osc_trim = self._eeprom[0x2400 - 0x2400]
        self._reg__ana_trim = self._eeprom[0x2401 - 0x2400]
        self._reg__conf_reg = self._eeprom[0x2403 - 0x2400]
        self._reg__id = ( self._eeprom[0x2407 - 0x2400], self._eeprom[0x2408 - 0x2400], self._eeprom[0x2409 - 0x2400] )
        self._reg__cr1 = self._eeprom[0x240C - 0x2400]
        self._reg__cr2 = self._eeprom[0x240D - 0x2400]
        self._reg__conf_i2c_conf = self._eeprom[0x240E - 0x2400]
        self._reg__conf_i2c_addr = self._eeprom[0x240F - 0x2400] & 0xFF
        
        # Restore calibration data ...



        ## OCC and ACC row and column
        for irow in range(int(self._rows / 4)):
            rv = self._eeprom[0x2412 - 0x2400 + irow]
            self._occ_row[irow*4 + 0] = rv & 0x000F
            self._occ_row[irow*4 + 1] = (rv & 0x00F0) >> 4
            self._occ_row[irow*4 + 2] = (rv & 0x0F00) >> 8
            self._occ_row[irow*4 + 3] = (rv & 0xF000) >> 12

            rv = self._eeprom[0x2422 - 0x2400 + irow]
            self._acc_row[irow * 4 + 0] = rv & 0x000F
            self._acc_row[irow * 4 + 1] = (rv & 0x00F0) >> 4
            self._acc_row[irow * 4 + 2] = (rv & 0x0F00) >> 8
            self._acc_row[irow * 4 + 3] = (rv & 0xF000) >> 12
        for icol in range(int(self._cols / 4)):
            rv = self._eeprom[0x2418 - 0x2400 + icol]
            self._occ_col[icol * 4 + 0] = rv & 0x000F
            self._occ_col[icol * 4 + 1] = (rv & 0x00F0) >> 4
            self._occ_col[icol * 4 + 2] = (rv & 0x0F00) >> 8
            self._occ_col[icol * 4 + 3] = (rv & 0xF000) >> 12

            rv = self._eeprom[0x2428 - 0x2400 + icol]
            self._acc_col[icol * 4 + 0] = rv & 0x000F
            self._acc_col[icol * 4 + 1] = (rv & 0x00F0) >> 4
            self._acc_col[icol * 4 + 2] = (rv & 0x0F00) >> 8
            self._acc_col[icol * 4 + 3] = (rv & 0xF000) >> 12

        for irow in range(self._rows):
            if self._occ_row[irow] > 7:
                self._occ_row[irow] = self._occ_row[irow] - 16
            if self._acc_row[irow] > 7:
                self._acc_row[irow] = self._acc_row[irow] - 16
        for icol in range(self._cols):
            if self._occ_col[icol] > 7:
                self._occ_col[icol] = self._occ_col[icol] - 16
            if self._acc_col[icol] > 7:
                self._acc_col[icol] = self._acc_col[icol] - 16

        ## OCC and ACC scale and remmanan.

        rv = self._eeprom[0x2410 - 0x2400]
        self._occ_scale_row = (rv & 0x0F00) >> 8
        self._occ_scale_col = (rv & 0x00F0) >> 4
        self._occ_scale_rem = (rv & 0x000F)
        self._pix_os_average = self._eeprom[0x2411 - 0x2400]
        if self._pix_os_average > 32767:
            self._pix_os_average = self._pix_os_average - 65536
        rv = self._eeprom[0x2420 - 0x2400]
        self._acc_scale = ((rv & 0xF000) >> 12) + 30
        self._acc_scale_row = (rv & 0x0F00) >> 8
        self._acc_scale_col = (rv & 0x00F0) >> 4
        self._acc_scale_rem = (rv & 0x000F)
        self._pix_sens_average = self._eeprom[0x2421 - 0x2400]

        ## KVdd (11.1.1)

        Kvdd = (self._eeprom[0x2433 - 0x2400] & 0xFF00) / 256
        if Kvdd > 127:
            Kvdd = Kvdd - 256
        Kvdd = Kvdd * 32
        Vdd25 = self._eeprom[0x2433 - 0x2400] & 0x00FF
        Vdd25 = (Vdd25 - 256) * 32 - 8192

        self._Kvdd = Kvdd
        self._Vdd25 = Vdd25

        ## Constant Ta parameters (11.1.2)

        self._Vptat25 = self._eeprom[0x2431 - 0x2400]
        if self._Vptat25 > 32767:
            self._Vptat25 = self._Vptat25 - 65536

        self._Ktptat = self._eeprom[0x2432 - 0x2400] & 0x03FF
        if self._Ktptat > 511:
            self._Ktptat = self._Ktptat - 1024
        self._Ktptat = self._Ktptat / 8

        self._Kvptat = (self._eeprom[0x2432 - 0x2400] & 0xFC00) / 1024
        if self._Kvptat > 31:
            self._Kvptat = self._Kvptat - 64
        self._Kvptat = self._Kvptat / 4096

        self._alphaptatee = (self._eeprom[0x2410 - 0x2400] & 0xF000) >> 12
        self._alphaptat = (self._alphaptatee >> 2) + 8

        ## Restoring GAIN coefficient (common, 11.1.7)

        self._gain = self._eeprom[0x2430 - 0x2400]
        if self._gain > 32767:
            self._gain = self._gain - 65536

        ## Restoring KsTa coefficient (common; 11.1.8)
        Ksta_EE = (self._eeprom[0x243C - 0x2400] & 0xFF99) >> 8
        if Ksta_EE > 127:
            Ksta_EE = Ksta_EE - 256
        self._Ksta = Ksta_EE / 8192

        ## Corner temperatures
        self._ct1 = -40
        self._ct2 = 0
        self._ctstep = ((self._eeprom[0x243F - 0x2400] & 0x3000) >> 12) * 10
        self._ct3 = ((self._eeprom[0x243F - 0x2400] & 0x00F0) >> 4) * self._ctstep
        self._ct4 = ((self._eeprom[0x243F - 0x2400] & 0x0F00) >> 8) * self._ctstep + self._ct3

        ## KsTo (common; 11.1.10)

        KSto_Scale = (self._eeprom[0x243F - 0x2400] & 0x000F) + 8
        KSto1_EE = (self._eeprom[0x243D - 0x2400] & 0x00FF)
        if KSto1_EE > 127:
            KSto1_EE = KSto1_EE - 256
        KSto1 = KSto1_EE / (2 ** KSto_Scale)
        KSto2_EE = (self._eeprom[0x243D - 0x2400] & 0xFF00) >> 8
        if KSto2_EE > 127:
            KSto2_EE = KSto2_EE - 256
        KSto2 = KSto2_EE / (2 ** KSto_Scale)
        KSto3_EE = (self._eeprom[0x243E - 0x2400] & 0x00FF)
        if KSto3_EE > 127:
            KSto3_EE = KSto3_EE - 256
        KSto3 = KSto3_EE / (2 ** KSto_Scale)
        KSto4_EE = (self._eeprom[0x243E - 0x2400] & 0xFF00) >> 8
        if KSto4_EE > 127:
            KSto4_EE = KSto4_EE - 256
        KSto4 = KSto4_EE / (2 ** KSto_Scale)
        self._KsTo = ( KSto1, KSto2, KSto3, KSto4 )

        ## Sensitivity correction for each temp. range (11.1.11)
        self._alphacorr = (
            1.0 / (1 + KSto1 * 40),
            1.0,
            1.0 + KSto2 * self._ct3,
            (1.0 + KSto2 * self._ct3) * (1 + KSto3 * (self._ct4 - self._ct3))
        )

        ## Compensation pixel sensitivity (11.1.12)

        self._alphaScale_CP = ((self._eeprom[0x2420 - 0x2400] & 0xF000) >> 12) + 27
        alphaCP_P0 = (self._eeprom[0x2439 - 0x2400] & 0x03FF) >> (self._alphaScale_CP)
        alphaCP_P1P0Ratio = (self._eeprom[0x2439 - 0x2400] & 0xFC00) >> 10
        if alphaCP_P1P0Ratio > 31:
            alphaCP_P1P0Ratio = alphaCP_P1P0Ratio - 64
        alphaCP_P1 = alphaCP_P0 * (1 + alphaCP_P1P0Ratio/128)
        self._alphaCP = (alphaCP_P0, alphaCP_P1)

        ## Compensation pixel offset (11.1.13)

        off_cp0 = self._eeprom[0x243A - 0x2400] & 0x03FF
        if off_cp0 > 511:
            off_cp0 = off_cp0 - 1024
        off_cp1cp2delta = (self._eeprom[0x243A - 0x2400] & 0xFC00) >> 10
        if off_cp1cp2delta > 31:
            off_cp1cp2delta = off_cp1cp2delta - 64
        off_cp1 = off_cp0 + off_cp1cp2delta
        self._offCP = ( off_cp0, off_cp1 )

        ## KV for compensation pixel

        KvCp_EE = (self._eeprom[0x243B - 0x2400] & 0xFF00) >> 8
        if KvCp_EE > 127:
            KvCp_EE = KvCp_EE - 256
        KVScale = (self._eeprom[0x2438 - 0x2400] & 0x0F00) >> 8
        self._KvCP = KvCp_EE / KVScale

        ## Kta for compensation pixel

        KTa_Scale1 = ((self._eeprom[0x2438 - 0x2400] & 0x00F0) >> 4) + 8
        KTaCP_EE = self._eeprom[0x243B - 0x2400] & 0x00FF
        if KTaCP_EE > 127:
            KTaCP_EE = KTaCP_EE - 256
        self._KtaCP = KTaCP_EE / KTa_Scale1

        ## TGC coefficient (11.1.16)

        TGCEE = self._eeprom[0x243C - 0x2400] & 0x00FF
        if TGCEE > 127:
            TGCEE = TGCEE - 256
        self._tgc = TGCEE / 32

        ## Resolution control (11.1.17)

        self._resolution_EE = (self._eeprom[0x2438 - 0x2400] & 0x3000) >> 12

        ## Per pixel offset values and alpha (sensitivity) values / KTa values

        self._KTa_scale_1 = ((self._eeprom[0x2438 - 0x2400] & 0x00F0) >> 4) + 8
        self._KTa_scale_2 = ((self._eeprom[0x2438 - 0x2400] & 0x000F))
        self._KTa_RC_EE = (
            (self._eeprom[0x2436 - 0x2400] & 0xFF00) >> 8,
            (self._eeprom[0x2436 - 0x2400] & 0x00FF),
            (self._eeprom[0x2437 - 0x2400] & 0xFF00) >> 8,
            (self._eeprom[0x2437 - 0x2400] & 0x00FF)
        )
        self._KVPixelValues = [
            (self._eeprom[0x2434 - 0x2400] & 0xF000) >> 12,
            (self._eeprom[0x2434 - 0x2400] & 0x0F00) >> 8,
            (self._eeprom[0x2434 - 0x2400] & 0x00F0) >> 4,
            (self._eeprom[0x2434 - 0x2400] & 0x000F)
        ]
        for i in range(len(self._KTa_RC_EE)):
            if self._KTa_RC_EE[i] > 127:
                self._KTa_RC_EE[i] = self._KTa_RC_EE[i] - 256
            if self._KVPixelValues[i] > 7:
                self._KVPixelValues[i] = self._KVPixelValues[i] - 16
            self._KVPixelValues[i] = self._KVPixelValues[i] / KVScale

        for irow in range(self._rows):
            for icol in range(self._cols):
                rv = self._eeprom[0x2440 - 0x2400 + icol + (irow * self._cols)]
                self._offset_pixel[irow, icol] = (rv & 0xFC00) >> 10
                if self._offset_pixel[irow, icol] > 31:
                    self._offset_pixel[irow, icol] = self._offset_pixel[irow, icol] - 64
                self._offset_pixel[irow, icol] = self._offset_pixel[irow, icol] * (1 << self._occ_scale_rem)
                self._offset_pixel[irow, icol] = self._offset_pixel[irow, icol] + (self._occ_row[irow] * (1 <<  self._occ_scale_row)) + (self._occ_col[icol] * (1 << self._occ_scale_col)) + self._pix_os_average

                self._alpha_pixel[irow, icol] = (rv & 0x03F0) >> 4
                if self._alpha_pixel[irow, icol] > 31:
                    self._alpha_pixel[irow, icol] = self._alpha_pixel[irow, icol] - 64
                self._alpha_pixel[irow, icol] = self._alpha_pixel[irow, icol] * (1 << self._acc_scale_rem)
                self._alpha_pixel[irow, icol] = self._alpha_pixel[irow, icol] + self._pix_sens_average + (self._acc_row[irow] * (1 << self._acc_scale_row)) + (self._acc_col[icol] * (1 << self._acc_scale_col))
                self._alpha_pixel[irow, icol] = self._alpha_pixel[irow, icol] / np.power(2, self._acc_scale)

                self._Kta_pixel[irow, icol] = (rv & 0xE) >> 1
                KTaRCij = self._KTa_RC_EE[ irow % 2 + (icol % 2) * 2 ]
                self._Kta_pixel[irow, icol] = (self._Kta_pixel[irow, icol] * np.power(2, self._KTa_scale_2) + KTaRCij) / (np.power(2, self._KTa_scale_1))

                self._Kv_pixel[irow, icol] = self._KVPixelValues[ irow % 2 + (icol % 2) * 2 ]





    def _shadow_init(self):
        # Initialize shadow parameters FROM RAM (only the ones that are copied)
        # This is done to allow initialization when they've changed from the values in EEPROM
        self._reg__cr1 = self._i2c_read_word(0x800D)[0]
        self._reg__cr2 = self._i2c_read_word(0x800E)[0]
        self._reg__conf_i2c_conf = self._i2c_read_word(0x800F)[0]
        self._reg__conf_i2c_addr = self._i2c_read_word(0x8010)[0]

        self._update_ram()


    # Update current state from RAM values

    def _update_ram(self):
        # Resolution correction
        res_ee = (self._eeprom[0x2438 - 0x2400] & 0x3000) >> 12
        res_ram = (self._i2c_read_word(0x800D)[0] & 0x0C00) >> 10
        self._resolutioncorr = 2 ** (res_ee - res_ram)

        # Vdd

        r = self._i2c_read_word(0x072A)[0]
        if r > 32767:
            r = r - 65536
        self._Vdd = (r * self._resolutioncorr - self._Vdd25) / self._Kvdd + 3.3

        # Ambient temperature

        dV = self._i2c_read_word(0x72A)[0]
        if dV > 32676:
            dV = dV - 65536
        self._dV = (dV - self._Vdd25) / (self._Kvdd)
        self._Vptat = self._i2c_read_word(0x0720)[0]
        if self._Vptat > 32767:
            self._Vptat = self._Vptat - 65536
        self._Vbe = self._i2c_read_word(0x0700)[0]
        if self._Vbe > 32767:
            self._Vbe = self._Vbe - 65536
        self._Vptatart = (self._Vptat / (self._Vptat * self._alphaptat + self._Vbe)) * 262144
        self._Ta = ((self._Vptatart / (1 + self._Kvptat * self._dV)) - self._Vptat25) / self._Ktptat + 25

        # Gain parameter

        r = self._i2c_read_word(0x070A)[0]
        if r > 32767:
            r = r - 65536
        self._KGain = self._gain / r





    def _i2c_write_word(
        self,
        addr,
        data
    ):
        cmd = struct.pack(">HH", addr, data)
        written = self._i2c.write(self._adr, cmd)
        if not written:
            return False

        # Verify write ...
        # ToDo
        return True

    def _i2c_read_word(
        self,
        addr
    ):
        cmd = struct.pack(">H", addr)
        ind = self._i2c.writeread(
            self._adr,
            cmd,
            2
        )
        if ind is None:
            return None

        return struct.unpack(">H", ind)

    def __repr__(self):
        info = {
            "i2c" : {
                "address" : self._adr
            },
            "pixels" : (self._rows, self._cols),
            "serial" : self._reg__id,
            "Tranges" : [
                ( self._ct1, self._ct2 ),
                ( self._ct2, self._ct3 ),
                ( self._ct3, self._ct4 ),
                ( self._ct4, None )
            ],
            "actual" : {
                "vdd" : self._Vdd,
                "Ta" : self._Ta,
                "gain" : self._gain
            }
        }
        return f"{info}"

if __name__ == "__main__":
    from fbsdi2c import FbsdI2C
    from time import sleep
    with FbsdI2C() as i2c:
        with MLX90640(i2c) as mlx:
            #print(mlx._i2c_read_word(0x800D))
            #mlx._load_eeprom()
            print(mlx)
            sleep(10)
            f1 = mlx._fetch_raw_frame_sync()
            #print(".")
            #sleep(5)
            #f2 = mlx._fetch_raw_frame_sync()
            #print(f1.tolist())
            #print(f2.tolist())
            #print((f2 - f1).tolist())
            f2 = mlx._process_frame(f1)
            print(f2.tolist())
            pass
