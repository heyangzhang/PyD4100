import ctypes
import sys
from msl.loadlib import Server32
from time import sleep

class D4100Server(Server32):
    """A wrapper around a 32-bit C++ library, 'cpp_lib32.dll', that has an 'add' function."""

    def __init__(self, host, port, **kwargs):
        # Load the 'cpp_lib32' shared-library file using ctypes.CDLL.
        path = 'lib/D4100_usb.dll'
        super(D4100Server, self).__init__(path, 'cdll', host, port)
        self.lib.GetFPGARev.restype = ctypes.c_uint
        self.rows = 1080
        self.cols = 2048

    @staticmethod
    def _split_bytes(rev):
        return (rev&0xFF00)>>8,rev&0x00FF

    @staticmethod
    def _split_wbytes(rev):
        return (rev&0xFFFF0000)>>16,rev&0x0000FFFF
    
    def program_FPGA(self, fpga, size, devnum):
        return self.lib.program_FPGA(fpga,size,devnum)

    # short GetNumDev( )
    def get_num_dev(self):
        # The Server32 class has a 'lib' property that is a reference to the ctypes.CDLL object.
        print('counting num dev')
        return self.lib.GetNumDev()

    # short GetDMDTYPE(short DeviceNumber)
    def get_dmd_type(self, devnum):
        return self.lib.GetDMDTYPE(devnum)

    # unsigned int GetDLLRev( )
    def get_dll_rev(self):
        rev = self.lib.GetDLLRev()
        return self._split_wbytes(rev)

    # unsigned int GetDriverRev(short DeviceNumber)
    def get_driver_rev(self,devnum):
        rev = self.lib.GetDriverRev(devnum)
        return (rev&0xFF000000)>>24,(rev&0x00FF0000)>>16,(rev&0x0000FF00)>>8,(rev&0x000000FF)

    # unsigned int GetFirmwareRev(short DeviceNumber)
    def get_firmware_rev(self,devnum):
        rev = self.lib.GetFirmwareRev(devnum)
        return self._split_bytes(rev)

    # unsigned int GetFPGARev(short DeviceNumber)
    def get_fpga_rev(self,devnum):        
        rev = self.lib.GetFPGARev(devnum)
        return rev&0xFF, (rev&0xF00)>>8, (rev&0xF000)>>12

    # short int GetUsbSpeed(short DeviceNumber)
    def get_usb_speed(self, devnum):
        speed = self.lib.GetUsbSpeed(devnum)
        return speed

    # short GetDDCVERSION(short DeviceNumber)
    def get_ddc_version(self,devnum):
        ret = self.lib.GetDDCVERSION(devnum)
        return ret&0b0111

    # short GetGPIO(short DeviceNumber)
    def get_gpio(self,devnum):
        ret = self.lib.GetGPIO(devnum)
        return (ret&0b11100)>>2

    # short SetGPIO(short value, short DeviceNumber)
    def set_gpio(self,devnum,gpio_val):
        return self.lib.SetGPIO(gpio_val,devnum)

    # short GetTPGEnable(short DeviceNumber)
    def get_tpg_enable(self,devnum):
        ret = self.lib.GetTPGEnable(devnum)
        return ret

    # short SetTPGEnable(short value, short DeviceNumber)
    def set_tpg_enable(self,devnum,val):
        self.lib.SetTPGEnable(val,devnum)

    # short GetPatternForce(short DeviceNumber)
    def get_pattern_force(self,devnum):
        ret = self.lib.GetPatternForce(devnum)
        return ret
    # short SetPatternForce(short value, short DeviceNumber)
    def set_pattern_force(self,devnum,val):
        self.lib.SetTPGEnable(val,devnum)

    # short GetPatternSelect(short DeviceNumber)
    def get_pattern_select(self,devnum):
        ret = self.lib.GetPatternSelect(devnum)
        return ret

    # short SetPatternSelect(short value, short DeviceNumber)
    def set_pattern_select(self,devnum,val):
        return self.lib.SetPatternSelect(val,devnum)

    # short GetPWRFLOAT(short DeviceNumber)
    def get_pwr_float(self,devnum):
        ret = self.lib.GetPWRFLOAT(devnum)
        return ret

    # short SetPWRFLOAT(short value, short DeviceNumber)
    def set_pwr_float(self,devnum,val):
        return self.lib.SetPWRFLOAT(val,devnum)

    # short GetRowMd(short DeviceNumber)
    def get_row_mode(self,devnum):
        return self.lib.GetRowMd(devnum)

    # short SetRowMd(short value, short DeviceNumber)
    def set_row_mode(self,devnum,val):
        return self.lib.SetRowMd(val, devnum)

    # short GetBlkMd(short DeviceNumber)
    def get_block_mode(self,devnum):
        return self.lib.GetBlkMd(devnum)

    # short SetBlkMd(short value, short DeviceNumber)
    def set_block_mode(self,devnum,val):
        return self.lib.SetBlkMd(val,devnum)

    # short GetBlkAd(short DeviceNumber)
    def get_block_address(self,devnum):
        return self.lib.GetBlkAd(devnum)

    # short SetBlkAd(short value, short DeviceNumber)
    def set_block_address(self,devnum,val):
        return self.lib.SetBlkAd(val,devnum)

    # short GetRowAddr(short DeviceNumber)
    def get_row_address(self,devnum):
        return self.lib.GetRowAddr(devnum)

    # short SetRowAddr(short value, short DeviceNumber)
    def set_row_address(self,devnum,val):
        return self.lib.SetRowAddr(val,devnum)

    # short LoadControl(short DeviceNumber)
    def load_control(self,devnum):
        return self.lib.LoadControl(devnum)
    
    def wait(self):
        sleep(0.001)

    def load3(self,devnum):
        self.wait()
        self.load_control(devnum)
        self.wait()
        self.load_control(devnum)
        self.wait()
        self.load_control(devnum)
        self.wait()

    def float_mirrors(self,devnum):
        self.set_row_mode(devnum,0b00)
        self.set_block_mode(devnum,0b11)
        self.set_block_address(devnum,0b1100)
        self.load3(devnum)

    def global_reset(self,devnum):
        """
        https://e2e.ti.com/support/dlp/f/94/t/819526
        A Reset operation using DLL functions consists of
        1. setting the Row Mode to NoOp [§ 6.2.10]
        2. the Block Address to the desired block(s) [§ 6.2.6]
        3. the Block Mode [§ 6.2.4]
        you may also need to set RST2BLKZ if using dual block [§ 6.2.8]
        and then calling Load Control [§ 6.2.1] to write these values to the DMD.
        I have found in practice that calling Load Control three times consecutively ensures that it writes over the USB interface. 
        Arguments:
            devnum {[type]} -- [description]
        """
        self.set_row_mode(devnum,0b00)
        self.wait()
        self.set_block_mode(devnum,0b11)
        self.wait()
        self.set_block_address(devnum,0b1000)
        self.load3(devnum)

    # short ClearFifos(short DeviceNumber)
    def clear_fifos(self,devnum):
        return self.lib.ClearFifos(devnum)

    # short SetNSFLIP(short value, short DeviceNumber)
    def set_ns_flip(self,devnum,val):
        return self.lib.SetNSFLIP(val,devnum)

    # short GetNSFLIP(short DeviceNumber)
    def get_ns_flip(self,devnum):
        return self.lib.GetNSFLIP(devnum)
    
    # short GetRST2BLKZ(short DeviceNumber)
    def get_rst2blkz(self,devnum):
        return self.lib.GetRST2BLKZ(devnum)
    
    # short SetRST2BLKZ(short value, short DeviceNumber)
    def set_rst2blkz(self,devnum, val):
        return self.lib.SetRST2BLKZ(val,devnum)
    
    # short GetLoad4(short DeviceNumber)
    def get_load4(self,devnum):
        return self.lib.GetLoad4(devnum)
    
    # short SetLoad4(short value, short DeviceNumber)
    def set_load4(self,devnum, val):
        return self.lib.SetLoad4(val,devnum)
    
    # int RegisterWrite(unsigned short regAddress, unsigned short data, short devNumber);
    def register_write(self, regAddress, data, devnum):
        return self.lib.RegisterWrite(regAddress, data, devnum)
    
    def register_read(self, regAddress, devnum):
        return self.lib.RegisterRead(regAddress, devnum)
    
# int GetDescriptor(int*, short DeviceNum)
# short SetCOMPDATA(short value, short DeviceNumber)
# short GetCOMPDATA(short DeviceNumber)
# short SetWDT(short value, short DeviceNumber)
    def set_WDT(self,devnum, val):
        return self.lib.SetWDT(val,devnum)
# short GetWDT(short DeviceNumber)
    def get_WDT(self,devnum):
        return self.lib.SetWDT(devnum)

    def _load_wrapper(self,devnum,image,num_ims = 0):
        if self.register_write(39,num_ims,0)==0:
            raise Exception("didn't set image count")
        self.wait()
        blocks = 15*(num_ims+1)
        block_size = (num_ims+1)*(self.cols*self.rows)//(blocks)
        for i in range(0, blocks):
            data = image[i*block_size:(i+1)*block_size]
            if self.load_data(devnum,data) == 0:
                raise Exception("didn't load")
            self.wait()

        #if block_size_track != self.cols*self.rows:
        #    raise Exception(f"data size wrong {self.cols*self.rows} is not {block_size_track}")

    def set_image(self, devnum, image, num_ims = 0):
        self.set_WDT(devnum,1)
        self.set_tpg_enable(devnum,0)
        self.clear_fifos(devnum)
        self._load_wrapper(devnum,image,num_ims)
    
    def set_all_mirrors(self,devnum,val):
        data_size = self.rows*self.cols
        image = [val for x in range(data_size)]
        self.set_image(devnum,image)

    def all_mirrors_off(self,devnum):
        if self.set_all_mirrors(devnum,0) == 0:
            raise Exception("didn't load")

    def all_mirrors_on(self,devnum):
        if self.set_all_mirrors(devnum,255) == 0:
            raise Exception("didn't load")

    # int LoadData(UCHAR* RowData, unsigned int length, short DMDType, short DeviceNumber)
    # takes in a python list and converts it to ctype for the dmd
    '''
    However, the note about length is still correct. 
    This is the length in bytes of the data in RowData.
    If you are only putting the length as the length of one row in pixels (i.e. 1920), then it will load 8 rows.
    If there are only 1920 bits (240 bytes) in the data, then it will not work, because it will run out of bytes in RowData and return an error.
    Try 240 instead of 1920 if you are only loading 1 row of data.  I have verified this is the length in bytes, NOT bits.  

    The DLPC410 is only a binary controller.  Each pattern must be 1-bit per pixel.  
    The controller does not do any gray-scale values, so that it is 240 bytes = 1920 bits for one row.'''
    def load_data(self,devnum,data):
        dmd_type = self.get_dmd_type(devnum)
        dlen = len(data)
        dlen = dlen//8
        #TODO: make 10 a variable threshold
        binary_data = [i>0 for i in data]
        data_list = []
        binary_convert = [1,2,4,8,16,32,64,128]
        for i in range(dlen):
            list8 = binary_data[i*8:(i+1)*8]
            res = [a*b for a,b in zip(binary_convert,list8)]
            data_list.append(sum(res))
        b_data = (ctypes.c_ubyte * dlen)(*data_list)
        return self.lib.LoadData(ctypes.pointer(b_data),dlen,dmd_type,devnum)
    
    
    '''
    def set_gray(self,devnum,im_list):
        self.set_WDT(devnum,1)
        self.set_tpg_enable(devnum,0)
        self.clear_fifos(devnum)
        self._load_gray_wrapper(devnum,im_list)


    def _set_image(self,devnum,im_list):

        blocks = 15
        block_size = (1920*self.rows)//blocks

        self.set_tpg_enable(devnum,0)
        self.clear_fifos(devnum)

        self.set_row_mode(devnum,0b11)
        self.load3(devnum)
        data = im_list[0:1920]
        if self.load_data(devnum,data) == 0:
                raise Exception("didn't load row 1")
        self.wait()
        self.set_row_mode(devnum,0b01)
        self.load3(devnum)
        data = im_list[1920:block_size]
        if self.load_data(devnum,data) == 0:
                raise Exception("didn't load block 1")
        self.wait()
        # load rows
        for i in range(1,blocks):
            data = im_list[i*block_size:(i+1)*block_size]
            if self.load_data(devnum,data) == 0:
                raise Exception("didn't load")
            self.wait()
        self.global_reset(devnum)

        self.clear_fifos(devnum)
        self.set_block_mode(devnum,0b00)
        self.set_row_mode(devnum,0b00)
        self.load3(devnum)

    def _load_gray_wrapper(self,devnum,im_list):
        num_ims = len(im_list)
        images = im_list
        if self.register_write(39,num_ims-1,0)==0:
            raise Exception("didn't set multiple images")
        blocks = 2
        block_size = (self.cols*self.rows)//blocks
        for im in images:
            for i in range(blocks):
                data = im[i*block_size:(i+1)*block_size]
                if self.load_data(devnum,data) == 0:
                    raise Exception("didn't load")
                self.wait()
    '''

    # short SetEXTRESETENBL(short value, short DeviceNumber)
# short GetEXTRESETENBL(short DeviceNumber)
# short GetRESETCOMPLETE(int waittime, short int DeviceNumber)
# short SetGPIORESETCOMPLETE(short DeviceNumber)
# short GetSWOverrideEnable(short DeviceNumber)
# short SetSWOverrideEnable(short value, short DeviceNumber)
# short GetSWOverrideValue(short DeviceNumber)
# short SetSWOverrideValue(short value, short DeviceNumber)