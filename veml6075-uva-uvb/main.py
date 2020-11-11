import time
from micropython import const


class I2CDevice:
   
    def __init__(self, i2c, device_address, probe=True):

        self.i2c = i2c
        self._has_write_read = hasattr(self.i2c, "writeto_then_readfrom")
        self.device_address = device_address

        if probe:
            self.__probe_for_device()

    def readinto(self, buf, *, start=0, end=None):
     
        if end is None:
            end = len(buf)
        rbuf=memoryview(buf)[start:end]
        self.i2c.readfrom_into(self.device_address, rbuf)
        #print(start,end) 
        #print("rd",rbuf,buf)
        

    def write(self, buf, *, start=0, end=None, stop=True):
       
        if end is None:
            end = len(buf)      
        wbuf=memoryview(buf)[start:end]
        #print(start,end) 
        #print("wt:",wbuf,buf)       
        self.i2c.writeto(self.device_address, wbuf,  stop)

    # pylint: disable-msg=too-many-arguments
    def write_then_readinto(
        self,
        out_buffer,
        in_buffer,
        *,
        out_start=0,
        out_end=None,
        in_start=0,
        in_end=None,
        stop=False
    ):
        
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)
        if stop:
            raise ValueError("Stop must be False. Use writeto instead.")
        if self._has_write_read:
            # In linux, at least, this is a special kernel function call
            self.i2c.writeto_then_readfrom(
                self.device_address,
                out_buffer,
                in_buffer,
                out_start=out_start,
                out_end=out_end,
                in_start=in_start,
                in_end=in_end,
            )

        else:
            # If we don't have a special implementation, we can fake it with two calls
            #print("wtn")
            self.write(out_buffer, start=out_start, end=out_end, stop=False)
            #print("rde")
            self.readinto(in_buffer, start=in_start, end=in_end)
            #print("done")

    # pylint: enable-msg=too-many-arguments

    def __enter__(self):
        #while not self.i2c.try_lock():
        #    pass
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        #self.i2c.unlock()
        return False

    def __probe_for_device(self):
       
        #while not self.i2c.try_lock():
        #    pass
        try:
            self.i2c.writeto(self.device_address, b"")
        except OSError:
            # some OS's dont like writing an empty bytesting...
            # Retry by reading a byte
            try:
                result = bytearray(1)
                self.i2c.readfrom_into(self.device_address, result)
            except OSError:
                raise ValueError("No I2C device at address: %x" % self.device_address)
        finally:
            #self.i2c.unlock()
            print("-")

_VEML6075_ADDR = const(0x10)
_REG_CONF = const(0x00)
_REG_UVA = const(0x07)
_REG_DARK = const(0x08)  # check is true?
_REG_UVB = const(0x09)
_REG_UVCOMP1 = const(0x0A)
_REG_UVCOMP2 = const(0x0B)
_REV_ID = const(0x0C)

# Valid constants for UV Integration Time
_VEML6075_UV_IT = {50: 0x00, 100: 0x01, 200: 0x02, 400: 0x03, 800: 0x04}


class VEML6075:


    def __init__(
        self,
        i2c_bus,
        *,
        integration_time=50,
        high_dynamic=True,
        uva_a_coef=2.22,
        uva_b_coef=1.33,
        uvb_c_coef=2.95,
        uvb_d_coef=1.74,
        uva_response=0.001461,
        uvb_response=0.002591
    ):
        # Set coefficients
        self._a = uva_a_coef
        self._b = uva_b_coef
        self._c = uvb_c_coef
        self._d = uvb_d_coef
        self._uvaresp = uva_response
        self._uvbresp = uvb_response
        self._uvacalc = self._uvbcalc = None

        # Init I2C
        self._i2c = I2CDevice(i2c_bus, _VEML6075_ADDR)
        self._buffer = bytearray(3)

        # read ID!
        print("init");
        veml_id = self._read_register(_REV_ID)
        print("veml_id",veml_id);
        if veml_id != 0x26:
            raise RuntimeError("Incorrect VEML6075 ID 0x%02X" % veml_id)

        # shut down
        print("shutdown")
        self._write_register(_REG_CONF, 0x01)

        # Set integration time
        self.integration_time = integration_time

        # enable
        conf = self._read_register(_REG_CONF)
        if high_dynamic:
            conf |= 0x08
        conf &= ~0x01  # Power on
        self._write_register(_REG_CONF, conf)

    def _take_reading(self):
        """Perform a full reading and calculation of all UV calibrated values"""
        time.sleep(0.1)
        uva = self._read_register(_REG_UVA)
        uvb = self._read_register(_REG_UVB)
        # dark = self._read_register(_REG_DARK)
        uvcomp1 = self._read_register(_REG_UVCOMP1)
        uvcomp2 = self._read_register(_REG_UVCOMP2)
        # Equasion 1 & 2 in App note, without 'golden sample' calibration
        self._uvacalc = uva - (self._a * uvcomp1) - (self._b * uvcomp2)
        self._uvbcalc = uvb - (self._c * uvcomp1) - (self._d * uvcomp2)
        # print("UVA = %d, UVB = %d, UVcomp1 = %d, UVcomp2 = %d, Dark = %d" %
        #      (uva, uvb, uvcomp1, uvcomp2, dark))

    @property
    def uva(self):
        """The calibrated UVA reading, in 'counts' over the sample period"""
        self._take_reading()
        return self._uvacalc

    @property
    def uvb(self):
        """The calibrated UVB reading, in 'counts' over the sample period"""
        self._take_reading()
        return self._uvbcalc

    @property
    def uv_index(self):
        """The calculated UV Index"""
        self._take_reading()
        return ((self._uvacalc * self._uvaresp) + (self._uvbcalc * self._uvbresp)) / 2

    @property
    def integration_time(self):
        """The amount of time the VEML is sampling data for, in millis.
        Valid times are 50, 100, 200, 400 or 800ms"""
        key = (self._read_register(_REG_CONF) >> 4) & 0x7
        for k, val in enumerate(_VEML6075_UV_IT):
            if key == k:
                return val
        raise RuntimeError("Invalid integration time")

    @integration_time.setter
    def integration_time(self, val):
        if not val in _VEML6075_UV_IT.keys():
            raise RuntimeError("Invalid integration time")
        conf = self._read_register(_REG_CONF)
        conf &= ~0b01110000  # mask off bits 4:6
        conf |= _VEML6075_UV_IT[val] << 4
        self._write_register(_REG_CONF, conf)

    def _read_register(self, register):
        """Read a 16-bit value from the `register` location"""
        #print("rd:",register);
        self._buffer[0] = register
        with self._i2c as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1, in_end=2)
        return (self._buffer[1] << 8) | self._buffer[0]

    def _write_register(self, register, value):
        """Write a 16-bit value to the `register` location"""        
        self._buffer[0] = register
        self._buffer[1] = value
        self._buffer[2] = value >> 8
        #print("wt:",self._buffer);
        with self._i2c as i2c:
            i2c.write(self._buffer)
            
import machine
veml_i2c = machine.I2C(scl = machine.Pin(14), sda = machine.Pin(2), freq = 50000)
print(veml_i2c.scan())
veml = VEML6075(veml_i2c, integration_time=100)
while True:
  print("uva:",veml.uva)
  print("uvb:",veml.uvb)
  print("uv-idx:",veml.uv_index)
  time.sleep(1)





