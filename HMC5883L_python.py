import smbus

bus = smbus.SMBus(1)

# default i2c address
I2C_ADDRESS = 0x1e

# config and id reg
REG_CONTROL_1 = 0x00
REG_CONTROL_2 = 0x01
REG_MODE = 0x02
REG_ID_1 = 0x0A
REG_ID_2 = 0x0B
REG_ID_3 = 0x0C
REG_ID_4 = 0x0D

# output regs
REG_X_MSB = 0x03
REG_X_LSB = 0x04
REG_Y_MSB = 0x05
REG_Y_LSB = 0x06
REG_Z_MSB = 0x07
REG_Z_LSB = 0x08

# config flags
FLAG_SAMP_PER_MEAS_1 = 0b00000000  # Samples per measurement = 1
FLAG_SAMP_PER_MEAS_2 = 0b00100000  # Samples per measurement = 2
FLAG_SAMP_PER_MEAS_4 = 0b01000000  # Samples per measurement = 4
FLAG_SAMP_PER_MEAS_8 = 0b01100000  # Samples per measurement = 8 0b0100000
FLAG_MEASURE_FREQ_0_75 = 0b00000000  # Output Data Rate = 0.75Hz
FLAG_MEASURE_FREQ_1_5 = 0b00000100  # Output Data Rate = 1.5Hz
FLAG_MEASURE_FREQ_3 = 0b00001000  # Output Data Rate = 3Hz
FLAG_MEASURE_FREQ_7_5 = 0b00001100  # Output Data Rate = 7.5Hz
FLAG_MEASURE_FREQ_15 = 0b00010000  # Output Data Rate = 15Hz
FLAG_MEASURE_FREQ_30 = 0b00010100  # Output Data Rate = 30Hz
FLAG_MEASURE_FREQ_75 = 0b00011000  # Output Data Rate = 75Hz
FLAG_MEAS_NORM = 0b00000000  # Normal Measurement Configuration
FLAG_MEAS_POSITIVE = 0b00000001  # Positive bias configuration for X, Y and Z-axes
FLAG_MEAS_NEGATIVE = 0b00000010  # Negative bias configuration for X, Y and Z-axes
FLAG_GAIN_0_88 = 0b00000000  # Gain = +/- 0.88
FLAG_GAIN_1_3 = 0b00100000  # Gain = +/- 1.3
FLAG_GAIN_1_9 = 0b01000000  # Gain = +/- 1.9
FLAG_GAIN_2_5 = 0b01100000  # Gain = +/- 2.5
FLAG_GAIN_4_0 = 0b10000000  # Gain = +/- 4.0
FLAG_GAIN_4_7 = 0b10100000  # Gain = +/- 4.7
FLAG_GAIN_5_6 = 0b11000000  # Gain = +/- 5.6
FLAG_GAIN_8_1 = 0b11100000  # Gain = +/- 8.1
FLAG_CONTROL_CONTINUOUS = 0b00000000  # continuous
FLAG_CONTROL_SINGLE = 0b00000001
FLAG_CONTROL_OFF = 0b00000010


class HMC5883L(object):
    def __init__(self,
                 address=I2C_ADDRESS,
                 samp_meas=FLAG_SAMP_PER_MEAS_8,
                 meas_freq=FLAG_MEASURE_FREQ_0_75,
                 read_bias=FLAG_MEAS_NORM,
                 gain_correction=FLAG_GAIN_0_88,
                 i2c_bus=bus):
        self.address = address
        self.bus = i2c_bus
        id_reg_asci = ""
        id_reg = self.bus.read_i2c_block_data(address, REG_ID_1, 3)
        for i in id_reg:
            id_reg_asci += str(chr(i))

        # checks if the chip is HMC5883L, failing that checks for a QMC5883L which is a modified version of HMC5883L
        if id_reg_asci != "H43":
            id_reg_alt = self.bus.read_byte_data(self.address, REG_ID_4)
            if id_reg_alt == 0xff:
                print(
                    "chip id {} instead of H43.Chip is a V2/V3 variant of "
                    "alternative magnetometer QMC5883L, not supported by library;".format(
                        id_reg_alt))
            else:
                print(
                    "Returned id {} in ID-registers of HMC5883L, chip not registering correctly.;".format(
                        id_reg_asci))

        elif id_reg_asci == "H43":
            print("Returned chip id is identifiying as QMC5883L")
        # The self.startup_configuration for resuming startup calibration after modifiying results
        self.startup_configuration = (samp_meas, meas_freq, read_bias, gain_correction)
        self.current_configuration = self.startup_configuration
        self.set_continuous_config(self.current_configuration[0], self.current_configuration[1],
                                   self.current_configuration[2], self.current_configuration[3])
        self.mode_continuous()

    def __del__(self):
        self.mode_off()

    # raw mag data
    def get_mag_raw(self):
        mag_xyz = bus.read_i2c_block_data(self.address, REG_X_MSB, 6)
        return mag_xyz

    # mag data for x/y/z coordinate
    def get_mag(self):
        data = self.get_mag_raw()
        x_mag = data[0] * 256 + data[1]
        if x_mag > 32767:
            x_mag -= 65536

        y_mag = data[2] * 256 + data[3]
        if y_mag > 32767:
            y_mag -= 65536

        z_mag = data[4] * 256 + data[5]
        if z_mag > 32767:
            z_mag -= 65536
        return x_mag, y_mag, z_mag

    def set_startup_config(self):
        self.set_continuous_config(self.startup_configuration[0], self.startup_configuration[1],
                                   self.startup_configuration[2], self.startup_configuration[3])

    # Modify config of device. You can modify one aspect at a time, or multiple it will retain current settings.
    def set_continuous_config(self,
                              samp_meas=None, meas_freq=None,
                              flag_meas_samples=None, flag_gain_correction=None):
        samp_meas = self.current_configuration[0] if samp_meas is None else samp_meas
        meas_freq = self.current_configuration[1] if meas_freq is None else meas_freq
        flag_meas_samples = self.current_configuration[2] if flag_meas_samples is None else flag_meas_samples
        flag_gain_correction = self.current_configuration[3] if flag_gain_correction is None else flag_meas_samples
        self.current_configuration = [samp_meas, meas_freq, flag_meas_samples, flag_gain_correction]
        control1_bool = (samp_meas | meas_freq | flag_meas_samples)
        bus.write_byte_data(self.address, REG_CONTROL_1, control1_bool)
        bus.write_byte_data(self.address, REG_CONTROL_2, flag_gain_correction)

    # continuous measurements single measurement or turn device off.
    def mode_continuous(self):
        bus.write_byte_data(self.address, REG_MODE, FLAG_CONTROL_CONTINUOUS)

    def mode_single(self):
        bus.write_byte_data(self.address, REG_MODE, FLAG_CONTROL_SINGLE)

    def mode_off(self):
        bus.write_byte_data(self.address, REG_MODE, FLAG_CONTROL_OFF)
