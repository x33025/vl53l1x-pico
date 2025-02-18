#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "VL53L1X.h" // your translated header

// Constructors ////////////////////////////////////////////////////////////////

VL53L1X::VL53L1X(i2c_inst_t *i2c_port, uint8_t i2c_address, uint16_t io_timeout_ms)
  : _i2c_port(i2c_port)
  , address(i2c_address)
  , io_timeout(io_timeout_ms)
  , did_timeout(false)
  , calibrated(false)
  , saved_vhv_init(0)
  , saved_vhv_timeout(0)
  , distance_mode(Unknown)
{
  // Nothing else needed here unless you want to auto-init the sensor
}

// Public Methods //////////////////////////////////////////////////////////////

void VL53L1X::setAddress(uint8_t new_addr)
{
  writeReg(I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
  address = new_addr;
}

// Initialize sensor using settings taken mostly from VL53L1_DataInit() and
// VL53L1_StaticInit(). If io_2v8 is true, the sensor is configured for 2.8 V mode.
bool VL53L1X::init(bool io_2v8)
{
  // check model ID and module type registers (values specified in datasheet)
  if (readReg16Bit(IDENTIFICATION__MODEL_ID) != 0xEACC)
  {
    return false;
  }

  // VL53L1_software_reset() begin
  writeReg(SOFT_RESET, 0x00);
  sleep_us(100);
  writeReg(SOFT_RESET, 0x01);

  // give it some time to boot
  sleep_ms(1);

  // VL53L1_poll_for_boot_completion() begin
  startTimeout();
  while (((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0) || (last_status != 0))
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return false;
    }
  }
  // VL53L1_poll_for_boot_completion() end

  // VL53L1_software_reset() end

  // VL53L1_DataInit() begin

  // sensor uses 1V8 mode by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    writeReg(PAD_I2C_HV__EXTSUP_CONFIG,
      readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
  }

  // store oscillator info for later use
  fast_osc_frequency = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
  osc_calibrate_val  = readReg16Bit(RESULT__OSC_CALIBRATE_VAL);

  // VL53L1_DataInit() end

  // VL53L1_StaticInit() begin

  writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate);
  writeReg(GPIO__TIO_HV_STATUS, 0x02);
  writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);
  writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
  writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  writeReg(ALGO__RANGE_MIN_CLIP, 0);
  writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);

  writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
  writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
  writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360);
  writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);

  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  writeReg(SD_CONFIG__QUANTIFIER, 2);

  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  writeReg(SYSTEM__SEED_CONFIG, 1);
  writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B);
  writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2);

  // default to long range, 50 ms timing budget
  setDistanceMode(Long);
  setMeasurementTimingBudget(50000);

  // the API triggers this once a measurement is started; we do it now
  writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
                readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);

  return true;
}

// Write an 8-bit register
void VL53L1X::writeReg(uint16_t reg, uint8_t value)
{
  uint8_t data[3];
  data[0] = (uint8_t)(reg >> 8);  // reg high byte
  data[1] = (uint8_t)(reg & 0xFF);// reg low byte
  data[2] = value;

  int num_written = i2c_write_blocking(_i2c_port, address, data, 3, false);
  // If the number of bytes written != 3, treat it as an error
  last_status = (num_written == 3) ? 0 : 1;
}

// Write a 16-bit register
void VL53L1X::writeReg16Bit(uint16_t reg, uint16_t value)
{
  uint8_t data[4];
  data[0] = (uint8_t)(reg >> 8);
  data[1] = (uint8_t)(reg & 0xFF);
  data[2] = (uint8_t)(value >> 8);
  data[3] = (uint8_t)(value & 0xFF);

  int num_written = i2c_write_blocking(_i2c_port, address, data, 4, false);
  last_status = (num_written == 4) ? 0 : 1;
}

// Write a 32-bit register
void VL53L1X::writeReg32Bit(uint16_t reg, uint32_t value)
{
  uint8_t data[6];
  data[0] = (uint8_t)(reg >> 8);
  data[1] = (uint8_t)(reg & 0xFF);
  data[2] = (uint8_t)(value >> 24);
  data[3] = (uint8_t)(value >> 16);
  data[4] = (uint8_t)(value >>  8);
  data[5] = (uint8_t)(value & 0xFF);

  int num_written = i2c_write_blocking(_i2c_port, address, data, 6, false);
  last_status = (num_written == 6) ? 0 : 1;
}

// Read an 8-bit register
uint8_t VL53L1X::readReg(regAddr reg)
{
  // First, send the 16-bit register address
  uint8_t addrBuf[2];
  addrBuf[0] = (uint8_t)(reg >> 8);
  addrBuf[1] = (uint8_t)(reg & 0xFF);

  int written = i2c_write_blocking(_i2c_port, address, addrBuf, 2, false);
  last_status = (written == 2) ? 0 : 1;

  // Then read one byte back
  uint8_t value = 0;
  int readCount = i2c_read_blocking(_i2c_port, address, &value, 1, false);
  if (readCount != 1) { last_status = 1; }

  return value;
}

// Read a 16-bit register
uint16_t VL53L1X::readReg16Bit(uint16_t reg)
{
  // Send register address
  uint8_t addrBuf[2];
  addrBuf[0] = (uint8_t)(reg >> 8);
  addrBuf[1] = (uint8_t)(reg & 0xFF);

  int written = i2c_write_blocking(_i2c_port, address, addrBuf, 2, false);
  last_status = (written == 2) ? 0 : 1;

  // Read two bytes
  uint8_t data[2] = {0,0};
  int readCount = i2c_read_blocking(_i2c_port, address, data, 2, false);
  if (readCount != 2) { last_status = 1; }

  uint16_t value = (uint16_t)data[0] << 8;
  value |= data[1];

  return value;
}

// Read a 32-bit register
uint32_t VL53L1X::readReg32Bit(uint16_t reg)
{
  // Send register address
  uint8_t addrBuf[2];
  addrBuf[0] = (uint8_t)(reg >> 8);
  addrBuf[1] = (uint8_t)(reg & 0xFF);

  int written = i2c_write_blocking(_i2c_port, address, addrBuf, 2, false);
  last_status = (written == 2) ? 0 : 1;

  // Read four bytes
  uint8_t data[4] = {0,0,0,0};
  int readCount = i2c_read_blocking(_i2c_port, address, data, 4, false);
  if (readCount != 4) { last_status = 1; }

  uint32_t value  = (uint32_t)data[0] << 24;
           value |= (uint32_t)data[1] << 16;
           value |= (uint32_t)data[2] <<  8;
           value |= (uint32_t)data[3];
  return value;
}

// set distance mode to Short, Medium, or Long
bool VL53L1X::setDistanceMode(DistanceMode mode)
{
  // save existing timing budget
  uint32_t budget_us = getMeasurementTimingBudget();

  switch (mode)
  {
    case Short:
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
      writeReg(SD_CONFIG__WOI_SD0, 0x07);
      writeReg(SD_CONFIG__WOI_SD1, 0x05);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6);
      break;

    case Medium:
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
      writeReg(SD_CONFIG__WOI_SD0, 0x0B);
      writeReg(SD_CONFIG__WOI_SD1, 0x09);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10);
      break;

    case Long:
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
      writeReg(SD_CONFIG__WOI_SD0, 0x0F);
      writeReg(SD_CONFIG__WOI_SD1, 0x0D);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14);
      break;

    default:
      return false;
  }

  setMeasurementTimingBudget(budget_us);
  distance_mode = mode;

  return true;
}

// set measurement timing budget in microseconds
bool VL53L1X::setMeasurementTimingBudget(uint32_t budget_us)
{
  if (budget_us <= TimingGuard) { return false; }

  uint32_t range_config_timeout_us = budget_us - TimingGuard;
  if (range_config_timeout_us > 1100000) { return false; }

  range_config_timeout_us /= 2; // used for both A and B

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // Phasecal timeout (defaults 1000 us)
  uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) phasecal_timeout_mclks = 0xFF;
  writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // MM Timing A
  writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // Range Timing A
  writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B"
  macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));

  // MM Timing B
  writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // Range Timing B
  writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  return true;
}

// get measurement timing budget in microseconds
uint32_t VL53L1X::getMeasurementTimingBudget()
{
  uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(
    decodeTimeout(readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  return (2 * range_config_timeout_us) + TimingGuard;
}

void VL53L1X::setROISize(uint8_t width, uint8_t height)
{
  if (width > 16)  { width  = 16; }
  if (height > 16) { height = 16; }

  if (width > 10 || height > 10)
  {
    writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199);
  }

  writeReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
           ((height - 1) << 4) | (width - 1));
}

void VL53L1X::getROISize(uint8_t * width, uint8_t * height)
{
  uint8_t reg_val = readReg(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
  *width  = (reg_val & 0xF) + 1;
  *height = (reg_val >> 4) + 1;
}

void VL53L1X::setROICenter(uint8_t spadNumber)
{
  writeReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD, spadNumber);
}

uint8_t VL53L1X::getROICenter()
{
  return readReg(ROI_CONFIG__USER_ROI_CENTRE_SPAD);
}

// Start continuous ranging measurements
void VL53L1X::startContinuous(uint32_t period_ms)
{
  writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
  writeReg(SYSTEM__MODE_START, 0x40);
}

// Stop continuous measurements
void VL53L1X::stopContinuous()
{
  writeReg(SYSTEM__MODE_START, 0x80);

  calibrated = false;

  if (saved_vhv_init != 0)     { writeReg(VHV_CONFIG__INIT, saved_vhv_init); }
  if (saved_vhv_timeout != 0)  { writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout); }

  writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);
}

// Read a range in continuous mode
uint16_t VL53L1X::read(bool blocking)
{
  if (blocking)
  {
    startTimeout();
    while (!dataReady())
    {
      if (checkTimeoutExpired())
      {
        did_timeout = true;
        return 0;
      }
    }
  }

  readResults();

  if (!calibrated)
  {
    setupManualCalibration();
    calibrated = true;
  }

  updateDSS();
  getRangingData();

  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);

  return ranging_data.range_mm;
}

// Single-shot read
uint16_t VL53L1X::readSingle(bool blocking)
{
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
  writeReg(SYSTEM__MODE_START, 0x10);

  if (blocking)
  {
    return read(true);
  }
  else
  {
    return 0;
  }
}

const char * VL53L1X::rangeStatusToString(RangeStatus status)
{
  switch (status)
  {
    case RangeValid:                return "range valid";
    case SigmaFail:                 return "sigma fail";
    case SignalFail:                return "signal fail";
    case RangeValidMinRangeClipped: return "range valid, min range clipped";
    case OutOfBoundsFail:           return "out of bounds fail";
    case HardwareFail:              return "hardware fail";
    case RangeValidNoWrapCheckFail: return "range valid, no wrap check fail";
    case WrapTargetFail:            return "wrap target fail";
    case XtalkSignalFail:           return "xtalk signal fail";
    case SynchronizationInt:         return "synchronization int";
    case MinRangeFail:              return "min range fail";
    case None:                      return "no update";
    default:                        return "unknown status";
  }
}

bool VL53L1X::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

void VL53L1X::setupManualCalibration()
{
  saved_vhv_init    = readReg(VHV_CONFIG__INIT);
  saved_vhv_timeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  writeReg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F);
  writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
           (saved_vhv_timeout & 0x03) + (3 << 2));

  writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
  writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
}

void VL53L1X::readResults()
{
  // We need to read 17 bytes starting at RESULT__RANGE_STATUS
  uint16_t startReg = RESULT__RANGE_STATUS;
  uint8_t addrBuf[2];
  addrBuf[0] = (uint8_t)(startReg >> 8);
  addrBuf[1] = (uint8_t)(startReg & 0xFF);

  // Send address
  int written = i2c_write_blocking(_i2c_port, address, addrBuf, 2, false);
  last_status = (written == 2) ? 0 : 1;

  // Read 17 bytes
  uint8_t buffer[17];
  memset(buffer, 0, 17);
  int readCount = i2c_read_blocking(_i2c_port, address, buffer, 17, false);
  if (readCount != 17) { last_status = 1; }

  results.range_status = buffer[0];
  // buffer[1] is report_status (not used)
  results.stream_count = buffer[2];

  results.dss_actual_effective_spads_sd0 =
    ((uint16_t)buffer[3] << 8) | buffer[4];

  // buffer[5..6] = peak_signal_count_rate_mcps_sd0 (unused)
  results.ambient_count_rate_mcps_sd0 =
    ((uint16_t)buffer[7] << 8) | buffer[8];

  // buffer[9..10] = sigma_sd0 (unused)
  // buffer[11..12] = phase_sd0 (unused)
  results.final_crosstalk_corrected_range_mm_sd0 =
    ((uint16_t)buffer[13] << 8) | buffer[14];

  results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
    ((uint16_t)buffer[15] << 8) | buffer[16];
}

void VL53L1X::updateDSS()
{
  uint16_t spadCount = results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    uint32_t totalRatePerSpad =
      (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      results.ambient_count_rate_mcps_sd0;

    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    totalRatePerSpad <<= 16;
    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      return;
    }
  }

  // fallback
  writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

void VL53L1X::getRangingData()
{
  uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

  // apply gain factor 2011 (≈98%) as in ST's API
  ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) >> 11;

  switch (results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2:  // VCSELWATCHDOGTESTFAILURE
    case 1:  // VCSELCONTINUITYTESTFAILURE
    case 3:  // NOVHVVALUEFOUND
      ranging_data.range_status = HardwareFail;
      break;
    case 13: // USERROICLIP
      ranging_data.range_status = MinRangeFail;
      break;
    case 18: // GPHSTREAMCOUNT0READY
      ranging_data.range_status = SynchronizationInt;
      break;
    case 5:  // RANGEPHASECHECK
      ranging_data.range_status = OutOfBoundsFail;
      break;
    case 4:  // MSRCNOTARGET
      ranging_data.range_status = SignalFail;
      break;
    case 6:  // SIGMATHRESHOLDCHECK
      ranging_data.range_status = SigmaFail;
      break;
    case 7:  // PHASECONSISTENCY
      ranging_data.range_status = WrapTargetFail;
      break;
    case 12: // RANGEIGNORETHRESHOLD
      ranging_data.range_status = XtalkSignalFail;
      break;
    case 8:  // MINCLIP
      ranging_data.range_status = RangeValidMinRangeClipped;
      break;
    case 9:  // RANGECOMPLETE
      if (results.stream_count == 0)
        ranging_data.range_status = RangeValidNoWrapCheckFail;
      else
        ranging_data.range_status = RangeValid;
      break;
    default:
      ranging_data.range_status = None;
  }

  ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

// Decode sequence step timeout in MCLKs
uint32_t VL53L1X::decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from MCLKs
uint16_t VL53L1X::encodeTimeout(uint32_t timeout_mclks)
{
  if (timeout_mclks == 0) { return 0; }

  uint32_t ls_byte = timeout_mclks - 1;
  uint16_t ms_byte = 0;

  while ((ls_byte & 0xFFFFFF00) > 0)
  {
    ls_byte >>= 1;
    ms_byte++;
  }

  return (ms_byte << 8) | (ls_byte & 0xFF);
}

// Convert MCLKs -> microseconds
uint32_t VL53L1X::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  // uses 12.12 fixed point
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert microseconds -> MCLKs
uint32_t VL53L1X::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  // 12.12 fixed point
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12) with given VCSEL period
uint32_t VL53L1X::calcMacroPeriod(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  uint32_t pll_period_us = ((uint32_t)1 << 30) / fast_osc_frequency;

  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // 2304 * PLL_period_us >> 6
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}
