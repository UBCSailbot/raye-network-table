#ifndef _BMS_PWR_MGR_H_
#define _BMS_PWR_MGR_H_

#include <stdint.h>
#include <linux/can.h>

void OnNewVoltageReading(const uint8_t &bms_frame_id, const float &voltage, struct can_frame &frame);

#endif // _BMS_PWR_MGR_H_
