"""NOTE: These are Not actually constants.
If referenced in another file, the values can
be reassigned in the scope of that specific project.

CAN_to_URI_to_ROStopic.py

A giant nested dictionary containing the attributes of all
the data that is sent to the NUC. This file uses data from
all the imported files below so take a look at those. Some
of these data frames come from the CAN bus and some from
the AIS (which is attached to the AIS listener).
An explanation of the entries in the dictionary is provided
below:

'WS1_SPEED' ... 'ACCELEROMETER_Z' - The sensor ID tag. These
are sort of a reference to the sensor tags given in
frame_parser.h and indicates which sensor we're dealing with
and what its attributes are.

'id' - The CAN bus ID corresponding to the sensor tag. These
are referenced in frame_parser.h/frame_parser.py.

'rostopic' - The NUC ROStopic corresponding to the sensor tag.
These are referenced in uri.py.

'data' - The data field to be filled in when running dummy
tests. These fields get filled in with whatever you decide it
to be. For datapath testing, we fill this value in with dummy
data that is found in the dummy_data.py file.

'parser' - The callback function that does pre-parsing on the
data you are trying to send through the CAN bus. This is
referenced in frame_parser.py and made use of in the
make_dummy_tests function in dummy_data.py

'parsed_data' - The pre-parsed CAN data that is filled in after
running the parser function in the dictionary. This data is made
use of in the make_dummy_tests function of dummy_data.py.


The CAN_bus_sensor list acts as a filter when making dummy_tests
for the CAN_BBB_NUC datapath test


"""


from .uri import *
from .rostopic import *
from .frame_parser import *


CAN_to_URI_to_ROStopic = {
                          'WS1_FRAME': {'id': WIND_SENS1_FRAME_ID,
                                        'uri': [WIND1_SPEED,
                                                WIND1_ANGLE],
                                        'rostopic': [WIND1_SPEED_TOPIC,
                                                     WIND1_ANGLE_TOPIC],
                                        'data': None,
                                        'parser': [GET_WIND_SPEED,
                                                   GET_WIND_ANGLE],
                                        'parsed_data': []},
                          'WS2_FRAME': {'id': WIND_SENS2_FRAME_ID,
                                        'uri': [WIND2_SPEED,
                                                WIND2_ANGLE],
                                        'rostopic': [WIND2_SPEED_TOPIC,
                                                     WIND2_ANGLE_TOPIC],
                                        'data': None,
                                        'parser': [GET_WIND_SPEED,
                                                   GET_WIND_ANGLE],
                                        'parsed_data': []},
                          'WS3_FRAME': {'id': WIND_SENS3_FRAME_ID,
                                        'uri': [WIND3_SPEED,
                                                WIND3_ANGLE],
                                        'rostopic': [WIND3_SPEED_TOPIC,
                                                     WIND3_ANGLE_TOPIC],
                                        'data': None,
                                        'parser': [GET_WIND_SPEED,
                                                   GET_WIND_ANGLE],
                                        'parsed_data': []},
                          'GPS_CAN_DATE_FRAME': {'id': GPS_DATE_FRAME_ID,
                                                 'uri': [GPS_CAN_TIME],
                                                 'rostopic': [GPS_CAN_TIME_TOPIC],
                                                 'data': None,
                                                 'parser': [GET_HOUR,
                                                            GET_MINUTE,
                                                            GET_SECOND,
                                                            GET_DAY,
                                                            GET_MONTH,
                                                            GET_YEAR],
                                                 'parsed_data': []},
                          'GPS_CAN_LAT_FRAME': {'id': GPS_LAT_FRAME_ID,
                                                'uri': [GPS_CAN_LAT],
                                                'rostopic': [GPS_CAN_LAT_TOPIC],
                                                'data': None,
                                                'parser': [GET_GPS_LAT],
                                                'parsed_data': []},
                          'GPS_CAN_LON_FRAME': {'id': GPS_LONG_FRAME_ID,
                                                'uri': [GPS_CAN_LON],
                                                'rostopic': [GPS_CAN_LON_TOPIC],
                                                'data': None,
                                                'parser': [GET_GPS_LONG],
                                                'parsed_data': []},
                          'GPS_CAN_OTHER_FRAME': {'id': GPS_OTHER_FRAME_ID,
                                                  'uri': [GPS_CAN_GNDSPEED,
                                                          GPS_CAN_MAGVAR,
                                                          GPS_CAN_TMG,
                                                          GPS_CAN_TRUE_HEADING],
                                                  'rostopic': [GPS_CAN_GNDSPEED_TOPIC,
                                                               GPS_CAN_MAGVAR_TOPIC,
                                                               GPS_CAN_TMG_TOPIC,
                                                               GPS_CAN_TRUE_HEADING_TOPIC],
                                                  'data': None,
                                                  'parser': [GET_GPS_GND_SPEED,
                                                             GET_GPS_MAG_VAR,
                                                             GET_GPS_TMG,
                                                             GET_GPS_TRUE_HEADING],
                                                  'parsed_data': []},
                          # GPS is not connected to the CANBUS so ID is None
                          'GPS_AIS_DATE_FRAME': {'id': None,
                                                 'uri': [GPS_AIS_TIME],
                                                 'rostopic': [GPS_AIS_TIME_TOPIC],
                                                 'data': None,
                                                 'parser': [GET_HOUR,
                                                            GET_MINUTE,
                                                            GET_SECOND,
                                                            GET_DAY,
                                                            GET_MONTH,
                                                            GET_YEAR],
                                                 'parsed_data': []},
                          'GPS_AIS_LAT_FRAME': {'id': None,
                                                'uri': [GPS_AIS_LAT],
                                                'rostopic': [GPS_AIS_LAT_TOPIC],
                                                'data': None,
                                                'parser': [GET_GPS_LAT],
                                                'parsed_data': []},
                          'GPS_AIS_LON_FRAME': {'id': None,
                                                'uri': [GPS_AIS_LON],
                                                'rostopic': [GPS_AIS_LON_TOPIC],
                                                'data': None,
                                                'parser': [GET_GPS_LONG],
                                                'parsed_data': []},
                          'GPS_AIS_OTHER_FRAME': {'id': None,
                                                  'uri': [GPS_AIS_GNDSPEED,
                                                          GPS_AIS_MAGVAR,
                                                          GPS_AIS_TMG,
                                                          GPS_AIS_TRUE_HEADING],
                                                  'rostopic': [GPS_AIS_GNDSPEED_TOPIC,
                                                               GPS_AIS_MAGVAR_TOPIC,
                                                               GPS_AIS_TMG_TOPIC,
                                                               GPS_AIS_TRUE_HEADING_TOPIC],
                                                  'data': None,
                                                  'parser': [GET_GPS_GND_SPEED,
                                                             GET_GPS_MAG_VAR,
                                                             GET_GPS_TMG,
                                                             GET_GPS_TRUE_HEADING],
                                                  'parsed_data': []},
                          'ACCEL_FRAME': {'id': ACCEL_FRAME_ID,
                                          'uri': [ACCELEROMETER_X,
                                                  ACCELEROMETER_Y,
                                                  ACCELEROMETER_Z],
                                          'rostopic': [ACCELEROMETER_X_TOPIC,
                                                       ACCELEROMETER_Y_TOPIC,
                                                       ACCELEROMETER_Z_TOPIC],
                                          'data': None,
                                          'parser': [GET_ACCEL_X_DATA,
                                                     GET_ACCEL_Y_DATA,
                                                     GET_ACCEL_Z_DATA],
                                          'parsed_data': []},
                          'GYRO_FRAME': {'id': GYRO_FRAME_ID,
                                         'uri': [GYROSCOPE_X,
                                                 GYROSCOPE_Y,
                                                 GYROSCOPE_Z],
                                         'rostopic': [GYROSCOPE_X_TOPIC,
                                                      GYROSCOPE_Y_TOPIC,
                                                      GYROSCOPE_Z_TOPIC],
                                         'data': None,
                                         'parser': [GET_GYRO_X_DATA,
                                                    GET_GYRO_Y_DATA,
                                                    GET_GYRO_Z_DATA],
                                         'parsed_data': []},
                          # Not connected to NUC so rostopic is None
                          'SAILENCODER_FRAME': {'id': SAILENCODER_FRAME_ID,
                                                'uri': [SAILENCODER_ANGLE],
                                                'rostopic': None,
                                                'data': None,
                                                'parser': [GET_SAILENCODER_ANGLE],
                                                'parsed_data': []},
                          # bbb_canbus_listener.cpp only considers BMS1
                          # it also only sets current, voltage, maxcell, and mincell
                          'BMS1': {'id': BMS1,
                                             'uri': [BMS1_CURRENT,
                                                     BMS1_VOLTAGE,
                                                     BMS1_MAXCELL,
                                                     BMS1_MINCELL],
                                             'rostopic': None,
                                             'data': None,
                                             'parser': [GET_BMS_CURR_DATA,
                                                        GET_BMS_VOLT_DATA,
                                                        GET_BMS_MAX_VOLT_DATA,
                                                        GET_BMS_MIN_VOLT_DATA],
                                             'parsed_data': []}
}

CAN_bus_sensor = ['WS1_FRAME',
                  'WS2_FRAME',
                  'WS3_FRAME',
                  'GPS_CAN_DATE_FRAME',
                  'GPS_CAN_LAT_FRAME',
                  'GPS_CAN_LON_FRAME',
                  'GPS_CAN_OTHER_FRAME',
                  'ACCEL_FRAME',
                  'GYRO_FRAME',
                  'SAILENCODER_FRAME',
                  'BMS1'
                  ]
