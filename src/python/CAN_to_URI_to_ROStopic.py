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


CAN_to_URI_to_ROStopic = {'WS1_FRAME': {'id': WIND_SENS1_FRAME_ID,
                                        'uri': [WIND1_SPEED, WIND1_ANGLE],
                                        'rostopic': [WIND1_SPEED_TOPIC, WIND1_ANGLE_TOPIC],
                                        'data': None,
                                        'parser': [GET_WIND_SPEED, GET_WIND_ANGLE],
                                        'parsed_data': []},
                          'WS2_FRAME': {'id': WIND_SENS2_FRAME_ID,
                                        'uri': [WIND2_SPEED, WIND2_ANGLE],
                                        'rostopic': [WIND2_SPEED_TOPIC, WIND2_ANGLE_TOPIC],
                                        'data': None,
                                        'parser': [GET_WIND_SPEED, GET_WIND_ANGLE],
                                        'parsed_data': []},
                          'WS3_FRAME': {'id': WIND_SENS3_FRAME_ID,
                                        'uri': [WIND3_SPEED, WIND3_ANGLE],
                                        'rostopic': [WIND3_SPEED_TOPIC, WIND3_ANGLE_TOPIC],
                                        'data': None,
                                        'parser': [GET_WIND_SPEED, GET_WIND_ANGLE],
                                        'parsed_data': []},

                          # The rest is commented out since it would involve
                          # changing the rest of frame_parser.py and we can
                          # separate that into a separate task

                          # 'GPS_CAN_TIME': {'id': GPS_DATE_FRAME_ID,
                          #                  'uri': GPS_CAN_TIME,
                          #                  'rostopic' : GPS_CAN_TIME_TOPIC,
                          #                  'data': None},
                          # 'GPS_CAN_LAT': {'id': GPS_LAT_FRAME_ID,
                          #                 'uri': GPS_CAN_LAT,
                          #                 'rostopic': GPS_CAN_LAT_TOPIC,
                          #                 'data': None},
                          # 'GPS_CAN_LON': {'id': GPS_LONG_FRAME_ID,
                          #                 'uri': GPS_CAN_LON,
                          #                 'rostopic': GPS_CAN_LON_TOPIC,
                          #                 'data': None},
                          # 'GPS_CAN_GND_SPEED': {'id': GPS_OTHER_FRAME_ID,
                          #                       'uri': GPS_CAN_GNDSPEED,
                          #                       'rostopic': GPS_CAN_GNDSPEED_TOPIC, # NOQA
                          #                       'data': None},
                          # 'GPS_CAN_TMG' : {'id': GPS_OTHER_FRAME_ID,
                          #                  'uri': GPS_CAN_TMG,
                          #                  'rostopic': GPS_CAN_TMG_TOPIC,
                          #                  'data': None},
                          # 'GPS_CAN_TRUE_HEADING' : {'id': GPS_OTHER_FRAME_ID,
                          #                           'uri': GPS_CAN_TRUE_HEADING, # NOQA
                          #                           'rostopic': GPS_CAN_TRUE_HEADING_TOPIC, # NOQA
                          #                           'data': None},
                          # 'GPS_CAN_MAGVAR' : {'id': GPS_OTHER_FRAME_ID,
                          #                     'uri': GPS_CAN_MAGVAR,
                          #                     'rostopic': GPS_CAN_MAGVAR_TOPIC, # NOQA
                          #                     'data': None}, # NOQA
                          # 'GPS_AIS_TIME' : {'id': None,
                          #                   'uri': GPS_AIS_TIME,
                          #                   'rostopic': GPS_AIS_TIME_TOPIC,
                          #                   'data': None}, # NOQA
                          # 'GPS_AIS_LAT' : {'id': None,
                          #                  'uri': GPS_AIS_LAT,
                          #                  'rostopic': GPS_AIS_LAT_TOPIC,
                          #                  'data': None}, # NOQA
                          # 'GPS_AIS_LON' : {'id': None,
                          #                  'uri': GPS_AIS_LON,
                          #                  'rostopic': GPS_AIS_LON_TOPIC,
                          #                  'data': None}, # NOQA
                          # 'GPS_AIS_GNDSPEED' : {'id': None,
                          #                       'uri': GPS_AIS_GNDSPEED,
                          #                       'rostopic': GPS_AIS_GNDSPEED_TOPIC, # NOQA
                          #                       'data': None}, # NOQA
                          # 'GPS_AIS_TMG' : {'id': None,
                          #                  'uri': GPS_AIS_TMG,
                          #                  'rostopic': GPS_AIS_TMG_TOPIC,
                          #                  'data': None}, # NOQA
                          # 'GPS_AIS_TRUE_HEADING' : {'id': None,
                          #                           'uri': GPS_AIS_TRUE_HEADING, # NOQA
                          #                           'rostopic': GPS_AIS_TRUE_HEADING_TOPIC, # NOQA
                          #                           'data': None}, # NOQA
                          # 'GPS_AIS_MAGVAR' : {'id': None,
                          #                     'uri': GPS_AIS_MAGVAR,
                          #                     'rostopic': GPS_AIS_MAGVAR_TOPIC, # NOQA
                          #                     'data': None}, # NOQA
                          # 'ACCELEROMETER_X' : {'id': ACCEL_FRAME_ID,
                          #                      'uri': ACCELEROMETER_X,
                          #                      'rostopic': ACCELEROMETER_X_TOPIC, # NOQA
                          #                      'data': None}, # NOQA
                          # 'ACCELEROMETER_Y' : {'id': ACCEL_FRAME_ID,
                          #                      'uri': ACCELEROMETER_Y,
                          #                      'rostopic': ACCELEROMETER_Y_TOPIC, # NOQA
                          #                      'data': None}, # NOQA
                          # 'ACCELEROMETER_Z' : {'id': ACCEL_FRAME_ID,
                          #                      'uri': ACCELEROMETER_Z,
                          #                      'rostopic': ACCELEROMETER_Z_TOPIC, # NOQA
                          #                      'data': None} # NOQA
                          }

CAN_bus_sensor = ['WS1_FRAME',
                  'WS1_ANGLE',
                  'WS2_FRAME',
                  'WS2_ANGLE',
                  'WS3_FRAME',
                  'WS3_ANGLE',
                  # 'GPS_CAN_TIME',
                  'GPS_CAN_LAT',
                  'GPS_CAN_LON',
                  'GPS_CAN_GND_SPEED',
                  'GPS_CAN_TMG',
                  # 'GPS_CAN_TRUE_HEADING',
                  'GPS_CAN_MAGVAR',
                  'ACCX',
                  'ACCY',
                  'ACCZ']
