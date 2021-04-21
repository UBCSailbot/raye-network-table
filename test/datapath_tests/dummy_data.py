import random


def make_dummy_tests(can_filter, dummy_data, CAN_URI_ROS):
    '''
    This function makes dummy tests with the dummy data found in
    dummy_data.py. It chooses those data values and attributes them
    to random sensors that are hooked on the CAN bus.

    This makes use of the CAN_to_URI_to_ROS.py file's gigantic
    dictionary (which is used in the CAN_URI_ROS parameter) so make
    sure to check that out in order to better understand this.
    '''
    dummy_tests = {}
    filtered_dict = {}

    for key, value in CAN_URI_ROS.items():
        if key in can_filter:
            filtered_dict[key] = CAN_URI_ROS[key]

    for key, value in dummy_data.items():
        sensor = random.choice(list(filtered_dict))
        dummy_tests[key] = filtered_dict[sensor]
        dummy_tests[key]['data'] = value
        dummy_tests[key]['parsed_data'] = \
            dummy_tests[key]['parser'](int(value, 16))
    return dummy_tests


'''Test_Data is used as dummy data to be used for datapath tests'''
# Some tests are commented out since it would take a long time
# to run the datapath test
Test_Data = {'Test 1': '00000000DEADBEEF',
             # 'Test 2': '10101010DEADBEEF',
             # 'Test 3': '5555555666666666',
             # 'Test 5': '0010219475431897',
             # 'Test 6': '0058943756934628'
             'Test 7': '00AABBCCDDFF1133'
             }
