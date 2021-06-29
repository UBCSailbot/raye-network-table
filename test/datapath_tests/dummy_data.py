import random
import copy


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
        dummy_tests[key] = copy.deepcopy(filtered_dict[sensor])
        dummy_tests[key]['data'] = value
        for i in range(len(dummy_tests[key]['parser'])):
            dummy_tests[key]['parsed_data'].append(dummy_tests[key]['parser'][i](int(value, 16)))
        format_data(sensor, dummy_tests[key]['parsed_data'], key)

    return dummy_tests


def format_data(tag, parsed_data, key):
    '''
    Takes in a list of parsed data and returns the form of parsed data
    for each sensor on the can bus
    '''
    formatted_data = 0
    if 'DATE_FRAME' in tag:
        formatted_data = str(parsed_data[-1]) + "/" + str(parsed_data[-2]) + "/" + \
            str(parsed_data[-3]) + "-" + str(parsed_data[0]) + ":" + str(parsed_data[1]) + ":" + str(parsed_data[2])
        parsed_data.clear()
        parsed_data.append('\"{}\"'.format(formatted_data))
        return


'''Test_Data is used as dummy data to be used for datapath tests'''
# Some tests are commented out since it would take a long time
# to run the datapath test
Test_Data = {'Test 1': '00000000DEADBEEF',
             'Test 2': '10101010DEADBEEF',
             'Test 3': '5555555666666666',
             # 'Test 4': '0010219475431897',
             # 'Test 5': '0058943756934628'
             # 'Test 6': '00AABBCCDDFF1133'
             }
