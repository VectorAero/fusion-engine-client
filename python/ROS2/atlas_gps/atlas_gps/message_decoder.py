from atlas_gps.fusion_engine_client.messages.core import *

class Message_Decoder() :

    def __init__(self):
        super().__init__()
        self.ros_msg = None
 
    def decode_message (self, header, data, offset):
        # Validate the message length and CRC.
        if len(data) != header.calcsize() + header.payload_size_bytes:
            return False
        else:
            header.validate_crc(data)

        '''
        # Check that the sequence number increments as expected.
        if header.sequence_number != decode_message.expected_sequence_number:
            print('Warning: unexpected sequence number. [expected=%d, received=%d]' %
                (decode_message.expected_sequence_number, header.sequence_number))
        '''

        # FIX ME
        # decode_message.expected_sequence_number = header.sequence_number + 1
        # Deserialize and print the message contents.
    #
    # Note: This could also be done more generally using the fusion_engine_client.core.message_type_to_class dictionary.
    # We do it explicitly here for sake of example.

        if header.message_type == ROS_PoseMessage.MESSAGE_TYPE:
            contents = ROS_PoseMessage()
            self.ros_msg = contents.unpack_to_msg(buffer=data, offset=offset)
            return self.ros_msg
        elif header.message_type == ROS_GPSFixMessage.MESSAGE_TYPE:
            contents = ROS_GPSFixMessage()
            contents.unpack(buffer=data, offset=offset)
        elif header.message_type == ROS_IMUMessage.MESSAGE_TYPE:
            contents = ROS_IMUMessage()
            contents.unpack(buffer=data, offset=offset)
        else:
            print('Decoded %s message [sequence=%d, size=%d B]' %
                (header.get_type_string(), header.sequence_number, len(data)))
            contents = None

        if contents is not None:
            parts = str(contents).split('\n')
            parts[0] += ' [sequence=%d, size=%d B]' % (header.sequence_number, len(data))
            print('\n'.join(parts))

        return None