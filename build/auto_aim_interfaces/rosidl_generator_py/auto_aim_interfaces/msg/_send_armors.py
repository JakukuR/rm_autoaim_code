# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/SendArmors.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SendArmors(type):
    """Metaclass of message 'SendArmors'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('auto_aim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim_interfaces.msg.SendArmors')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__send_armors
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__send_armors
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__send_armors
            cls._TYPE_SUPPORT = module.type_support_msg__msg__send_armors
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__send_armors

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SendArmors(metaclass=Metaclass_SendArmors):
    """Message class 'SendArmors'."""

    __slots__ = [
        '_yaw_angle',
        '_pitch_angle',
        '_distance',
        '_shoot_flag',
        '_detect_flag',
    ]

    _fields_and_field_types = {
        'yaw_angle': 'double',
        'pitch_angle': 'double',
        'distance': 'double',
        'shoot_flag': 'int32',
        'detect_flag': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.yaw_angle = kwargs.get('yaw_angle', float())
        self.pitch_angle = kwargs.get('pitch_angle', float())
        self.distance = kwargs.get('distance', float())
        self.shoot_flag = kwargs.get('shoot_flag', int())
        self.detect_flag = kwargs.get('detect_flag', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.yaw_angle != other.yaw_angle:
            return False
        if self.pitch_angle != other.pitch_angle:
            return False
        if self.distance != other.distance:
            return False
        if self.shoot_flag != other.shoot_flag:
            return False
        if self.detect_flag != other.detect_flag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def yaw_angle(self):
        """Message field 'yaw_angle'."""
        return self._yaw_angle

    @yaw_angle.setter
    def yaw_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_angle = value

    @builtins.property
    def pitch_angle(self):
        """Message field 'pitch_angle'."""
        return self._pitch_angle

    @pitch_angle.setter
    def pitch_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pitch_angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_angle = value

    @builtins.property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance = value

    @builtins.property
    def shoot_flag(self):
        """Message field 'shoot_flag'."""
        return self._shoot_flag

    @shoot_flag.setter
    def shoot_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'shoot_flag' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'shoot_flag' field must be an integer in [-2147483648, 2147483647]"
        self._shoot_flag = value

    @builtins.property
    def detect_flag(self):
        """Message field 'detect_flag'."""
        return self._detect_flag

    @detect_flag.setter
    def detect_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'detect_flag' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'detect_flag' field must be an integer in [-2147483648, 2147483647]"
        self._detect_flag = value
