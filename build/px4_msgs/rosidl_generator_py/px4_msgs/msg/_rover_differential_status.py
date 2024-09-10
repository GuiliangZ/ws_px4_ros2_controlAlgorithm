# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/RoverDifferentialStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RoverDifferentialStatus(type):
    """Metaclass of message 'RoverDifferentialStatus'."""

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
            module = import_type_support('px4_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'px4_msgs.msg.RoverDifferentialStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rover_differential_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rover_differential_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rover_differential_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rover_differential_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rover_differential_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RoverDifferentialStatus(metaclass=Metaclass_RoverDifferentialStatus):
    """Message class 'RoverDifferentialStatus'."""

    __slots__ = [
        '_timestamp',
        '_actual_speed',
        '_actual_yaw_deg',
        '_actual_yaw_rate_deg_s',
        '_desired_yaw_rate_deg_s',
        '_pid_yaw_integral',
        '_pid_yaw_rate_integral',
        '_pid_throttle_integral',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'actual_speed': 'float',
        'actual_yaw_deg': 'float',
        'actual_yaw_rate_deg_s': 'float',
        'desired_yaw_rate_deg_s': 'float',
        'pid_yaw_integral': 'float',
        'pid_yaw_rate_integral': 'float',
        'pid_throttle_integral': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.actual_speed = kwargs.get('actual_speed', float())
        self.actual_yaw_deg = kwargs.get('actual_yaw_deg', float())
        self.actual_yaw_rate_deg_s = kwargs.get('actual_yaw_rate_deg_s', float())
        self.desired_yaw_rate_deg_s = kwargs.get('desired_yaw_rate_deg_s', float())
        self.pid_yaw_integral = kwargs.get('pid_yaw_integral', float())
        self.pid_yaw_rate_integral = kwargs.get('pid_yaw_rate_integral', float())
        self.pid_throttle_integral = kwargs.get('pid_throttle_integral', float())

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
        if self.timestamp != other.timestamp:
            return False
        if self.actual_speed != other.actual_speed:
            return False
        if self.actual_yaw_deg != other.actual_yaw_deg:
            return False
        if self.actual_yaw_rate_deg_s != other.actual_yaw_rate_deg_s:
            return False
        if self.desired_yaw_rate_deg_s != other.desired_yaw_rate_deg_s:
            return False
        if self.pid_yaw_integral != other.pid_yaw_integral:
            return False
        if self.pid_yaw_rate_integral != other.pid_yaw_rate_integral:
            return False
        if self.pid_throttle_integral != other.pid_throttle_integral:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp = value

    @builtins.property
    def actual_speed(self):
        """Message field 'actual_speed'."""
        return self._actual_speed

    @actual_speed.setter
    def actual_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'actual_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'actual_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._actual_speed = value

    @builtins.property
    def actual_yaw_deg(self):
        """Message field 'actual_yaw_deg'."""
        return self._actual_yaw_deg

    @actual_yaw_deg.setter
    def actual_yaw_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'actual_yaw_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'actual_yaw_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._actual_yaw_deg = value

    @builtins.property
    def actual_yaw_rate_deg_s(self):
        """Message field 'actual_yaw_rate_deg_s'."""
        return self._actual_yaw_rate_deg_s

    @actual_yaw_rate_deg_s.setter
    def actual_yaw_rate_deg_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'actual_yaw_rate_deg_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'actual_yaw_rate_deg_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._actual_yaw_rate_deg_s = value

    @builtins.property
    def desired_yaw_rate_deg_s(self):
        """Message field 'desired_yaw_rate_deg_s'."""
        return self._desired_yaw_rate_deg_s

    @desired_yaw_rate_deg_s.setter
    def desired_yaw_rate_deg_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'desired_yaw_rate_deg_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'desired_yaw_rate_deg_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._desired_yaw_rate_deg_s = value

    @builtins.property
    def pid_yaw_integral(self):
        """Message field 'pid_yaw_integral'."""
        return self._pid_yaw_integral

    @pid_yaw_integral.setter
    def pid_yaw_integral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_yaw_integral' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pid_yaw_integral' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pid_yaw_integral = value

    @builtins.property
    def pid_yaw_rate_integral(self):
        """Message field 'pid_yaw_rate_integral'."""
        return self._pid_yaw_rate_integral

    @pid_yaw_rate_integral.setter
    def pid_yaw_rate_integral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_yaw_rate_integral' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pid_yaw_rate_integral' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pid_yaw_rate_integral = value

    @builtins.property
    def pid_throttle_integral(self):
        """Message field 'pid_throttle_integral'."""
        return self._pid_throttle_integral

    @pid_throttle_integral.setter
    def pid_throttle_integral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_throttle_integral' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pid_throttle_integral' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pid_throttle_integral = value
