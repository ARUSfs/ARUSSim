# generated from rosidl_generator_py/resource/_idl.py.em
# with input from arussim_msgs:msg/Trajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 's'
# Member 'k'
# Member 'speed_profile'
# Member 'acc_profile'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Trajectory(type):
    """Metaclass of message 'Trajectory'."""

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
            module = import_type_support('arussim_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'arussim_msgs.msg.Trajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trajectory
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trajectory
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trajectory
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trajectory
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trajectory

            from arussim_msgs.msg import PointXY
            if PointXY.__class__._TYPE_SUPPORT is None:
                PointXY.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Trajectory(metaclass=Metaclass_Trajectory):
    """Message class 'Trajectory'."""

    __slots__ = [
        '_header',
        '_points',
        '_s',
        '_k',
        '_speed_profile',
        '_acc_profile',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'points': 'sequence<arussim_msgs/PointXY>',
        's': 'sequence<float>',
        'k': 'sequence<float>',
        'speed_profile': 'sequence<float>',
        'acc_profile': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['arussim_msgs', 'msg'], 'PointXY')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.points = kwargs.get('points', [])
        self.s = array.array('f', kwargs.get('s', []))
        self.k = array.array('f', kwargs.get('k', []))
        self.speed_profile = array.array('f', kwargs.get('speed_profile', []))
        self.acc_profile = array.array('f', kwargs.get('acc_profile', []))

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
        if self.header != other.header:
            return False
        if self.points != other.points:
            return False
        if self.s != other.s:
            return False
        if self.k != other.k:
            return False
        if self.speed_profile != other.speed_profile:
            return False
        if self.acc_profile != other.acc_profile:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if __debug__:
            from arussim_msgs.msg import PointXY
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, PointXY) for v in value) and
                 True), \
                "The 'points' field must be a set or sequence and each value of type 'PointXY'"
        self._points = value

    @builtins.property
    def s(self):
        """Message field 's'."""
        return self._s

    @s.setter
    def s(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 's' array.array() must have the type code of 'f'"
            self._s = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 's' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._s = array.array('f', value)

    @builtins.property
    def k(self):
        """Message field 'k'."""
        return self._k

    @k.setter
    def k(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'k' array.array() must have the type code of 'f'"
            self._k = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'k' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._k = array.array('f', value)

    @builtins.property
    def speed_profile(self):
        """Message field 'speed_profile'."""
        return self._speed_profile

    @speed_profile.setter
    def speed_profile(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'speed_profile' array.array() must have the type code of 'f'"
            self._speed_profile = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'speed_profile' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._speed_profile = array.array('f', value)

    @builtins.property
    def acc_profile(self):
        """Message field 'acc_profile'."""
        return self._acc_profile

    @acc_profile.setter
    def acc_profile(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'acc_profile' array.array() must have the type code of 'f'"
            self._acc_profile = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'acc_profile' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._acc_profile = array.array('f', value)
