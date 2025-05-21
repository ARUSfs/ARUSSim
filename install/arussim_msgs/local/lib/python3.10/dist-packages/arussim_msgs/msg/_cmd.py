# generated from rosidl_generator_py/resource/_idl.py.em
# with input from arussim_msgs:msg/Cmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Cmd(type):
    """Metaclass of message 'Cmd'."""

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
                'arussim_msgs.msg.Cmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cmd

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


class Cmd(metaclass=Metaclass_Cmd):
    """Message class 'Cmd'."""

    __slots__ = [
        '_header',
        '_acc',
        '_delta',
        '_target_r',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'acc': 'float',
        'delta': 'float',
        'target_r': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.acc = kwargs.get('acc', float())
        self.delta = kwargs.get('delta', float())
        self.target_r = kwargs.get('target_r', float())

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
        if self.acc != other.acc:
            return False
        if self.delta != other.delta:
            return False
        if self.target_r != other.target_r:
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
    def acc(self):
        """Message field 'acc'."""
        return self._acc

    @acc.setter
    def acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'acc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._acc = value

    @builtins.property
    def delta(self):
        """Message field 'delta'."""
        return self._delta

    @delta.setter
    def delta(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'delta' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'delta' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._delta = value

    @builtins.property
    def target_r(self):
        """Message field 'target_r'."""
        return self._target_r

    @target_r.setter
    def target_r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_r' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_r' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_r = value
