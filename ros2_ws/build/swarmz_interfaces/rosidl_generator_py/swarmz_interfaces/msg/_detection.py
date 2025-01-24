# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swarmz_interfaces:msg/Detection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Detection(type):
    """Metaclass of message 'Detection'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'DRONE': 0,
        'SHIP': 1,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('swarmz_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swarmz_interfaces.msg.Detection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DRONE': cls.__constants['DRONE'],
            'SHIP': cls.__constants['SHIP'],
        }

    @property
    def DRONE(self):
        """Message constant 'DRONE'."""
        return Metaclass_Detection.__constants['DRONE']

    @property
    def SHIP(self):
        """Message constant 'SHIP'."""
        return Metaclass_Detection.__constants['SHIP']


class Detection(metaclass=Metaclass_Detection):
    """
    Message class 'Detection'.

    Constants:
      DRONE
      SHIP
    """

    __slots__ = [
        '_vehicle_type',
        '_is_friend',
        '_relative_position',
    ]

    _fields_and_field_types = {
        'vehicle_type': 'int8',
        'is_friend': 'boolean',
        'relative_position': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.vehicle_type = kwargs.get('vehicle_type', int())
        self.is_friend = kwargs.get('is_friend', bool())
        from geometry_msgs.msg import Pose
        self.relative_position = kwargs.get('relative_position', Pose())

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
        if self.vehicle_type != other.vehicle_type:
            return False
        if self.is_friend != other.is_friend:
            return False
        if self.relative_position != other.relative_position:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vehicle_type(self):
        """Message field 'vehicle_type'."""
        return self._vehicle_type

    @vehicle_type.setter
    def vehicle_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vehicle_type' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'vehicle_type' field must be an integer in [-128, 127]"
        self._vehicle_type = value

    @builtins.property
    def is_friend(self):
        """Message field 'is_friend'."""
        return self._is_friend

    @is_friend.setter
    def is_friend(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_friend' field must be of type 'bool'"
        self._is_friend = value

    @builtins.property
    def relative_position(self):
        """Message field 'relative_position'."""
        return self._relative_position

    @relative_position.setter
    def relative_position(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'relative_position' field must be a sub message of type 'Pose'"
        self._relative_position = value
