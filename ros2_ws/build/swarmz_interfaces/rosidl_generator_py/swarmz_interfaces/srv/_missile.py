# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swarmz_interfaces:srv/Missile.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Missile_Request(type):
    """Metaclass of message 'Missile_Request'."""

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
            module = import_type_support('swarmz_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swarmz_interfaces.srv.Missile_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__missile__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__missile__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__missile__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__missile__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__missile__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Missile_Request(metaclass=Metaclass_Missile_Request):
    """Message class 'Missile_Request'."""

    __slots__ = [
        '_robot_name',
    ]

    _fields_and_field_types = {
        'robot_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_name = kwargs.get('robot_name', str())

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
        if self.robot_name != other.robot_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_name(self):
        """Message field 'robot_name'."""
        return self._robot_name

    @robot_name.setter
    def robot_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_name' field must be of type 'str'"
        self._robot_name = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Missile_Response(type):
    """Metaclass of message 'Missile_Response'."""

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
            module = import_type_support('swarmz_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swarmz_interfaces.srv.Missile_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__missile__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__missile__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__missile__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__missile__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__missile__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Missile_Response(metaclass=Metaclass_Missile_Response):
    """Message class 'Missile_Response'."""

    __slots__ = [
        '_has_fired',
        '_ammo',
    ]

    _fields_and_field_types = {
        'has_fired': 'boolean',
        'ammo': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.has_fired = kwargs.get('has_fired', bool())
        self.ammo = kwargs.get('ammo', int())

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
        if self.has_fired != other.has_fired:
            return False
        if self.ammo != other.ammo:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def has_fired(self):
        """Message field 'has_fired'."""
        return self._has_fired

    @has_fired.setter
    def has_fired(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_fired' field must be of type 'bool'"
        self._has_fired = value

    @builtins.property
    def ammo(self):
        """Message field 'ammo'."""
        return self._ammo

    @ammo.setter
    def ammo(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ammo' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'ammo' field must be an integer in [-128, 127]"
        self._ammo = value


class Metaclass_Missile(type):
    """Metaclass of service 'Missile'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('swarmz_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swarmz_interfaces.srv.Missile')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__missile

            from swarmz_interfaces.srv import _missile
            if _missile.Metaclass_Missile_Request._TYPE_SUPPORT is None:
                _missile.Metaclass_Missile_Request.__import_type_support__()
            if _missile.Metaclass_Missile_Response._TYPE_SUPPORT is None:
                _missile.Metaclass_Missile_Response.__import_type_support__()


class Missile(metaclass=Metaclass_Missile):
    from swarmz_interfaces.srv._missile import Missile_Request as Request
    from swarmz_interfaces.srv._missile import Missile_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
