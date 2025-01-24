// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from swarmz_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "swarmz_interfaces/msg/detail/detection__struct.h"
#include "swarmz_interfaces/msg/detail/detection__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool swarmz_interfaces__msg__detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("swarmz_interfaces.msg._detection.Detection", full_classname_dest, 42) == 0);
  }
  swarmz_interfaces__msg__Detection * ros_message = _ros_message;
  {  // vehicle_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vehicle_type = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // is_friend
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_friend");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_friend = (Py_True == field);
    Py_DECREF(field);
  }
  {  // relative_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "relative_position");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->relative_position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * swarmz_interfaces__msg__detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Detection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("swarmz_interfaces.msg._detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Detection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  swarmz_interfaces__msg__Detection * ros_message = (swarmz_interfaces__msg__Detection *)raw_ros_message;
  {  // vehicle_type
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->vehicle_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_friend
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_friend ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_friend", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // relative_position
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->relative_position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "relative_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
