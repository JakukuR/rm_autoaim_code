// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from auto_aim_interfaces:msg/SendArmors.idl
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
#include "auto_aim_interfaces/msg/detail/send_armors__struct.h"
#include "auto_aim_interfaces/msg/detail/send_armors__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool auto_aim_interfaces__msg__send_armors__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("auto_aim_interfaces.msg._send_armors.SendArmors", full_classname_dest, 47) == 0);
  }
  auto_aim_interfaces__msg__SendArmors * ros_message = _ros_message;
  {  // yaw_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_angle = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_angle = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shoot_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "shoot_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->shoot_flag = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // detect_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "detect_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->detect_flag = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * auto_aim_interfaces__msg__send_armors__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SendArmors */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("auto_aim_interfaces.msg._send_armors");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SendArmors");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  auto_aim_interfaces__msg__SendArmors * ros_message = (auto_aim_interfaces__msg__SendArmors *)raw_ros_message;
  {  // yaw_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shoot_flag
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->shoot_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shoot_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // detect_flag
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->detect_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "detect_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
