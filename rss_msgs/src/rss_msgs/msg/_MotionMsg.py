"""autogenerated by genmsg_py from MotionMsg.msg. Do not edit."""
import roslib.message
import struct


class MotionMsg(roslib.message.Message):
  _md5sum = "dcd1efab7b0193f9e7f726b14abbf015"
  _type = "rss_msgs/MotionMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 translationalVelocity
float64 rotationalVelocity
"""
  __slots__ = ['translationalVelocity','rotationalVelocity']
  _slot_types = ['float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       translationalVelocity,rotationalVelocity
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(MotionMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.translationalVelocity is None:
        self.translationalVelocity = 0.
      if self.rotationalVelocity is None:
        self.rotationalVelocity = 0.
    else:
      self.translationalVelocity = 0.
      self.rotationalVelocity = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2d.pack(_x.translationalVelocity, _x.rotationalVelocity))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.translationalVelocity, _x.rotationalVelocity,) = _struct_2d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2d.pack(_x.translationalVelocity, _x.rotationalVelocity))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.translationalVelocity, _x.rotationalVelocity,) = _struct_2d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2d = struct.Struct("<2d")
