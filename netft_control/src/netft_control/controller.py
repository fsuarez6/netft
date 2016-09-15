#!/usr/bin/env python
import rospy
import collections
import numpy as np
from criros.conversions import from_wrench
from criros.utils import read_parameter, solve_namespace
# Messages
from denso_msgs.msg import EndpointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import WrenchStamped


class FTSensor(object):
  """
  Interface class to connect to the FT sensor controller. 
  It subscribes to the C{ft_sensor/raw} and C{endpoint_state} topics to retrieve the raw and compensated force-torque values.
  This class checks for errors in the FT sensor and returns the raw and compensated values along with the sensor status and readings timestamp.
  """
  queue_len = 10
  def __init__(self, namespace='', read_compensated=True, timeout=3.0):
    """
    FTSensor constructor. It subscribes to the following topics:
      - C{ft_sensor/diagnostics},
      - C{ft_sensor/raw}, and
      - C{endpoint_state}.
    It reports the raw and compensated (if available) FT sensor values.
    @type namespace: string
    @param namespace: Override ROS namespace manually. Useful when accessing multiple FT sensors from the same node.
    @type timeout: double
    @param timeout: Time to wait for each of the FT sensor's topics to start publishing.
    """
    ns = solve_namespace(namespace)
    # Set-up subscribers
    self.rate = read_parameter('%s/ft_sensor/ft_sensor_controller/publish_rate' % ns, 250.0)
    self.raw_queue = collections.deque(maxlen=self.queue_len)
    rospy.Subscriber('%sft_sensor/diagnostics' % ns, DiagnosticArray, self.cb_diagnostics)
    rospy.Subscriber('%sft_sensor/raw' % ns, WrenchStamped, self.cb_raw)
    rospy.Subscriber('%sendpoint_state' % ns, EndpointState, self.cb_compensated)
    initime = rospy.get_time()
    while not rospy.is_shutdown() and not self.is_raw_alive():
      rospy.sleep(0.1)
      if (rospy.get_time() - initime) > timeout:
        rospy.logwarn('FTSensor: Cannot read raw wrench')
        return
    if read_compensated:
      initime = rospy.get_time()
      while not rospy.is_shutdown() and not self.is_compensated_alive():
        rospy.sleep(0.1)
        if (rospy.get_time() - initime) > timeout:
          rospy.logwarn('FTSensor: Cannot read compensated wrench')
          return
    rospy.loginfo('FTSensor successfully initialized')
  
  def cb_compensated(self, msg):
    """
    Callback executed every time a message is publish in the C{endpoint_state} topic.
    @type  msg: denso_msgs/EndpointState
    @param msg: The EndpointState message published by the compensation node.
    """
    self.compensated_msg = msg
  
  def cb_diagnostics(self, msg):
    """
    Callback executed every time a message is publish in the C{ft_sensor/diagnostics} topic.
    @type  msg: diagnostic_msgs/DiagnosticArray
    @param msg: The DiagnosticArray message published by RT hardware interface.
    """
    self.diagnostics_msg = msg
  
  def cb_raw(self, msg):
    """
    Callback executed every time a message is publish in the C{ft_sensor/raw} topic.
    @type  msg: geometry_msgs/WrenchStamped
    @param msg: The WrenchStamped message published by RT hardware interface.
    """
    self.raw_msg = msg
    self.raw_queue.append(msg)
    
  
  def is_compensated_alive(self):
    """
    Returns 
    @rtype: bool
    @return: B{True} if the C{endpoint_state} topic has published something.
    """
    return (self.is_raw_alive() and hasattr(self, 'compensated_msg'))
    
  def is_raw_alive(self):
    diagnostics = hasattr(self, 'diagnostics_msg')
    raw = hasattr(self, 'raw_msg')
    return (diagnostics and raw)
  
  def get_compensated(self):
    status = DiagnosticStatus().ERROR
    value = None
    timestamp = None
    if self.is_compensated_alive():
      status = self._get_status()
      timestamp = self.compensated_msg.header.stamp.to_sec()
      value = from_wrench(self.compensated_msg.wrench)
    return status, timestamp, value

  def get_compensated_wrench(self):
    value = None
    if self.is_compensated_alive():
      value = from_wrench(self.compensated_msg.wrench)
    return value
  
  def get_raw(self):
    status = DiagnosticStatus().ERROR
    value = None
    timestamp = None
    if self.is_raw_alive():
      status = self._get_status()
      timestamp = self.raw_msg.header.stamp.to_sec()
      value = from_wrench(self.raw_msg.wrench)
    return status, timestamp, value

  def get_raw_wrench(self):
    value = None
    if self.is_raw_alive():
      value = from_wrench(self.raw_msg.wrench)
    return value
  
  def _get_status(self):
    """
    Omits some errors that aren't really errors. The overload error is treated as a warning
    """
    status = self.diagnostics_msg.status[0].level
    if status == DiagnosticStatus().ERROR and self._receiving():
      # Check that the FT sensor reports new values
      w1 = from_wrench(self.raw_queue[0].wrench)
      w2 = from_wrench(self.raw_queue[self.queue_len-1].wrench)
      status = DiagnosticStatus().ERROR if np.allclose(w1, w2) else DiagnosticStatus().OK
    return status
  
  def _receiving(self):
    new_msg = False
    if len(self.raw_queue) == self.queue_len:
      laststamp = self.raw_queue[-1].header.stamp.to_sec()
      now = rospy.get_time()
      new_msg = (now-laststamp) < (self.queue_len/float(self.rate))
    return new_msg
