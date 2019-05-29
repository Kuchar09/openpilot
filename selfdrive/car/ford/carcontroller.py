from common.numpy_fast import clip, interp
from cereal import car
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.ford.fordcan import create_steer_command, create_lkas_ui, \
                                       spam_cancel_button
from selfdrive.can.packer import CANPacker


MAX_STEER_DELTA = 1
TOGGLE_DEBUG = False

class CarController(object):
  def __init__(self, car_fingerprint):
    self.enabled_last = False
    self.main_on_last = False
    self.car_fingerprint = car_fingerprint

  def update(self, sendcan, enabled, CS, frame, actuators):

    can_sends = []
    steer_alert = False

    apply_steer = actuators.steer

    if self.enable_camera:

      if (frame % 3) == 0:

        curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*3.1415/180., CS.v_ego)

        self.lkas_action = 5   # 4 and 5 seem the best. 8 and 9 seem to aggressive and laggy

        can_sends.append(create_steer_command(self.packer, apply_steer, enabled,
                                              CS.lkas_state, CS.angle_steers, curvature, self.lkas_action))

      if (frame % 100) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.main_on) or \
         (self.steer_alert_last != steer_alert):
        can_sends.append(create_lkas_ui(self.packer, CS.main_on, enabled, steer_alert))

      self.enabled_last = enabled
      self.main_on_last = CS.main_on
      self.steer_alert_last = steer_alert

      sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
