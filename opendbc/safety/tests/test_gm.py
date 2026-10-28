#!/usr/bin/env python3
import unittest

from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class Buttons:
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  CANCEL = 6


class GmLongitudinalBase(common.CarSafetyTest, common.LongitudinalGasBrakeSafetyTest):

  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB), 2: (0x184,)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd, PSCMStatus

  MAX_POSSIBLE_BRAKE = 2 ** 12
  MAX_BRAKE = 400

  MAX_POSSIBLE_GAS = 4000  # reasonably excessive limits, not signal max
  MIN_POSSIBLE_GAS = -4000

  PCM_CRUISE = False  # openpilot can control the PCM state if longitudinal

  def _send_brake_msg(self, brake):
    values = {"FrictionBrakeCmd": -brake}
    return self.packer_chassis.make_can_msg_safety("EBCMFrictionBrakeCmd", self.BRAKE_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"GasRegenCmd": gas}
    return self.packer.make_can_msg_safety("ASCMGasRegenCmd", 0, values)

  # override these tests from CarSafetyTest, GM longitudinal uses button enable
  def _pcm_status_msg(self, enable):
    raise NotImplementedError

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_resume_buttons(self):
    """
      SET and RESUME enter controls allowed on their falling and rising edges, respectively.
    """
    for btn_prev in range(8):
      for btn_cur in range(8):
        with self.subTest(btn_prev=btn_prev, btn_cur=btn_cur):
          self._rx(self._button_msg(btn_prev))
          self.safety.set_controls_allowed(0)
          for _ in range(10):
            self._rx(self._button_msg(btn_cur))

          should_enable = btn_cur != Buttons.DECEL_SET and btn_prev == Buttons.DECEL_SET
          should_enable = should_enable or (btn_cur == Buttons.RES_ACCEL and btn_prev != Buttons.RES_ACCEL)
          should_enable = should_enable and btn_cur != Buttons.CANCEL
          self.assertEqual(should_enable, self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())


class TestGmSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "gm_global_a_powertrain_generated"
  SAFETY_MODEL = CarParams.SafetyModel.gm

  STANDSTILL_THRESHOLD = 10 * 0.0311
  # Ensures ASCM is off on ASCM cars, and relay is not malfunctioning for camera-ACC cars
  RELAY_MALFUNCTION_ADDRS = {0: (0x180,), 2: (0x184,)}  # ASCMLKASteeringCmd, PSCMStatus
  BUTTONS_BUS = 0  # rx or tx
  BRAKE_BUS = 0  # tx only

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 15
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 128
  DRIVER_TORQUE_ALLOWANCE = 65
  DRIVER_TORQUE_FACTOR = 4

  PCM_CRUISE = True  # openpilot is tied to the PCM state if not longitudinal

  EXTRA_SAFETY_PARAM = 0

  def setUp(self):
    super().setUp()
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")

  def _pcm_status_msg(self, enable):
    if self.PCM_CRUISE:
      values = {"CruiseState": enable}
      return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)
    else:
      raise NotImplementedError

  def _speed_msg(self, speed):
    values = {"%sWheelSpd" % s: speed for s in ["RL", "RR"]}
    return self.packer.make_can_msg_safety("EBCMWheelSpdRear", 0, values)

  def _user_brake_msg(self, brake):
    # GM safety has a brake threshold of 8
    values = {"BrakePedalPos": 8 if brake else 0}
    return self.packer.make_can_msg_safety("ECMAcceleratorPos", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal2": 1 if gas else 0}
    if self.PCM_CRUISE:
      # Fill CruiseState with expected value if the safety mode reads cruise state from gas msg
      values["CruiseState"] = self.safety.get_controls_allowed()
    return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)

  def _torque_driver_msg(self, torque):
    # Safety tests assume driver torque is an int, use DBC factor
    values = {"LKADriverAppldTrq": torque * 0.01}
    return self.packer.make_can_msg_safety("PSCMStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKASteeringCmd": torque, "LKASteeringCmdActive": steer_req}
    return self.packer.make_can_msg_safety("ASCMLKASteeringCmd", 0, values)

  def _button_msg(self, buttons):
    values = {"ACCButtons": buttons}
    return self.packer.make_can_msg_safety("ASCMSteeringButton", self.BUTTONS_BUS, values)


class TestGmEVSafetyBase(TestGmSafetyBase):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV

  # existence of _user_regen_msg adds regen tests
  def _user_regen_msg(self, regen):
    values = {"RegenPaddle": 2 if regen else 0}
    return self.packer.make_can_msg_safety("EBCMRegenPaddle", 0, values)


class TestGmAscmSafety(GmLongitudinalBase, TestGmSafetyBase):
  TX_MSGS = [[0x180, 0], [0x409, 0], [0x40A, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0xA1, 1], [0x306, 1], [0x308, 1], [0x310, 1],  # obs bus
             [0x315, 2]]  # ch bus
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd
  FWD_BUS_LOOKUP: dict[int, int] = {}
  BRAKE_BUS = 2

  MAX_GAS = 1018
  MIN_GAS = -650  # maximum regen
  INACTIVE_GAS = -650

  @property
  def SAFETY_PARAM(self):
    return self.EXTRA_SAFETY_PARAM


class TestGmAscmEVSafety(TestGmAscmSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraSafetyBase(TestGmSafetyBase):
  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("ECMEngineStatus", 0, values)


class TestGmCameraSafety(TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180], 0: [0x184]}  # block LKAS message and PSCMStatus
  BUTTONS_BUS = 2  # tx only

  @property
  def SAFETY_PARAM(self):
    return GMSafetyFlags.HW_CAM | self.EXTRA_SAFETY_PARAM

  def test_buttons(self):
    # Only CANCEL button is allowed while cruise is enabled
    self.safety.set_controls_allowed(0)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    self.safety.set_controls_allowed(1)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      self.assertEqual(enabled, self._tx(self._button_msg(Buttons.CANCEL)))


class TestGmCameraEVSafety(TestGmCameraSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraLongitudinalSafety(GmLongitudinalBase, TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0], [0x315, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180, 0x2CB, 0x370, 0x315], 0: [0x184]}  # block LKAS, ACC messages and PSCMStatus
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB, 0x370, 0x315), 2: (0x184,)}
  BUTTONS_BUS = 0  # rx only

  MAX_GAS = 1346
  MIN_GAS = -540  # maximum regen
  INACTIVE_GAS = -500

  @property
  def SAFETY_PARAM(self):
    return GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | self.EXTRA_SAFETY_PARAM


class TestGmSdgmCameraLongitudinalSafety(TestGmCameraLongitudinalSafety):
  TX_MSGS = [[0x180, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0x315, 2], [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180, 0x2CB, 0x370], 0: [0x184]}
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB, 0x370), 2: (0x184,)}
  BRAKE_BUS = 2

  @property
  def SAFETY_PARAM(self):
    return GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.HW_SDGM | self.EXTRA_SAFETY_PARAM


class TestGmCameraLongitudinalEVSafety(TestGmCameraLongitudinalSafety, TestGmEVSafetyBase):
  pass


class AlwaysOnLateralMixin:
  # Personal: turns on ALT_EXP_ALWAYS_ON_LATERAL for the GM test classes below.
  # init_tests() (called by SafetyTestBase.setUp) clears alternative_experience, so set it and re-init the hooks.
  def setUp(self):
    super().setUp()
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL)
    self._reset_safety_hooks()
    self.safety.init_tests()  # restores safety_mode_cnt; always_on_lateral was latched by the safety init above


class TestGmAscmAlwaysOnLateralIgnoredSafety(AlwaysOnLateralMixin, TestGmAscmSafety):
  # flag is ignored on ASCM: every inherited test (brake disengages, button enable) must still hold
  pass


class TestGmCameraLongAlwaysOnLateralIgnoredSafety(AlwaysOnLateralMixin, TestGmCameraLongitudinalSafety):
  # flag is ignored on camera longitudinal (not stock cruise): brake still disengages
  pass


class TestGmSdgmCameraLongAlwaysOnLateralIgnoredSafety(AlwaysOnLateralMixin, TestGmSdgmCameraLongitudinalSafety):
  pass


class AlwaysOnLateralCameraMixin(AlwaysOnLateralMixin):
  # Personal: shared helpers/tests for stock-cruise camera harness with always-on lateral
  def setUp(self):
    super().setUp()
    self._rx(self._main_msg(True))

  def _main_msg(self, main_on, brake=0):
    values = {"CruiseMainOn": main_on, "BrakePressed": brake}
    return self.packer.make_can_msg_safety("ECMEngineStatus", 0, values)

  def _user_brake_msg(self, brake):
    # base brake helper must keep the main switch on, otherwise it would end always-on lateral
    return self._main_msg(True, brake)

  def _steer_allowed(self, torque=5):
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    return self._tx(self._torque_cmd_msg(torque, steer_req=1))

  # --- overrides of base tests whose expectation is brake/regen/ACC disengage ---

  def test_allow_user_brake_at_zero_speed(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    # always-on lateral: a brake (or regen, via the generated twin) press never disengages, even on its rising edge
    _user_brake_msg = _user_brake_msg or self._user_brake_msg
    self._rx(self._vehicle_moving_msg(0))
    self._rx(_user_brake_msg(0))
    self.safety.set_controls_allowed(1)
    self._rx(_user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(_user_brake_msg(0))
    self._rx(_user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(_user_brake_msg(0))

  def test_not_allow_user_brake_when_moving(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    # always-on lateral: brake (or regen) press while moving does not disengage lateral
    _user_brake_msg = _user_brake_msg or self._user_brake_msg
    self.safety.set_controls_allowed(1)
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(_user_brake_msg(1))
    self._rx(_user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._vehicle_moving_msg(0))

  def test_disable_control_allowed_from_cruise(self):
    # always-on lateral: stock ACC turning off must not clear controls_allowed
    self.safety.set_controls_allowed(1)
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())

  # --- new tests ---

  def test_brake_keeps_controls_and_steering(self):
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 10))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(3):
      self._rx(self._user_brake_msg(1))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self.safety.get_brake_pressed_prev())
      self.assertTrue(self._steer_allowed())
      self._rx(self._user_brake_msg(0))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self._steer_allowed())

  def test_acc_disengage_keeps_controls_and_steering(self):
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 10))
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_cruise_engaged_prev())
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._steer_allowed())

  def test_acc_reengage_while_steering_only_is_noop(self):
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    self.assertTrue(self._steer_allowed())

  def test_main_off_clears_controls_and_blocks_steering(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._steer_allowed())
    self._rx(self._main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._steer_allowed())
    for _ in range(5):
      self._rx(self._main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._steer_allowed())

  def test_main_off_while_steering_only(self):
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_off_during_brake(self):
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 10))
    self._rx(self._pcm_status_msg(True))
    self._rx(self._user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._main_msg(False, brake=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._steer_allowed())

  def test_main_off_then_acc_engage_edge(self):
    self._rx(self._pcm_status_msg(True))
    self._rx(self._main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    # stale ACC edge while main is off must not engage
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    # main back on alone does not engage (needs a new ACC rising edge)
    self._rx(self._main_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    # new ACC rising edge with main on engages
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_other_faults_still_disengage(self):
    # missing/invalid rx checks clear controls regardless of the flag
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_torque_limits_still_apply(self):
    self._rx(self._pcm_status_msg(True))
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP + 1, steer_req=1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(301, steer_req=1)))

  def test_steer_blocked_when_controls_not_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._steer_allowed())


class TestGmCameraAlwaysOnLateralSafety(AlwaysOnLateralCameraMixin, TestGmCameraSafety):
  pass


class TestGmCameraEVAlwaysOnLateralSafety(AlwaysOnLateralCameraMixin, TestGmCameraEVSafety):
  # also exercises the regen paddle path; the generated regen tests call the overridden brake tests above
  pass


class TestGmIgnition(common.SafetyTestBase):
  DBC = "gm_global_a_powertrain_generated"
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, mode):
    return self.packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0, {"SystemPowerMode": mode})

  # SystemPowerMode 2=Run, 3=Crank Request
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(2))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(2))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
