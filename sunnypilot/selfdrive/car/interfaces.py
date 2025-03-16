"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os

from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.car_helpers import can_fingerprint
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import HyundaiFlags, DBC as HYUNDAI_DBC
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import get_nn_model_path

import openpilot.system.sentry as sentry


def log_fingerprint(CP: structs.CarParams) -> None:
  if CP.carFingerprint == "MOCK":
    sentry.capture_fingerprint_mock()
  else:
    sentry.capture_fingerprint(CP.carFingerprint, CP.brand)


def initialize_neural_network_lateral_control(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params) -> None:
  nnlc_model_path, nnlc_model_name, fuzzy_fingerprint = get_nn_model_path(CP)

  if nnlc_model_path is None:
    cloudlog.error({"nnlc event": "car doesn't match any Neural Network model"})
    nnlc_model_path = "MOCK"

  if nnlc_model_path != "MOCK" and CP.steerControlType != structs.CarParams.SteerControlType.angle:
    CP_SP.neuralNetworkLateralControl.enabled = params.get_bool("NeuralNetworkLateralControl")

  if CP_SP.neuralNetworkLateralControl.enabled:
    CarInterfaceBase.configure_torque_tune(CP.carFingerprint, CP.lateralTuning)

  CP_SP.neuralNetworkLateralControl.modelPath = os.path.splitext(os.path.basename(nnlc_model_path))[0]
  CP_SP.neuralNetworkLateralControl.modelName = nnlc_model_name
  CP_SP.neuralNetworkLateralControl.fuzzyFingerprint = fuzzy_fingerprint


def setup_car_interface_sp(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params):
  if CP.brand == 'hyundai':
    if CP.flags & HyundaiFlags.MANDO_RADAR and CP.radarUnavailable:
      # Having this automatic without a toggle causes a weird process replay diff because
      # somehow it sees fewer logs than intended
      if params.get_bool("HyundaiRadarTracksToggle"):
        CP_SP.flags |= HyundaiFlagsSP.ENABLE_RADAR_TRACKS.value
        if params.get_bool("HyundaiRadarTracks"):
          CP.radarUnavailable = False

  initialize_neural_network_lateral_control(CP, CP_SP, params)


def initialize_car_interface_sp(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params, can_recv: CanRecvCallable,
                                can_send: CanSendCallable):
  if CP.brand == 'hyundai':
    if CP_SP.flags & HyundaiFlagsSP.ENABLE_RADAR_TRACKS:
      can_recv()
      _, fingerprint = can_fingerprint(can_recv)
      radar_unavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in HYUNDAI_DBC[CP.carFingerprint]

      radar_tracks = params.get_bool("HyundaiRadarTracks")
      radar_tracks_persistent = params.get_bool("HyundaiRadarTracksPersistent")

      params.put_bool_nonblocking("HyundaiRadarTracksConfirmed", radar_tracks)

      if not radar_tracks_persistent:
        params.put_bool_nonblocking("HyundaiRadarTracks", not radar_unavailable)
        params.put_bool_nonblocking("HyundaiRadarTracksPersistent", True)
