import json
import os
import pickle
import time
from collections import deque
from typing import Any

import numpy as np

from cereal import messaging
from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params
from sunnypilot.nnff_longitudinal_controller.learning_model import TinyNeuralNetwork
from tinygrad.tensor import Tensor


class LongitudinalLiveTuner:
  """Adaptive tuning system for longitudinal control parameters based on neural network learning.

  This system monitors vehicle behavior during stopping and starting to adapt
  longitudinal control parameters to the specific vehicle and driver preferences.
  """

  BLOCK_SIZE = 50  # Number of samples to gather before updating
  MIN_BLOCKS_NEEDED = 10  # Minimum blocks needed for valid calibration
  TUNE_LIMIT_PERCENT = 0.50  # Maximum percent change allowed from base value
  SMOOTH_FACTOR = 0.1  # How fast to adapt (0.1 = 10% of new value each update)
  MAX_AGE_DAYS = 90  # TODO: change this to resetting to first learned params/ else, if calibration reset pressed, reset to default
  STOPPING_SPEED_THRESHOLD = 0.2  # m/s
  STARTING_SPEED_THRESHOLD = 0.5  # m/s

  # Safety parameters
  MIN_SAFE_LEAD_DISTANCE = 5.0  # Minimum safe distance from lead car (meters)
  IDEAL_LEAD_DISTANCE = 6.0     # Ideal distance when stopped behind lead car

  # Neural network hyperparameters
  MEMORY_SIZE = 250  # Store up to 250 braking events
  LEARNING_RATE = 0.05  # How quickly to adapt to new brake profiles

  # How many update calls to reach 20 minutes of learning (at 20Hz)
  TARGET_TRAINING_STEPS = 20 * 60 * 20  # 20 minutes * 60 seconds * 20Hz

  def __init__(self, CP: structs.CarParams, original_kpV=None, original_kiV=None):
    # If CP.carFingerprint is provided, get default tuning; otherwise, use fallback defaults.
    if CP.carFingerprint is not None:
      default_cp = CarInterfaceBase.get_std_params(CP.carFingerprint)
    else:
      default_cp = structs.CarParams()
      # Set our fallback defaults from interfaces.py
      default_cp.longitudinalTuning.kf = 1.
      default_cp.longitudinalTuning.kpBP = [0.]
      default_cp.longitudinalTuning.kpV = [0.]
      default_cp.longitudinalTuning.kiBP = [0.]
      default_cp.longitudinalTuning.kiV = [0.]
      # Also assign default stopping parameters
      default_cp.vEgoStopping = 0.5
      default_cp.vEgoStarting = 0.5
      default_cp.stoppingDecelRate = 0.8

    self.vego_stopping_default = default_cp.vEgoStopping
    self.vego_starting_default = default_cp.vEgoStarting
    self.stopping_decel_rate_default = default_cp.stoppingDecelRate
    self.kf_default = default_cp.longitudinalTuning.kf
    self.kpBP_default = np.array(default_cp.longitudinalTuning.kpBP)
    self.kpV_default = np.array(default_cp.longitudinalTuning.kpV)
    self.kiBP_default = np.array(default_cp.longitudinalTuning.kiBP)
    self.kiV_default = np.array(default_cp.longitudinalTuning.kiV)

    # If the retrieved defaults do not include the longitudinal tuning values:
    if not hasattr(default_cp.longitudinalTuning, 'kf'):
      default_cp.longitudinalTuning.kf = 1.
    if not getattr(default_cp.longitudinalTuning, 'kpBP', None):
      default_cp.longitudinalTuning.kpBP = [0.]
    if not getattr(default_cp.longitudinalTuning, 'kpV', None):
      default_cp.longitudinalTuning.kpV = [0.]
    if not getattr(default_cp.longitudinalTuning, 'kiBP', None):
      default_cp.longitudinalTuning.kiBP = [0.]
    if not getattr(default_cp.longitudinalTuning, 'kiV', None):
      default_cp.longitudinalTuning.kiV = [0.]

    self.original_kpV = original_kpV if original_kpV is not None else np.array(default_cp.longitudinalTuning.kpV)
    self.original_kiV = original_kiV if original_kiV is not None else np.array(default_cp.longitudinalTuning.kiV)

    self.params = Params()
    self.sm = messaging.SubMaster(['longitudinalPlan', 'radarState'])
    # Initialize tuned parameters to individualized defaults
    self.kf = self.kf_default
    self.kpBP = self.kpBP_default.copy()
    self.kpV = self.kpV_default.copy()
    self.kiBP = self.kiBP_default.copy()
    self.kiV = self.kiV_default.copy()
    self.vego_stopping = self.vego_stopping_default
    self.vego_starting = self.vego_starting_default
    self.stopping_decel_rate = self.stopping_decel_rate_default

    self.kp_gain_factor = 1.0
    self.ki_gain_factor = 1.0

    # Data collection buffers
    self.stopping_data: list[float] = []
    self.starting_data: list[float] = []
    self.decel_rate_data: list[float] = []


    # Braking behavior tracking
    self.braking_events: deque[dict[str, Any]] = deque(maxlen=self.MEMORY_SIZE)
    self.current_braking_event: dict[str, Any] | None = None

    # Neural network for learning
    self.nn = TinyNeuralNetwork(
      input_size=10,
      hidden_size=24,
      output_size=10,
      lr=self.LEARNING_RATE,
      optimizer_type='adam',
      activation='leakyrelu',
      weight_init='he',
      dropout_rate=0.1
    )
    # Try to load previously saved model checkpoint
    self._load_model_checkpoint()

    self.best_nn_validation_loss = float('inf')
    self.training_step_count = 0.0
    self.training_progress = 0.0  # Progress from 0.0 to 1.0

    # Training data
    self.training_data: list[dict[str, Any]] = []
    self.validation_data: list[dict[str, Any]] = []
    self.last_training_time = int(time.time())
    self.training_interval = 60.0  # seconds between training sessions

    # For tracking long control behavior
    self.last_output_accel = 0.0
    self.integral_error = 0.0
    self.prev_error = 0.0
    self.last_pid_error = 0.0
    self.last_lead_distance = 0.0

    # Current collection state
    self.is_stopping = False
    self.is_starting = False
    self.stopping_start_time = 0.0
    self.starting_start_time = 0.0
    self.prev_speed = 0.0
    self.prev_accel = 0.0
    self.sample_idx = 0.0

    # Safety validation
    self.unsafe_stops_count = 0.0
    self.total_stops_count = 0.0
    self.safety_validation_window: deque[int] = deque(maxlen=50)
    self.unsafe_param_detected = False

    # Load saved parameters if available
    self._load_params()

  def _load_model_checkpoint(self):
      """Load neural network weights from saved checkpoint if available."""
      try:
          checkpoint_path = "nn_longitudinal.pkl"

          # Check if model exists
          if os.path.exists(checkpoint_path):
              with open(checkpoint_path, "rb") as f:
                  nn_weights = pickle.load(f)

              # Restore weights to the neural network
              if all(k in nn_weights for k in ['nn_w1', 'nn_b1', 'nn_w2', 'nn_b2', 'nn_w3', 'nn_b3']):
                  self.nn.W1 = Tensor(nn_weights['nn_w1'])
                  self.nn.b1 = Tensor(nn_weights['nn_b1'])
                  self.nn.W2 = Tensor(nn_weights['nn_w2'])
                  self.nn.b2 = Tensor(nn_weights['nn_b2'])
                  self.nn.W3 = Tensor(nn_weights['nn_w3'])
                  self.nn.b3 = Tensor(nn_weights['nn_b3'])
                  print(f"Loaded neural network checkpoint from {checkpoint_path}")
                  return True
      except Exception as e:
          print(f"Error loading model checkpoint: {e}")
      return False

  def _load_params(self):
    """Load saved tuning parameters from persistent storage."""
    try:
      # Check for forced reset flag
      if self.params.get("LongitudinalLiveTuneReset"):
        print("Forced reset of longitudinal live tune parameters")
        self.params.put("LongitudinalLiveTuneReset", "")
        return

      params_bytes = self.params.get("LongitudinalLiveTuneParams")
      if params_bytes is not None:
          try:
              # Try to load as pickle first
              stored_params = pickle.loads(params_bytes)
              print("Loaded parameters from pickle format")
          except Exception as e:
              # Fall back to JSON if pickle fails
              print(f"Pickle loading failed: {e}, trying JSON")
              stored_params = json.loads(params_bytes)
              print("Loaded parameters from JSON format")

          # Check if stored params are too old
          current_time = int(time.time())
          if current_time - stored_params.get('timestamp', 0.0) < self.MAX_AGE_DAYS * 24 * 3600:
              self.vego_stopping = stored_params.get('vego_stopping', self.vego_stopping_default)
              self.vego_starting = stored_params.get('vego_starting', self.vego_starting_default)
              self.stopping_decel_rate = stored_params.get('stopping_decel_rate', self.stopping_decel_rate_default)
              self.kp_gain_factor = stored_params.get('kp_gain_factor', self.kp_gain_factor)
              self.ki_gain_factor = stored_params.get('ki_gain_factor', self.ki_gain_factor)

              # Load neural network weights
              if all(k in stored_params for k in ['nn_w1', 'nn_b1', 'nn_w2', 'nn_b2', 'nn_w3', 'nn_b3']):
                  self.nn.W1 = Tensor(stored_params['nn_w1'])
                  self.nn.b1 = Tensor(stored_params['nn_b1'])
                  self.nn.W2 = Tensor(stored_params['nn_w2'])
                  self.nn.b2 = Tensor(stored_params['nn_b2'])
                  self.nn.W3 = Tensor(stored_params['nn_w3'])
                  self.nn.b3 = Tensor(stored_params['nn_b3'])

              # If we have learned braking profiles, load them too
              if 'braking_profiles' in stored_params:
                  self.braking_events = deque(stored_params['braking_profiles'], maxlen=self.MEMORY_SIZE)

              # Training metrics
              if 'training_progress' in stored_params:
                  self.training_progress = stored_params.get('training_progress', 0.0)
                  self.training_step_count = stored_params.get('training_step_count', 0)

              # Safety metrics
              if 'safety_metrics' in stored_params:
                  safety = stored_params['safety_metrics']
                  self.unsafe_stops_count = safety.get('unsafe_stops', 0)
                  self.total_stops_count = safety.get('total_stops', 0)

              self._validate_params()
    except Exception as e:
      print(f"Error loading longitudinal tuning params: {e}")

  def _save_params(self):
    """Save tuning parameters to persistent storage."""
    try:
        # Convert braking events to list for serialization (limit to most recent 50)
        braking_profiles = []
        try:
            braking_profiles = list(self.braking_events)[-50:] if self.braking_events else []
        except Exception as e:
            print(f"Error converting braking events: {e}")

        # Save neural network weights
        nn_weights = {}
        try:
            nn_weights = {
                'nn_w1': self.nn.W1.tolist() if hasattr(self.nn, 'W1') else [],
                'nn_b1': self.nn.b1.tolist() if hasattr(self.nn, 'b1') else [],
                'nn_w2': self.nn.W2.tolist() if hasattr(self.nn, 'W2') else [],
                'nn_b2': self.nn.b2.tolist() if hasattr(self.nn, 'b2') else [],
                'nn_w3': self.nn.W3.tolist() if hasattr(self.nn, 'W3') else [],
                'nn_b3': self.nn.b3.tolist() if hasattr(self.nn, 'b3') else [],
            }
        except Exception as e:
            print(f"Error converting NN weights: {e}")
            # Fall back to empty weights if conversion fails
            nn_weights = {
                'nn_w1': [], 'nn_b1': [],
                'nn_w2': [], 'nn_b2': [],
                'nn_w3': [], 'nn_b3': []
            }

        # Safety metrics
        safety_metrics = {
            'unsafe_stops': float(self.unsafe_stops_count),
            'total_stops': float(self.total_stops_count),
            'safety_score': 0 if self.total_stops_count == 0 else 1.0 - (
                        self.unsafe_stops_count / self.total_stops_count)
        }

        params_dict = {
            'vego_stopping': float(self.vego_stopping),
            'vego_starting': float(self.vego_starting),
            'stopping_decel_rate': float(self.stopping_decel_rate),
            'kp_gain_factor': float(self.kp_gain_factor),
            'ki_gain_factor': float(self.ki_gain_factor),
            'braking_profiles': braking_profiles,
            'timestamp': int(time.time()),
            'training_progress': float(self.training_progress),
            'training_step_count': int(self.training_step_count),
            'safety_metrics': safety_metrics,
            **nn_weights
        }

        try:
            pickled_data = pickle.dumps(params_dict)
            self.params.put("LongitudinalLiveTuneParams", pickled_data)
            print("Parameters saved to .pkl")
        except Exception as pickle_err:
            # Fall back to JSON if pickle fails
            print(f"Pickle serialization failed: {pickle_err}, falling back to JSON")
            self.params.put("LongitudinalLiveTuneParams", json.dumps(params_dict))
            print("Parameters saved to JSON")

    except Exception as e:
        print(f"Error saving longitudinal tuning params: {e}")

  def _validate_params(self):
    """Ensure parameters are within acceptable ranges and meet safety criteria."""
    # Apply limits to how far we can adapt from defaults
    lower_limit = 1.0 - self.TUNE_LIMIT_PERCENT
    upper_limit = 1.0 + self.TUNE_LIMIT_PERCENT

    # Safety checks for stopping distance - make sure we never stop too late
    if self.vego_stopping < 0.1:
      # Minimum non-zero stopping speed to prevent unsafe stops
      self.vego_stopping = 0.1

    # Ensure vego_stopping isn't too low (would cause late braking)
    self.vego_stopping = max(
      self.vego_stopping,
      self.vego_stopping_default * lower_limit
    )

    self.vego_stopping = np.clip(
      self.vego_stopping,
      self.vego_stopping_default * lower_limit,
      self.vego_stopping_default * upper_limit
    )

    self.vego_starting = np.clip(
      self.vego_starting,
      self.vego_starting_default * lower_limit,
      self.vego_starting_default * upper_limit
    )

    # Make sure decel rate is high enough for safety
    self.stopping_decel_rate = np.clip(
      self.stopping_decel_rate,
      self.stopping_decel_rate_default * lower_limit,
      self.stopping_decel_rate_default * upper_limit
    )

    # Also constrain PID gain factors
    self.kp_gain_factor = np.clip(self.kp_gain_factor, 0.75, 1.25)
    self.ki_gain_factor = np.clip(self.ki_gain_factor, 0.75, 1.25)

    # Additional safety checks to validate parameter combinations
    # Balance between stopping speed and decel rate
    if self.stopping_decel_rate < 0.2 and self.vego_stopping > 0.3:
      # If decel rate is too gentle but stopping speed is too high, this could cause unsafe stops
      self.vego_stopping = min(self.vego_stopping, 0.25)

  def update(self, CS: structs.CarState, actuators: structs.CarControl.Actuators):
    """Update the live tuner with new data."""
    self.sample_idx += 1

    v_ego = CS.vEgo
    current_accel = actuators.accel

    # Calculate PID error for monitoring controller behavior
    pid_error = 0
    if hasattr(self.sm['longitudinalPlan'], 'aTarget') and hasattr(CS, 'aEgo'):
      pid_error = self.sm['longitudinalPlan'].aTarget - CS.aEgo
      self.last_pid_error = pid_error

    # Capture lead vehicle distance for safety checks
    lead_dist = self.sm['radarState'].leadOne.dRel if hasattr(self.sm, 'radarState') and self.sm['radarState'].leadOne.status else None
    if lead_dist is not None:
      self.last_lead_distance = lead_dist

    # Start tracking a new braking event when deceleration begins
    if not self.is_stopping and CS.out.cruiseState.enabled and CS.aEgo < -0.10 and v_ego > 1.0:
      self.is_stopping = True
      self.stopping_start_time = int(time.time())
      # Initialize new braking event
      self.current_braking_event = {
          'initial_speed': v_ego,
          'decel_samples': [],
          'comfort_scores': [],
          'jerk_samples': [],
          'lead_distances': [],
          'pid_errors': [],
          'distance': 0.0,
          'duration': 0.0,
          'final_stopping_distance': 0.0,
        'timestamp': int(time.time())
      }

    # Record braking event data
    if self.is_stopping:
      # Calculate deceleration rate and jerk
      decel_rate = (self.prev_speed - v_ego) / DT_CTRL if self.prev_speed > v_ego else 0.0
      jerk = (current_accel - self.last_output_accel) / DT_CTRL

      # Update braking event data
      if self.current_braking_event is not None:
        self.current_braking_event['decel_samples'].append(decel_rate)
        self.current_braking_event['jerk_samples'].append(jerk)
        self.current_braking_event['distance'] += v_ego * DT_CTRL
        self.current_braking_event['duration'] += DT_CTRL

        # Store lead distance if available
        if lead_dist is not None:
          self.current_braking_event['lead_distances'].append(lead_dist)

        # Store PID error for controller analysis
        self.current_braking_event['pid_errors'].append(pid_error)

        # Compute a comfort score (lower jerk = more comfort)
        comfort_score = 1.0 / (1.0 + abs(jerk))
        self.current_braking_event['comfort_scores'].append(comfort_score)

      # Check if we've reached a stop
      if v_ego < self.STOPPING_SPEED_THRESHOLD:
        if self.current_braking_event is not None:
          # Calculate final metrics
          self.current_braking_event['final_speed'] = v_ego

          # Record final stopping distance (how far from target)
          self.current_braking_event['final_stopping_distance'] = lead_dist if lead_dist is not None else 0.0

          # Safety check: track if this was an unsafe stop (too close to lead)
          self.total_stops_count += 1
          if lead_dist is not None and lead_dist < self.MIN_SAFE_LEAD_DISTANCE:
            self.unsafe_stops_count += 1
            self.safety_validation_window.append(0)  # 0 = unsafe
          else:
            self.safety_validation_window.append(1)  # 1 = safe

          # Check if we need to revert unsafe parameters
          if len(self.safety_validation_window) >= 5:  # Need at least 5 stops to evaluate
            unsafe_ratio = 1.0 - (sum(self.safety_validation_window) / len(self.safety_validation_window))
            if unsafe_ratio > 0.3:  # If more than 30% of recent stops are unsafe
              # Revert to more conservative parameters
              self.unsafe_param_detected = True
              self.vego_stopping = min(self.vego_stopping + 0.05, self.vego_stopping_default * 1.1)
              self.stopping_decel_rate = max(self.stopping_decel_rate * 0.9, self.stopping_decel_rate_default * 0.9)
              print(f"Unsafe stopping parameters detected! Reverting to safer values. Unsafe ratio: {unsafe_ratio:.2f}")

          # Store the braking event for learning
          self.braking_events.append(self.current_braking_event)
          self.current_braking_event = None

          # Process the collected data to update parameters
          self._analyze_braking_behavior()

        # Record data for basic tuning
        self.stopping_data.append(v_ego)
        self.is_stopping = False

    # Detect starting event (transitioning from standstill)
    if not self.is_starting and CS.out.cruiseState.enabled and v_ego < 0.1 < actuators.accel:
      self.is_starting = True
      self.starting_start_time = int(time.time())

    # Record starting data
    if self.is_starting:
      # Check if we've started moving
      if v_ego > self.STARTING_SPEED_THRESHOLD:
        self.starting_data.append(v_ego)
        self.is_starting = False

    # Record PID controller behavior
    self.last_output_accel = current_accel

    # Detect deceleration rate for basic tuning
    if self.prev_speed > v_ego > 0.01 and CS.out.cruiseState.enabled:
      decel_rate = (self.prev_speed - v_ego) / DT_CTRL
      self.decel_rate_data.append(decel_rate)

    self.prev_speed = v_ego
    self.prev_accel = current_accel

    # Periodically train the neural network
    current_time = int(time.time())
    if current_time - self.last_training_time > self.training_interval:
      self._train_network()
      self.last_training_time = current_time

    # Process data if we have enough samples
    if self.sample_idx >= self.BLOCK_SIZE:
      self._process_data()
      self.sample_idx = 0

    # Track progress towards fully trained model
    if self.training_step_count < self.TARGET_TRAINING_STEPS:
      self.training_step_count += 1
      self.training_progress = min(1.0, self.training_step_count / self.TARGET_TRAINING_STEPS)

  def force_update(self, CS: structs.CarState, actuators: structs.CarControl.Actuators):
    # Force an immediate data update and training trigger
    self.update(CS, actuators)

  def _train_network(self):
    """Train the neural network on collected data."""
    if len(self.braking_events) < 10:  # Need enough data for meaningful training
      return

    # Extract training features and targets
    features = []
    targets = []

    for event in self.braking_events:
      if len(event.get('decel_samples', [])) == 0:
          continue

      # Create feature vector
      try:
          feature = [
              event['initial_speed'],
              np.mean(event['decel_samples']),
              np.std(event['jerk_samples']) if event['jerk_samples'] else 0,
              np.mean(event['comfort_scores']) if event['comfort_scores'] else 0.5,
              event['final_stopping_distance'] if event.get('final_stopping_distance') is not None else 8.0,
              event['duration'],
              len(event.get('lead_distances', [])) / max(1, len(event['decel_samples'])),
              np.mean(self.original_kpV),
              np.mean(self.original_kiV),
              1.0  # Default feature padding to match input_size=10
          ]
          feature = np.clip(feature, -10, 10)
          features.append(feature)

          # Target values normalized between 0 and 1 for sigmoid output
          eps = 1e-9  # Epsilon value to avoid division by zero
          target = [
              (self.vego_stopping - self.vego_stopping_default * 0.5) / (self.vego_stopping_default * 0.5 + eps),
              (self.vego_starting - self.vego_starting_default * 0.5) / (self.vego_starting_default * 0.5 + eps),
              (self.stopping_decel_rate - self.stopping_decel_rate_default * 0.5) / (
                          self.stopping_decel_rate_default * 0.5 + eps),
              (self.kp_gain_factor - 0.75) / 0.5,
              (self.ki_gain_factor - 0.75) / 0.5,
              (self.kf - self.kf_default * 0.5) / (self.kf_default * 0.5 + eps),
              (np.mean(self.kpBP) - np.mean(self.kpBP_default) * 0.5) / (np.mean(self.kpBP_default) * 0.5 + eps),
              (np.mean(self.kpV) - np.mean(self.kpV_default) * 0.5) / (np.mean(self.kpV_default) * 0.5 + eps),
              (np.mean(self.kiBP) - np.mean(self.kiBP_default) * 0.5) / (np.mean(self.kiBP_default) * 0.5 + eps),
              (np.mean(self.kiV) - np.mean(self.kiV_default) * 0.5) / (np.mean(self.kiV_default) * 0.5 + eps)
          ]
          targets.append(target)
      except Exception as e:
          print(f"Error processing event for neural network: {e}")
          continue

    # Split into training and validation sets
    if len(features) < 5:
        print("Not enough valid training examples")
        return

    train_size = int(len(features) * 0.8)

    # Convert to numpy arrays with explicit dtype
    try:
        train_features_np = np.array(features[:train_size], dtype=np.float32)
        train_targets_np = np.array(targets[:train_size], dtype=np.float32)
        val_features_np = np.array(features[train_size:], dtype=np.float32) if len(features) > train_size else np.array(
            [], dtype=np.float32)
        val_targets_np = np.array(targets[train_size:], dtype=np.float32) if len(targets) > train_size else np.array([],
                                                                                                                     dtype=np.float32)

        # Verify shapes match network expectations
        print(f"Training features shape: {train_features_np.shape}")
        print(f"Training targets shape: {train_targets_np.shape}")
    except Exception as e:
        print(f"Error creating training arrays: {e}")
        return

    try:
        # Train the model
        iterations_per_batch = 100
        batch_size = 1  # Use single examples for test stability

        for i in range(0, len(train_features_np), batch_size):
            if i + batch_size <= len(train_features_np):  # Ensure we have a complete batch
                batch_x = Tensor(train_features_np[i:i + batch_size])
                batch_y = Tensor(train_targets_np[i:i + batch_size])

                try:
                    # Simple forward pass to check if dimensions work
                    _ = self.nn.forward(batch_x)
                    # Only try training if forward pass succeeds
                    self.nn.train(batch_x, batch_y, iterations=iterations_per_batch)
                except Exception as e:
                    print(f"Error during neural network training batch: {e}")
                    # Continue with next batch
                    continue

        if len(val_features_np) > 0:
            try:
                val_pred = self.nn.forward(Tensor(val_features_np))
                val_loss = ((val_pred - Tensor(val_targets_np)) ** 2).mean().numpy()
                print(f"Neural network training: validation error = {val_loss:.6f}")

                # Print detailed prediction information to understand impact of loss changes
                if len(val_pred) > 0:
                    pred_values = val_pred.numpy()[0]  # First prediction
                    target_values = val_targets_np[0]  # First target
                    param_names = ['vego_stopping', 'vego_starting', 'stopping_decel_rate',
                                   'kp_factor', 'ki_factor', 'kf', 'kpBP', 'kpV', 'kiBP', 'kiV']

                    print("\nPrediction details (normalized [0-1]):")
                    for name, pred, target in zip(param_names, pred_values, target_values, strict=False):
                        error = abs(pred - target)
                        print(f"  {name}: pred={pred:.4f}, target={target:.4f}, error={error:.4f}")

                    # Show how these translate to actual parameter values
                    print("\nDenormalized parameter values:")
                    # Example for a few key parameters
                    vego_stopping_pred = pred_values[0] * (self.vego_stopping_default * 0.5) + (
                                self.vego_stopping_default * 0.5)
                    vego_stopping_target = target_values[0] * (self.vego_stopping_default * 0.5) + (
                                self.vego_stopping_default * 0.5)
                    print(
                        f"  vego_stopping: current={self.vego_stopping:.4f}, pred={vego_stopping_pred:.4f}, target={vego_stopping_target:.4f}")

                    decel_rate_pred = pred_values[2] * (self.stopping_decel_rate_default * 0.5) + (
                                self.stopping_decel_rate_default * 0.5)
                    decel_rate_target = target_values[2] * (self.stopping_decel_rate_default * 0.5) + (
                                self.stopping_decel_rate_default * 0.5)
                    print(
                        f"  stopping_decel_rate: current={self.stopping_decel_rate:.4f}, pred={decel_rate_pred:.4f}, target={decel_rate_target:.4f}")

                # Track if validation loss is improving with proper handling of infinity
                if val_loss < self.best_nn_validation_loss:
                    # Handle the case where best_nn_validation_loss is infinity
                    if np.isinf(self.best_nn_validation_loss):
                        improvement = 100.0  # Just report 100% improvement from infinity
                    else:
                        improvement = (self.best_nn_validation_loss - val_loss) / self.best_nn_validation_loss * 100

                    self.best_nn_validation_loss = val_loss
                    print(f"Validation loss improved by {improvement:.2f}%, saving checkpoint.")
                    try:
                        self._save_model_checkpoint()
                    except Exception as e:
                        print(f"Error saving model checkpoint: {e}")

                # After successful training and validation, apply the trained model
                self.apply_trained_model(direct_training_application=True)

            except Exception as e:
                print(f"Error during validation: {e}")

        # Try to save params
        try:
            self._save_params()
        except Exception as e:
            print(f"Error saving parameters: {e}")

    except Exception as e:
        print(f"Error in neural network training: {e}")

  def apply_trained_model(self, direct_training_application=False):
      """Apply the trained neural network model to update parameters."""
      if len(self.braking_events) < 5:
          print("Not enough braking events to apply model")
          return False

      # Use recent events to create representative features
      recent_events = list(self.braking_events)[-10:]

      # Calculate average metrics from recent events
      avg_speed = np.mean([event['initial_speed'] for event in recent_events])
      avg_decel = np.mean([np.mean(event.get('decel_samples', [0.8])) for event in recent_events])
      avg_jerk_std = np.mean([np.std(event.get('jerk_samples', [0.0])) for event in recent_events])
      avg_comfort = np.mean([np.mean(event.get('comfort_scores', [0.5])) for event in recent_events])
      stopping_distances = [event.get('final_stopping_distance', 8.0) for event in recent_events if
                            event.get('final_stopping_distance') is not None]
      avg_stopping_distance = np.mean(stopping_distances) if stopping_distances else 8.0
      avg_duration = np.mean([event['duration'] for event in recent_events])
      avg_lead_dist = np.mean([np.mean(event.get('lead_distances', [10.0])) for event in recent_events])

      # Create input features for the neural network
      features = np.array([
          avg_speed,
          avg_decel,
          avg_jerk_std,
          avg_comfort,
          avg_stopping_distance,
          avg_duration,
          avg_lead_dist,
          np.mean(self.original_kpV),
          np.mean(self.original_kiV),
          1.0
      ])

      # Clip features to prevent extreme values
      features = np.clip(features, -10, 10)

      # Record original values for comparison
      original_values = {
          'vego_stopping': self.vego_stopping,
          'vego_starting': self.vego_starting,
          'stopping_decel_rate': self.stopping_decel_rate,
          'kp_gain_factor': self.kp_gain_factor,
          'ki_gain_factor': self.ki_gain_factor
      }

      # Get neural network prediction with proper GPU error handling and CPU fallback
      try:
          # Try GPU prediction first
          prediction = self.nn.forward(Tensor(features))
          prediction_array = prediction.numpy()
          print(f"\nApplying model - Neural network raw predictions: {prediction_array}")
      except Exception as e:
          print(f"Error during model prediction: {e}")
          print("Attempting CPU fallback for prediction...")
          try:
              # Try to force CPU execution
              original_device = os.environ.get('TINYGRAD_DEVICE', '')
              os.environ['TINYGRAD_DEVICE'] = 'CPU'
              # Create a new tensor on CPU
              cpu_features = Tensor(features.astype(np.float32))
              # Run prediction on CPU
              prediction = self.nn.forward(cpu_features)
              prediction_array = prediction.numpy()
              # Restore original device setting
              if original_device:
                  os.environ['TINYGRAD_DEVICE'] = original_device
              else:
                  os.environ.pop('TINYGRAD_DEVICE', None)
              print(f"CPU fallback successful - predictions: {prediction_array}")
          except Exception as cpu_error:
              print(f"CPU fallback also failed: {cpu_error}")
              print("Using default values instead")
              return False

      # Use more aggressive learning rate if this is from direct training application
      learning_rate = 0.3 if direct_training_application else 0.1

      # Safely apply predictions
      try:
          # Denormalize predictions and apply to parameters
          self.vego_stopping = (1.0 - learning_rate) * self.vego_stopping + learning_rate * (
                  prediction_array[0] * (self.vego_stopping_default * 0.5) + (self.vego_stopping_default * 0.5)
          )
          self.vego_starting = (1.0 - learning_rate) * self.vego_starting + learning_rate * (
                  prediction_array[1] * (self.vego_starting_default * 0.5) + (self.vego_starting_default * 0.5)
          )
          self.stopping_decel_rate = (1.0 - learning_rate) * self.stopping_decel_rate + learning_rate * (
                  prediction_array[2] * (self.stopping_decel_rate_default * 0.5) + (
                      self.stopping_decel_rate_default * 0.5)
          )
          self.kp_gain_factor = (1.0 - learning_rate) * self.kp_gain_factor + learning_rate * (
                  prediction_array[3] * 0.5 + 0.75
          )
          self.ki_gain_factor = (1.0 - learning_rate) * self.ki_gain_factor + learning_rate * (
                  prediction_array[4] * 0.5 + 0.75
          )
      except Exception as e:
          print(f"Error applying model predictions: {e}")
          # Don't update parameters if there was an error
          return False

      # Log parameter changes
      print("\nParameter changes after applying trained model:")
      for param, before in original_values.items():
          after = getattr(self, param)
          change = after - before
          pct_change = (change / before) * 100 if before != 0 else float('inf')
          print(f"  {param}: {before:.4f} → {after:.4f} (Δ{change:.4f}, {pct_change:.2f}%)")

      # Validate parameters to ensure safety
      self._validate_params()
      return True

  def _process_data(self):
    """Process collected data and update parameters."""
    # Process stopping speed data
    if len(self.stopping_data) > self.MIN_BLOCKS_NEEDED:
      new_vego_stopping = np.median(self.stopping_data)

      # Apply smoothing
      self.vego_stopping = (1.0 - self.SMOOTH_FACTOR) * self.vego_stopping + self.SMOOTH_FACTOR * new_vego_stopping
      self.stopping_data = []  # Clear buffer

    # Process starting speed data
    if len(self.starting_data) > self.MIN_BLOCKS_NEEDED:
      new_vego_starting = np.median(self.starting_data)

      # Apply smoothing
      self.vego_starting = (1.0 - self.SMOOTH_FACTOR) * self.vego_starting + self.SMOOTH_FACTOR * new_vego_starting
      self.starting_data = []  # Clear buffer

    # Process deceleration rate data
    if len(self.decel_rate_data) > self.MIN_BLOCKS_NEEDED:
      # Take the 75th percentile for a conservative estimate
      # (higher values mean more aggressive stopping)
      new_stopping_decel_rate = np.percentile(self.decel_rate_data, 75)

      # Apply smoothing
      self.stopping_decel_rate = (1.0 - self.SMOOTH_FACTOR) * self.stopping_decel_rate + self.SMOOTH_FACTOR * new_stopping_decel_rate
      self.decel_rate_data = []  # Clear buffer

    # Validate and save parameters
    self._validate_params()
    self._save_params()

  def _analyze_braking_behavior(self):
    """Analyze collected braking events to optimize parameters using neural-network."""
    if len(self.braking_events) < 5:  # Need minimum data for accurate training
      return

    # Get the most recent events (more weight to recent events)
    recent_events = list(self.braking_events)[-20:]

    # Extract key metrics
    avg_comfort_scores = []
    avg_decel_rates = []
    stopping_accuracies = []
    stopping_distances = []

    for event in recent_events:
      if 'comfort_scores' in event and event['comfort_scores']:
        avg_comfort_scores.append(np.mean(event['comfort_scores']))

      if 'decel_samples' in event and event['decel_samples']:
        # Use 90th percentile for max deceleration rate
        avg_decel_rates.append(np.percentile(event['decel_samples'], 90))

      # Analyze stopping distance (critical for safety)
      if 'final_stopping_distance' in event and event['final_stopping_distance'] is not None:
        stopping_distance = event['final_stopping_distance']
        stopping_distances.append(stopping_distance)

        # Safety check - prioritize stopping at safe distance
        if stopping_distance < self.MIN_SAFE_LEAD_DISTANCE:
          # If stopping distance is too close to lead, we need more conservative params
          # Unsafe stops get a very low score
          accuracy = 0.0
        else:
          # Otherwise, optimal distance is between 5-7 meters
          # Formula gives highest score (1.0) at ideal distance and decreases as we move away
          accuracy = 1.0 / (1.0 + abs(stopping_distance - self.IDEAL_LEAD_DISTANCE) / 3.0)

        stopping_accuracies.append(accuracy)

    # Only proceed if we have valid data
    if not avg_comfort_scores or not avg_decel_rates:
      return

    # Safety check for stopping distance
    if stopping_distances and np.mean(stopping_distances) < self.MIN_SAFE_LEAD_DISTANCE:
      # If we're consistently stopping too close, make immediate corrections
      self.vego_stopping = min(self.vego_stopping * 1.05, self.vego_stopping_default * 1.2)  # Increase stopping threshold
      self.stopping_decel_rate = max(self.stopping_decel_rate * 1.1, self.stopping_decel_rate_default * 0.9)  # More aggressive braking
      print(f"Safety correction: Adjusting parameters due to unsafe stopping distance < {self.MIN_SAFE_LEAD_DISTANCE}m")
      self._validate_params()
      self._save_params()
      return

    # Calculate optimal decel rate based on comfort and accuracy
    # More weight to comfort for regular stops, more weight to accuracy for emergency stops
    comfort_weight = 0.6
    accuracy_weight = 0.4

    if stopping_accuracies:
      # If stopping accuracy is poor, shift weights to prioritize accuracy
      avg_accuracy = np.mean(stopping_accuracies)
      if avg_accuracy < 0.5:  # Below threshold, adjust weights
        comfort_weight = 0.3
        accuracy_weight = 0.7

    # Weighted average of metrics to determine optimal parameters
    avg_comfort = np.mean(avg_comfort_scores) if avg_comfort_scores else 0.5
    avg_decel = np.mean(avg_decel_rates) if avg_decel_rates else self.stopping_decel_rate

    # Neural-network adaptive learning:
    # Comfort score affects decel rate (higher comfort = gentler deceleration)
    comfort_factor = np.clip(avg_comfort, 0.5, 1.0)

    # Use the neural network if we have enough data for accurate prediction
    if len(recent_events) >= 10 and hasattr(self, 'nn'):
      # Prepare input features that represent the current driving situation
      avg_speed = np.mean([event['initial_speed'] for event in recent_events])
      avg_lead_dist = np.mean([np.mean(event.get('lead_distances', [10.0])) for event in recent_events])

      features = np.array([
        avg_speed,
        avg_decel,
        np.mean([np.std(event.get('jerk_samples', [0.0])) for event in recent_events]),  # Jerk variability
        avg_comfort, # Comfort score
        np.mean(stopping_distances) if stopping_distances else 8.0,  # Average stopping distance
        np.mean([event['duration'] for event in recent_events]),  # Average stop duration
        avg_lead_dist,
        len(stopping_distances) / max(1, len(recent_events))  # Traffic density proxy
      ])

      # Normalize and clip the features to prevent training issues
      features = np.clip(features, -10, 10)

      # Get NN prediction for optimal parameters
      prediction = self.nn.forward(features)

      # Log raw prediction values before denormalization
      print(f"\nNeural network raw predictions: {prediction.numpy()}")

      # Store original values for comparison
      original_values = {
          'vego_stopping': self.vego_stopping,
          'vego_starting': self.vego_starting,
          'stopping_decel_rate': self.stopping_decel_rate,
          'kp_gain_factor': self.kp_gain_factor,
          'ki_gain_factor': self.ki_gain_factor
      }

      # Denormalize outputs from [0,1] range to actual parameter values
      target_vego_stopping = prediction[0] * (self.vego_stopping_default * 0.5) + (self.vego_stopping_default * 0.5)
      target_vego_starting = prediction[1] * (self.vego_starting_default * 0.5) + (self.vego_starting_default * 0.5)
      target_decel_rate = prediction[2] * (self.stopping_decel_rate_default * 0.5) + (self.stopping_decel_rate_default * 0.5)
      target_kp_factor = prediction[3] * 0.5 + 0.75
      target_ki_factor = prediction[4] * 0.5 + 0.75
      target_kf = prediction[5] * (self.kf_default * 0.5) + (self.kf_default * 0.5)
      # Epsilon for safe division
      eps = 1e-9
      kp_mean = max(eps, abs(np.mean(self.kpBP_default)))
      target_kpBP = prediction[6] * (kp_mean * 0.5) + (kp_mean * 0.5)
      kpV_mean = max(eps, abs(np.mean(self.kpV_default)))
      target_kpV = prediction[7] * (kpV_mean * 0.5) + (kpV_mean * 0.5)
      kiBP_mean = max(eps, abs(np.mean(self.kiBP_default)))
      target_kiBP = prediction[8] * (kiBP_mean * 0.5) + (kiBP_mean * 0.5)
      kiV_mean = max(eps, abs(np.mean(self.kiV_default)))
      target_kiV = prediction[9] * (kiV_mean * 0.5) + (kiV_mean * 0.5)

      # Update parameters using suitable learning rates:
      safety_lr = 0.1
      comfort_lr = 0.05
      self.vego_stopping = (1.0 - safety_lr * accuracy_weight) * self.vego_stopping + (safety_lr * accuracy_weight) * target_vego_stopping
      self.stopping_decel_rate = (1.0 - safety_lr * accuracy_weight) * self.stopping_decel_rate + (safety_lr * accuracy_weight) * target_decel_rate
      self.vego_starting = (1.0 - comfort_lr * comfort_weight) * self.vego_starting + (comfort_lr * comfort_weight) * target_vego_starting
      self.kp_gain_factor = (1.0 - comfort_lr * comfort_weight) * self.kp_gain_factor + (comfort_lr * comfort_weight) * target_kp_factor
      self.ki_gain_factor = (1.0 - comfort_lr * comfort_weight) * self.ki_gain_factor + (comfort_lr * comfort_weight) * target_ki_factor
      self.kf = (1.0 - safety_lr * accuracy_weight) * self.kf + (safety_lr * accuracy_weight) * target_kf
      self.kpBP = (1.0 - comfort_lr * comfort_weight) * self.kpBP + (comfort_lr * comfort_weight) * target_kpBP
      self.kpV = (1.0 - comfort_lr * comfort_weight) * self.kpV + (comfort_lr * comfort_weight) * target_kpV
      self.kiBP = (1.0 - comfort_lr * comfort_weight) * self.kiBP + (comfort_lr * comfort_weight) * target_kiBP
      self.kiV = (1.0 - comfort_lr * comfort_weight) * self.kiV + (comfort_lr * comfort_weight) * target_kiV

      # Log the parameter changes to see the actual effect
      print("\nParameter changes after neural network prediction:")
      for param, before in original_values.items():
          after = getattr(self, param)
          change = after - before
          # Handle division by zero
          if before == 0:
              pct_change = float('inf') if change > 0 else 0.0
          else:
              pct_change = (change / before) * 100
          print(f"  {param}: {before:.4f} → {after:.4f} (Δ{change:.4f}, {pct_change:.2f}%)")

    else:
      # Fallback to simpler adaptive learning if neural network isn't ready
      # Adjust deceleration rate based on comfort factor
      target_decel_rate = avg_decel * (0.85 + 0.3 * (1.0 - comfort_factor))

      # Apply learning rate to the adjustment
      self.stopping_decel_rate = (1.0 - self.LEARNING_RATE) * self.stopping_decel_rate + self.LEARNING_RATE * target_decel_rate

      # Adjust PID gain factors based on stopping performance
      if stopping_accuracies:
        avg_accuracy = np.mean(stopping_accuracies)
        # Adjust P gain based on stopping accuracy
        target_kp_factor = 1.0 + (avg_accuracy - 0.5) * 0.5  # Range ~0.75-1.25
        # Adjust I gain based on stopping comfort
        target_ki_factor = 1.0 + (comfort_factor - 0.75) * 0.5  # Range ~0.75-1.25

        # Apply learning rate to gain adjustments
        self.kp_gain_factor = (1.0 - self.LEARNING_RATE) * self.kp_gain_factor + self.LEARNING_RATE * target_kp_factor
        self.ki_gain_factor = (1.0 - self.LEARNING_RATE) * self.ki_gain_factor + self.LEARNING_RATE * target_ki_factor

    # Ensure parameters are within bounds
    self._validate_params()

    # Save more frequently when we're actively learning
    self._save_params()

  def get_tuned_params(self):
    """Return current tuned parameters."""
    return {
      'vego_stopping': self.vego_stopping,
      'vego_starting': self.vego_starting,
      'stopping_decel_rate': self.stopping_decel_rate,
      'kp_gain_factor': self.kp_gain_factor,
      'ki_gain_factor': self.ki_gain_factor,
      'kf': self.kf,
      'kpBP': self.kpBP.tolist() if isinstance(self.kpBP, np.ndarray) else self.kpBP,
      'kpV': self.kpV.tolist() if isinstance(self.kpV, np.ndarray) else self.kpV,
      'kiBP': self.kiBP.tolist() if isinstance(self.kiBP, np.ndarray) else self.kiBP,
      'kiV': self.kiV.tolist() if isinstance(self.kiV, np.ndarray) else self.kiV
    }

  def get_pid_gains(self, original_kp, original_ki):
    """Apply the learned gain adjustments to the original PID values."""
    return original_kp * self.kp_gain_factor, original_ki * self.ki_gain_factor

  def _save_model_checkpoint(self):
    """Save the neural network weights as loss improves."""
    nn_weights = {
        'nn_w1': self.nn.W1.tolist(),
        'nn_b1': self.nn.b1.tolist(),
        'nn_w2': self.nn.W2.tolist(),
        'nn_b2': self.nn.b2.tolist(),
        'nn_w3': self.nn.W3.tolist(),
        'nn_b3': self.nn.b3.tolist(),
    }
    with open("nn_longitudinal.pkl", "wb") as f:
      pickle.dump(nn_weights, f)
