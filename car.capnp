using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0x8e2af1e708af8b8d;

# ******* events causing controls state machine transition *******

struct CarEvent @0x9b1657f34caf3ad3 {
  name @0 :EventName;

  # event types
  enable @1 :Bool;
  noEntry @2 :Bool;
  warning @3 :Bool;   # alerts presented only when  enabled or soft disabling
  userDisable @4 :Bool;
  softDisable @5 :Bool;
  immediateDisable @6 :Bool;
  preEnable @7 :Bool;
  permanent @8 :Bool; # alerts presented regardless of openpilot state

  enum EventName @0xbaa8c5d505f727de {
    canError @0;
    steerUnavailable @1;
    brakeUnavailable @2;
    wrongGear @4;
    doorOpen @5;
    seatbeltNotLatched @6;
    espDisabled @7;
    wrongCarMode @8;
    steerTempUnavailable @9;
    reverseGear @10;
    buttonCancel @11;
    buttonEnable @12;
    pedalPressed @13;
    cruiseDisabled @14;
    speedTooLow @17;
    outOfSpace @18;
    overheat @19;
    calibrationIncomplete @20;
    calibrationInvalid @21;
    controlsMismatch @22;
    pcmEnable @23;
    pcmDisable @24;
    noTarget @25;
    radarFault @26;
    brakeHold @28;
    parkBrake @29;
    manualRestart @30;
    lowSpeedLockout @31;
    plannerError @32;
    joystickDebug @34;
    steerTempUnavailableSilent @35;
    resumeRequired @36;
    preDriverDistracted @37;
    promptDriverDistracted @38;
    driverDistracted @39;
    preDriverUnresponsive @43;
    promptDriverUnresponsive @44;
    driverUnresponsive @45;
    belowSteerSpeed @46;
    lowBattery @48;
    vehicleModelInvalid @50;
    accFaulted @51;
    sensorDataInvalid @52;
    commIssue @53;
    tooDistracted @54;
    posenetInvalid @55;
    soundsUnavailable @56;
    preLaneChangeLeft @57;
    preLaneChangeRight @58;
    laneChange @59;
    communityFeatureDisallowed @62;
    lowMemory @63;
    stockAeb @64;
    ldw @65;
    carUnrecognized @66;
    invalidLkasSetting @69;
    speedTooHigh @70;
    laneChangeBlocked @71;
    relayMalfunction @72;
    gasPressed @73;
    stockFcw @74;
    startup @75;
    startupNoCar @76;
    startupNoControl @77;
    startupMaster @78;
    startupNoFw @85;
    fcw @79;
    steerSaturated @80;
    belowEngageSpeed @84;
    noGps @83;
    wrongCruiseMode @82;
    modeldLagging @81;
    deviceFalling @68;
    fanMalfunction @61;
    cameraMalfunction @67;
    gpsMalfunction @60;
    processNotRunning @49;
    dashcamMode @47;
    controlsInitializing @42;
    usbError @41;
    roadCameraError @40;
    driverCameraError @33;
    wideRoadCameraError @27;
    localizerMalfunction @16;
    highCpuUsage @15;
    cruiseMismatch @3;
  }
}

# ******* main car state @ 100hz *******
# all speeds in m/s

struct CarState {
  events @0 :List(CarEvent);

  # car speed
  vEgo @1 :Float32;         # best estimate of speed
  aEgo @2 :Float32;        # best estimate of acceleration
  vEgoRaw @3 :Float32;     # unfiltered speed from CAN sensors
  yawRate @4 :Float32;     # best estimate of yaw rate
  standstill @5 :Bool;
  wheelSpeeds @6 :WheelSpeeds;

  # gas pedal, 0.0-1.0
  gas @7 :Float32;        # this is user pedal only
  gasPressed @8 :Bool;    # this is user pedal only

  # brake pedal, 0.0-1.0
  brake @9 :Float32;      # this is user pedal only
  brakePressed @10 :Bool;  # this is user pedal only
  brakeHoldActive @11 :Bool;

  # steering wheel
  steeringAngleDeg @12 :Float32;
  steeringAngleOffsetDeg @13 :Float32; # Offset betweens sensors in case there multiple
  steeringRateDeg @14 :Float32;
  steeringTorque @15 :Float32;      # TODO: standardize units
  steeringTorqueEps @16 :Float32;  # TODO: standardize units
  steeringPressed @17 :Bool;        # if the user is using the steering wheel
  steeringRateLimited @18 :Bool;   # if the torque is limited by the rate limiter
  steerWarning @19 :Bool;          # temporary steer unavailble
  steerError @20 :Bool;            # permanent steer error
  stockAeb @21 :Bool;
  stockFcw @22 :Bool;
  espDisabled @23 :Bool;

  # cruise state
  cruiseState @24 :CruiseState;

  # gear
  gearShifter @25 :GearShifter;

  # button presses
  buttonEvents @26 :List(ButtonEvent);
  leftBlinker @27 :Bool;
  rightBlinker @28 :Bool;
  genericToggle @29 :Bool;

  # lock info
  doorOpen @30 :Bool;
  seatbeltUnlatched @31 :Bool;
  canValid @32 :Bool;

  # clutch (manual transmission only)
  clutchPressed @33 :Bool;

  # which packets this state came from
  canMonoTimes @34: List(UInt64);

  # blindspot sensors
  leftBlindspot @35 :Bool; # Is there something blocking the left lane change
  rightBlindspot @36 :Bool; # Is there something blocking the right lane change

  struct WheelSpeeds {
    # optional wheel speeds
    fl @0 :Float32;
    fr @1 :Float32;
    rl @2 :Float32;
    rr @3 :Float32;
  }

  struct CruiseState {
    enabled @0 :Bool;
    speed @1 :Float32;
    available @2 :Bool;
    speedOffset @3 :Float32;
    standstill @4 :Bool;
    nonAdaptive @5 :Bool;
  }

  enum GearShifter {
    unknown @0;
    park @1;
    drive @2;
    neutral @3;
    reverse @4;
    sport @5;
    low @6;
    brake @7;
    eco @8;
    manumatic @9;
  }

  # send on change
  struct ButtonEvent {
    pressed @0 :Bool;
    type @1 :Type;

    enum Type {
      unknown @0;
      leftBlinker @1;
      rightBlinker @2;
      accelCruise @3;
      decelCruise @4;
      cancel @5;
      altButton1 @6;
      altButton2 @7;
      altButton3 @8;
      setCruise @9;
      resumeCruise @10;
      gapAdjustCruise @11;
    }
  }
}

# ******* radar state @ 20hz *******

struct RadarData @0x888ad6581cf0aacb {
  errors @0 :List(Error);
  points @1 :List(RadarPoint);

  # which packets this state came from
  canMonoTimes @2 :List(UInt64);

  enum Error {
    canError @0;
    fault @1;
    wrongConfig @2;
  }

  # similar to LiveTracks
  # is one timestamp valid for all? I think so
  struct RadarPoint {
    trackId @0 :UInt64;  # no trackId reuse

    # these 3 are the minimum required
    dRel @1 :Float32; # m from the front bumper of the car
    yRel @2 :Float32; # m
    vRel @3 :Float32; # m/s

    # these are optional and valid if they are not NaN
    aRel @4 :Float32; # m/s^2
    yvRel @5 :Float32; # m/s

    # some radars flag measurements VS estimates
    measured @6 :Bool;
  }
}

# ******* car controls @ 100hz *******

struct CarControl {
  # must be true for any actuator commands to work
  enabled @0 :Bool;
  active @1 :Bool;

  actuators @2 :Actuators;
  roll @3 :Float32;
  pitch @4 :Float32;

  cruiseControl @5 :CruiseControl;
  hudControl @6 :HUDControl;

  struct Actuators {
    # range from 0.0 - 1.0
    # range from -1.0 - 1.0
    steer @0: Float32;
    steeringAngleDeg @1: Float32;

    accel @2: Float32; # m/s^2
    longControlState @3: LongControlState;

    enum LongControlState @0xe40f3a917d908282{
      off @0;
      pid @1;
      stopping @2;
      starting @3;
    }

  }

  struct CruiseControl {
    cancel @0: Bool;
    override @1: Bool;
    speedOverride @2: Float32;
    accelOverride @3: Float32;
  }

  struct HUDControl {
    speedVisible @0: Bool;
    setSpeed @1: Float32;
    lanesVisible @2: Bool;
    leadVisible @3: Bool;
    visualAlert @4: VisualAlert;
    audibleAlert @5: AudibleAlert;
    rightLaneVisible @6: Bool;
    leftLaneVisible @7: Bool;
    rightLaneDepart @8: Bool;
    leftLaneDepart @9: Bool;

    enum VisualAlert {
      # these are the choices from the Honda
      # map as good as you can for your car
      none @0;
      fcw @1;
      steerRequired @2;
      brakePressed @3;
      wrongGear @4;
      seatbeltUnbuckled @5;
      speedTooHigh @6;
      ldw @7;
    }

    enum AudibleAlert {
      none @0;

      engage @1;
      disengage @2;
      refuse @3;

      warningSoft @4;
      warningImmediate @5;

      prompt @6;
      promptRepeat @7;
      promptDistracted @8;
    }
  }
}

# ****** car param ******

struct CarParams {
  carName @0 :Text;
  carFingerprint @1 :Text;
  fuzzyFingerprint @2 :Bool;

  enableGasInterceptor @3 :Bool;
  pcmCruise @4 :Bool;        # is openpilot's state tied to the PCM's cruise state?
  enableDsu @5 :Bool;        # driving support unit
  enableApgs @6 :Bool;       # advanced parking guidance system
  enableBsm @7 :Bool;       # blind spot monitoring
  flags @8 :UInt32;         # flags for car specific quirks

  minEnableSpeed @9 :Float32;
  minSteerSpeed @10 :Float32;
  maxSteeringAngleDeg @11 :Float32;
  safetyConfigs @12 :List(SafetyConfig);

  steerMaxBP @13 :List(Float32);
  steerMaxV @14 :List(Float32);

  # things about the car in the manual
  mass @15 :Float32;            # [kg] curb weight: all fluids no cargo
  wheelbase @16 :Float32;       # [m] distance from rear axle to front axle
  centerToFront @17 :Float32;   # [m] distance from center of mass to front axle
  steerRatio @18 :Float32;      # [] ratio of steering wheel angle to front wheel angle
  steerRatioRear @19 :Float32;  # [] ratio of steering wheel angle to rear wheel angle (usually 0)

  # things we can derive
  rotationalInertia @20 :Float32;    # [kg*m2] body rotational inertia
  tireStiffnessFront @21 :Float32;   # [N/rad] front tire coeff of stiff
  tireStiffnessRear @22 :Float32;    # [N/rad] rear tire coeff of stiff

  longitudinalTuning @23 :LongitudinalPIDTuning;
  lateralParams @24 :LateralParams;
  lateralTuning :union {
    pid @25 :LateralPIDTuning;
    indi @26 :LateralINDITuning;
    lqr @27 :LateralLQRTuning;
  }

  steerLimitAlert @28 :Bool;
  steerLimitTimer @29 :Float32;  # time before steerLimitAlert is issued

  vEgoStopping @30 :Float32; # Speed at which the car goes into stopping state
  vEgoStarting @31 :Float32; # Speed at which the car goes into starting state
  directAccelControl @32 :Bool; # Does the car have direct accel control or just gas/brake
  stoppingControl @33 :Bool; # Does the car allows full control even at lows speeds when stopping
  startAccel @34 :Float32; # Required acceleraton to overcome creep braking
  stopAccel @35 :Float32; # Required acceleraton to keep vehicle stationary
  steerRateCost @36 :Float32; # Lateral MPC cost on steering rate
  steerControlType @37 :SteerControlType;
  radarOffCan @38 :Bool; # True when radar objects aren't visible on CAN
  minSpeedCan @39 :Float32; # Minimum vehicle speed from CAN (below this value drops to 0)
  stoppingDecelRate @40 :Float32; # m/s^2/s while trying to stop
  startingAccelRate @41 :Float32; # m/s^2/s while trying to start

  steerActuatorDelay @42 :Float32; # Steering wheel actuator delay in seconds
  longitudinalActuatorDelayLowerBound @43 :Float32; # Gas/Brake actuator delay in seconds, lower bound
  longitudinalActuatorDelayUpperBound @44 :Float32; # Gas/Brake actuator delay in seconds, upper bound
  openpilotLongitudinalControl @45 :Bool; # is openpilot doing the longitudinal control?
  carVin @46 :Text; # VIN number queried during fingerprinting
  dashcamOnly @47: Bool;
  transmissionType @48 :TransmissionType;
  carFw @49 :List(CarFw);

  radarTimeStep @50: Float32 = 0.05;  # time delta between radar updates, 20Hz is very standard
  communityFeature @51: Bool;  # true if a community maintained feature is detected
  fingerprintSource @52: FingerprintSource;
  networkLocation @53 :NetworkLocation;  # Where Panda/C2 is integrated into the car's CAN network

  wheelSpeedFactor @54 :Float32; # Multiplier on wheels speeds to computer actual speeds

  struct SafetyConfig {
    safetyModel @0 :SafetyModel;
    safetyParam @1 :Int16;
  }

  struct LateralParams {
    torqueBP @0 :List(Int32);
    torqueV @1 :List(Int32);
  }

  struct LateralPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @4 :Float32;
  }

  struct LongitudinalPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    deadzoneBP @4 :List(Float32);
    deadzoneV @5 :List(Float32);
  }

  struct LateralINDITuning {
    outerLoopGainBP @0 :List(Float32);
    outerLoopGainV @1 :List(Float32);
    innerLoopGainBP @2 :List(Float32);
    innerLoopGainV @3 :List(Float32);
    timeConstantBP @4 :List(Float32);
    timeConstantV @5 :List(Float32);
    actuatorEffectivenessBP @6 :List(Float32);
    actuatorEffectivenessV @7 :List(Float32);
  }

  struct LateralLQRTuning {
    scale @0 :Float32;
    ki @1 :Float32;
    dcGain @2 :Float32;

    # State space system
    a @3 :List(Float32);
    b @4 :List(Float32);
    c @5 :List(Float32);

    k @6 :List(Float32);  # LQR gain
    l @7 :List(Float32);  # Kalman gain
  }

  enum SafetyModel {
    silent @0;
    hondaNidec @1;
    toyota @2;
    elm327 @3;
    gm @4;
    hondaBoschGiraffe @5;
    ford @6;
    cadillac @7;
    hyundai @8;
    chrysler @9;
    tesla @10;
    subaru @11;
    gmPassive @12;
    mazda @13;
    nissan @14;
    volkswagen @15;
    toyotaIpas @16;
    allOutput @17;
    gmAscm @18;
    noOutput @19;  # like silent but without silent CAN TXs
    hondaBosch @20;
    volkswagenPq @21;
    subaruLegacy @22;  # pre-Global platform
    hyundaiLegacy @23;
    hyundaiCommunity @24;
    stellantis @25;
  }

  enum SteerControlType {
    torque @0;
    angle @1;
  }

  enum TransmissionType {
    unknown @0;
    automatic @1;  # Traditional auto, including DSG
    manual @2;  # True "stick shift" only
    direct @3;  # Electric vehicle or other direct drive
    cvt @4;
  }

  struct CarFw {
    ecu @0 :Ecu;
    fwVersion @1 :Data;
    address @2: UInt32;
    subAddress @3: UInt8;
  }

  enum Ecu {
    eps @0;
    esp @1;
    fwdRadar @2;
    fwdCamera @3;
    engine @4;
    unknown @5;
    transmission @8; # Transmission Control Module
    srs @9; # airbag
    gateway @10; # can gateway
    hud @11; # heads up display
    combinationMeter @12; # instrument cluster

    # Toyota only
    dsu @6;
    apgs @7;

    # Honda only
    vsa @13; # Vehicle Stability Assist
    programmedFuelInjection @14;
    electricBrakeBooster @15;
    shiftByWire @16;
  }

  enum FingerprintSource {
    can @0;
    fw @1;
    fixed @2;
  }

  enum NetworkLocation {
    fwdCamera @0;  # Standard/default integration at LKAS camera
    gateway @1;    # Integration at vehicle's CAN gateway
  }
}
