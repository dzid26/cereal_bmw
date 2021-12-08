using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Car = import "car.capnp";
using Legacy = import "legacy.capnp";

@0xf3b1f17e25a4285b;

const logVersion :Int32 = 1;

struct Map(Key, Value) {
  entries @0 :List(Entry);
  struct Entry {
    key @0 :Key;
    value @1 :Value;
  }
}

struct InitData {
  kernelArgs @0 :List(Text);
  kernelVersion @1 :Text;
  osVersion @2 :Text;

  gctx @3 :Text;
  dongleId @4 :Text;

  deviceType @5 :DeviceType;
  version @6 :Text;
  gitCommit @7 :Text;
  gitBranch @8 :Text;
  gitRemote @9 :Text;

  androidProperties @10 :Map(Text, Text);

  pandaInfo @11 :PandaInfo;

  dirty @12 :Bool;
  passive @13 :Bool;
  params @14 :Map(Text, Data);

  enum DeviceType {
    unknown @0;
    neo @1;
    chffrAndroid @2;
    chffrIos @3;
    tici @4;
    pc @5;
  }

  struct PandaInfo {
    hasPanda @0 :Bool;
    dongleId @1 :Text;
    stVersion @2 :Text;
    espVersion @3 :Text;
  }
}

struct FrameData {
  frameId @0 :UInt32;
  encodeId @1 :UInt32; # DEPRECATED

  frameType @2 :FrameType;
  frameLength @3 :Int32;

  # Timestamps
  timestampEof @4 :UInt64;
  timestampSof @5 :UInt64;

  # Exposure
  integLines @6 :Int32;
  highConversionGain @7 :Bool;
  gain @8 :Float32; # This includes highConversionGain if enabled
  measuredGreyFraction @9 :Float32;
  targetGreyFraction @10 :Float32;

  # Focus
  lensPos @11 :Int32;
  lensSag @12 :Float32;
  lensErr @13 :Float32;
  lensTruePos @14 :Float32;
  focusVal @15 :List(Int16);
  focusConf @16 :List(UInt8);
  sharpnessScore @17 :List(UInt16);
  recoverState @18 :Int32;

  transform @19 :List(Float32);

  androidCaptureResult @20 :AndroidCaptureResult;

  image @21 :Data;

  enum FrameType {
    unknown @0;
    neo @1;
    chffrAndroid @2;
    front @3;
  }

  struct AndroidCaptureResult {
    sensitivity @0 :Int32;
    frameDuration @1 :Int64;
    exposureTime @2 :Int64;
    rollingShutterSkew @3 :UInt64;
    colorCorrectionTransform @4 :List(Int32);
    colorCorrectionGains @5 :List(Float32);
    displayRotation @6 :Int8;
  }
}

struct Thumbnail {
  frameId @0 :UInt32;
  timestampEof @1 :UInt64;
  thumbnail @2 :Data;
}

struct GPSNMEAData {
  timestamp @0 :Int64;
  localWallTime @1 :UInt64;
  nmea @2 :Text;
}

# android sensor_event_t
struct SensorEventData {
  version @0 :Int32;
  sensor @1 :Int32;
  type @2 :Int32;
  timestamp @3 :Int64;

  union {
    acceleration @4 :SensorVec;
    magnetic @5 :SensorVec;
    orientation @6 :SensorVec;
    gyro @7 :SensorVec;
    pressure @8 :SensorVec;
    magneticUncalibrated @9 :SensorVec;
    gyroUncalibrated @10 :SensorVec;
    proximity @11: Float32;
    light @12: Float32;
    temperature @13: Float32;
  }
  source @14 :SensorSource;

  struct SensorVec {
    v @0 :List(Float32);
    status @1 :Int8;
  }

  enum SensorSource {
    android @0;
    iOS @1;
    fiber @2;
    velodyne @3;  # Velodyne IMU
    bno055 @4;    # Bosch accelerometer
    lsm6ds3 @5;   # accelerometer (c2)
    bmp280 @6;    # barometer (c2)
    mmc3416x @7;  # magnetometer (c2)
    bmx055 @8;
    rpr0521 @9;
    lsm6ds3trc @10;
    mmc5603nj @11;
  }
}

# android struct GpsLocation
struct GpsLocationData {
  # Contains GpsLocationFlags bits.
  flags @0 :UInt16;

  # Represents latitude in degrees.
  latitude @1 :Float64;

  # Represents longitude in degrees.
  longitude @2 :Float64;

  # Represents altitude in meters above the WGS 84 reference ellipsoid.
  altitude @3 :Float64;

  # Represents speed in meters per second.
  speed @4 :Float32;

  # Represents heading in degrees.
  bearingDeg @5 :Float32;

  # Represents expected accuracy in meters. (presumably 1 sigma?)
  accuracy @6 :Float32;

  # Timestamp for the location fix.
  # Milliseconds since January 1, 1970.
  timestamp @7 :Int64;

  source @8 :SensorSource;

  # Represents NED velocity in m/s.
  vNED @9 :List(Float32);

  # Represents expected vertical accuracy in meters. (presumably 1 sigma?)
  verticalAccuracy @10 :Float32;

  # Represents bearing accuracy in degrees. (presumably 1 sigma?)
  bearingAccuracyDeg @11 :Float32;

  # Represents velocity accuracy in m/s. (presumably 1 sigma?)
  speedAccuracy @12 :Float32;

  enum SensorSource {
    android @0;
    iOS @1;
    car @2;
    velodyne @3;  # Velodyne IMU
    fusion @4;
    external @5;
    ublox @6;
    trimble @7;
  }
}

struct CanData {
  address @0 :UInt32;
  busTime @1 :UInt16;
  dat     @2 :Data;
  src     @3 :UInt8;
}

struct DeviceState @0xa4d8b5af2aa492eb {
  usbOnline @0 :Bool;
  networkType @1 :NetworkType;
  networkInfo @2 :NetworkInfo;
  networkStrength @3 :NetworkStrength;
  lastAthenaPingTime @4 :UInt64;

  started @5 :Bool;
  startedMonoTime @6 :UInt64;

  # system utilization
  freeSpacePercent @7 :Float32;
  memoryUsagePercent @8 :Int8;
  gpuUsagePercent @9 :Int8;
  cpuUsagePercent @10 :List(Int8);  # per-core cpu usage

  # power
  batteryPercent @11 :Int16;
  batteryCurrent @12 :Int32;
  chargingError @13 :Bool;
  chargingDisabled @14 :Bool;
  offroadPowerUsageUwh @15 :UInt32;
  carBatteryCapacityUwh @16 :UInt32;
  powerDrawW @17 :Float32;

  # device thermals
  cpuTempC @18 :List(Float32);
  gpuTempC @19 :List(Float32);
  memoryTempC @20 :Float32;
  ambientTempC @21 :Float32;
  nvmeTempC @22 :List(Float32);
  modemTempC @23 :List(Float32);
  pmicTempC @24 :List(Float32);
  thermalZones @25 :List(ThermalZone);
  thermalStatus @26 :ThermalStatus;

  fanSpeedPercentDesired @27 :UInt16;
  screenBrightnessPercent @28 :Int8;

  struct ThermalZone {
    name @0 :Text;
    temp @1 :Float32;
  }

  enum ThermalStatus {
    green @0;
    yellow @1;
    red @2;
    danger @3;
  }

  enum NetworkType {
    none @0;
    wifi @1;
    cell2G @2;
    cell3G @3;
    cell4G @4;
    cell5G @5;
    ethernet @6;
  }

  enum NetworkStrength {
    unknown @0;
    poor @1;
    moderate @2;
    good @3;
    great @4;
  }

  struct NetworkInfo {
    technology @0 :Text;
    operator @1 :Text;
    band @2 :Text;
    channel @3 :UInt16;
    extra @4 :Text;
    state @5 :Text;
  }

  # deprecated
}

struct PandaState @0xa7649e2575e4591e {
  ignitionLine @0 :Bool;
  controlsAllowed @1 :Bool;
  gasInterceptorDetected @2 :Bool;
  canSendErrs @3 :UInt32;
  canFwdErrs @4 :UInt32;
  canRxErrs @5 :UInt32;
  gmlanSendErrs @6 :UInt32;
  pandaType @7 :PandaType;
  ignitionCan @8 :Bool;
  safetyModel @9 :Car.CarParams.SafetyModel;
  safetyParam @10 :Int16;
  faultStatus @11 :FaultStatus;
  powerSaveEnabled @12 :Bool;
  uptime @13 :UInt32;
  faults @14 :List(FaultType);
  harnessStatus @15 :HarnessStatus;
  heartbeatLost @16 :Bool;

  enum FaultStatus {
    none @0;
    faultTemp @1;
    faultPerm @2;
  }

  enum FaultType {
    relayMalfunction @0;
    unusedInterruptHandled @1;
    interruptRateCan1 @2;
    interruptRateCan2 @3;
    interruptRateCan3 @4;
    interruptRateTach @5;
    interruptRateGmlan @6;
    interruptRateInterrupts @7;
    interruptRateSpiDma @8;
    interruptRateSpiCs @9;
    interruptRateUart1 @10;
    interruptRateUart2 @11;
    interruptRateUart3 @12;
    interruptRateUart5 @13;
    interruptRateUartDma @14;
    interruptRateUsb @15;
    interruptRateTim1 @16;
    interruptRateTim3 @17;
    registerDivergent @18;
    interruptRateKlineInit @19;
    interruptRateClockSource @20;
    interruptRateTick @21;
    # Update max fault type in boardd when adding faults
  }

  enum PandaType @0x8a58adf93e5b3751 {
    unknown @0;
    whitePanda @1;
    greyPanda @2;
    blackPanda @3;
    pedal @4;
    uno @5;
    dos @6;
    redPanda @7;
  }

  enum HarnessStatus {
    notConnected @0;
    normal @1;
    flipped @2;
  }
}

struct PeripheralState {
  pandaType @0 :PandaState.PandaType;
  voltage @1 :UInt32;
  current @2 :UInt32;
  fanSpeedRpm @3 :UInt16;
  usbPowerMode @4 :UsbPowerMode;

  enum UsbPowerMode @0xa8883583b32c9877 {
    none @0;
    client @1;
    cdp @2;
    dcp @3;
  }
}

struct RadarState @0x9a185389d6fdd05f {
  canMonoTimes @0 :List(UInt64);
  mdMonoTime @1 :UInt64;
  carStateMonoTime @2 :UInt64;
  radarErrors @3 :List(Car.RadarData.Error);

  leadOne @4 :LeadData;
  leadTwo @5 :LeadData;
  cumLagMs @6 :Float32;

  struct LeadData {
    dRel @0 :Float32;
    yRel @1 :Float32;
    vRel @2 :Float32;
    aRel @3 :Float32;
    vLead @4 :Float32;
    dPath @5 :Float32;
    vLat @6 :Float32;
    vLeadK @7 :Float32;
    aLeadK @8 :Float32;
    fcw @9 :Bool;
    status @10 :Bool;
    aLeadTau @11 :Float32;
    modelProb @12 :Float32;
    radar @13 :Bool;
  }
}

struct LiveCalibrationData {
  calStatus @0 :Int8;
  calCycle @1 :Int32;
  calPerc @2 :Int8;
  validBlocks @3 :Int32;

  # view_frame_from_road_frame
  # ui's is inversed needs new
  extrinsicMatrix @4 :List(Float32);
  # the direction of travel vector in device frame
  rpyCalib @5 :List(Float32);
  rpyCalibSpread @6 :List(Float32);
}

struct LiveTracks {
  trackId @0 :Int32;
  dRel @1 :Float32;
  yRel @2 :Float32;
  vRel @3 :Float32;
  aRel @4 :Float32;
  timeStamp @5 :Float32;
  status @6 :Float32;
  currentTime @7 :Float32;
  stationary @8 :Bool;
  oncoming @9 :Bool;
}

struct ControlsState @0x97ff69c53601abf1 {
  startMonoTime @0 :UInt64;
  canMonoTimes @1 :List(UInt64);
  longitudinalPlanMonoTime @2 :UInt64;
  lateralPlanMonoTime @3 :UInt64;

  state @4 :OpenpilotState;
  enabled @5 :Bool;
  active @6 :Bool;

  longControlState @7 :Car.CarControl.Actuators.LongControlState;
  vPid @8 :Float32;
  vTargetLead @9 :Float32;
  vCruise @10 :Float32;
  upAccelCmd @11 :Float32;
  uiAccelCmd @12 :Float32;
  ufAccelCmd @13 :Float32;
  aTarget @14 :Float32;
  curvature @15 :Float32;  # path curvature from vehicle model
  forceDecel @16 :Bool;

  # UI alerts
  alertText1 @17 :Text;
  alertText2 @18 :Text;
  alertStatus @19 :AlertStatus;
  alertSize @20 :AlertSize;
  alertBlinkingRate @21 :Float32;
  alertType @22 :Text;
  alertSound @23 :Car.CarControl.HUDControl.AudibleAlert;
  engageable @24 :Bool;  # can OP be engaged?

  cumLagMs @25 :Float32;
  canErrorCounter @26 :UInt32;

  lateralControlState :union {
    indiState @27 :LateralINDIState;
    pidState @28 :LateralPIDState;
    lqrState @29 :LateralLQRState;
    angleState @30 :LateralAngleState;
    debugState @31 :LateralDebugState;
  }

  enum OpenpilotState @0xdbe58b96d2d1ac61 {
    disabled @0;
    preEnabled @1;
    enabled @2;
    softDisabling @3;
  }

  enum AlertStatus {
    normal @0;       # low priority alert for user's convenience
    userPrompt @1;   # mid priority alert that might require user intervention
    critical @2;     # high priority alert that needs immediate user intervention
  }

  enum AlertSize {
    none @0;    # don't display the alert
    small @1;   # small box
    mid @2;     # mid screen
    full @3;    # full screen
  }

  struct LateralINDIState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    steeringRateDeg @2 :Float32;
    steeringAccelDeg @3 :Float32;
    rateSetPoint @4 :Float32;
    accelSetPoint @5 :Float32;
    accelError @6 :Float32;
    delayedOutput @7 :Float32;
    delta @8 :Float32;
    output @9 :Float32;
    saturated @10 :Bool;
    steeringAngleDesiredDeg @11 :Float32;
    steeringRateDesiredDeg @12 :Float32;
  }

  struct LateralPIDState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    steeringRateDeg @2 :Float32;
    angleError @3 :Float32;
    p @4 :Float32;
    i @5 :Float32;
    f @6 :Float32;
    output @7 :Float32;
    saturated @8 :Bool;
    steeringAngleDesiredDeg @9 :Float32;
   }

  struct LateralLQRState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    i @2 :Float32;
    output @3 :Float32;
    lqrOutput @4 :Float32;
    saturated @5 :Bool;
    steeringAngleDesiredDeg @6 :Float32;
  }

  struct LateralAngleState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    output @2 :Float32;
    saturated @3 :Bool;
    steeringAngleDesiredDeg @4 :Float32;
  }

  struct LateralDebugState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    output @2 :Float32;
    saturated @3 :Bool;
  }
}

struct ModelDataV2 {
  frameId @0 :UInt32;
  frameAge @1 :UInt32;
  frameDropPerc @2 :Float32;
  timestampEof @3 :UInt64;
  modelExecutionTime @15 :Float32;
  gpuExecutionTime @17 :Float32;
  rawPredictions @16 :Data;

  # predicted future position, orientation, etc..
  position @4 :XYZTData;
  orientation @5 :XYZTData;
  velocity @6 :XYZTData;
  orientationRate @7 :XYZTData;
  acceleration @19 :XYZTData;

  # prediction lanelines and road edges
  laneLines @8 :List(XYZTData);
  laneLineProbs @9 :List(Float32);
  laneLineStds @13 :List(Float32);
  roadEdges @10 :List(XYZTData);
  roadEdgeStds @14 :List(Float32);

  # predicted lead cars
  leads @11 :List(LeadDataV2);
  leadsV3 @18 :List(LeadDataV3);

  meta @12 :MetaData;

  # All SI units and in device frame
  struct XYZTData {
    x @0 :List(Float32);
    y @1 :List(Float32);
    z @2 :List(Float32);
    t @3 :List(Float32);
    xStd @4 :List(Float32);
    yStd @5 :List(Float32);
    zStd @6 :List(Float32);
  }

  struct LeadDataV2 {
    prob @0 :Float32; # probability that car is your lead at time t
    t @1 :Float32;

    # x and y are relative position in device frame
    # v is norm relative speed
    # a is norm relative acceleration
    xyva @2 :List(Float32);
    xyvaStd @3 :List(Float32);
  }

  struct LeadDataV3 {
    prob @0 :Float32; # probability that car is your lead at time t
    probTime @1 :Float32;
    t @2 :List(Float32);

    # x and y are relative position in device frame
    # v absolute norm speed
    # a is derivative of v
    x @3 :List(Float32);
    xStd @4 :List(Float32);
    y @5 :List(Float32);
    yStd @6 :List(Float32);
    v @7 :List(Float32);
    vStd @8 :List(Float32);
    a @9 :List(Float32);
    aStd @10 :List(Float32);
  }


  struct MetaData {
    engagedProb @0 :Float32;
    desirePrediction @1 :List(Float32);
    desireState @2 :List(Float32);
    disengagePredictions @3 :DisengagePredictions;
    hardBrakePredicted @4 :Bool;
  }

  struct DisengagePredictions {
    t @0 :List(Float32);
    brakeDisengageProbs @1 :List(Float32);
    gasDisengageProbs @2 :List(Float32);
    steerOverrideProbs @3 :List(Float32);
    brake3MetersPerSecondSquaredProbs @4 :List(Float32);
    brake4MetersPerSecondSquaredProbs @5 :List(Float32);
    brake5MetersPerSecondSquaredProbs @6 :List(Float32);
  }
}

struct EncodeIndex {
  # picture from camera
  frameId @0 :UInt32;
  type @1 :Type;
  # index of encoder from start of route
  encodeId @2 :UInt32;
  # minute long segment this frame is in
  segmentNum @3 :Int32;
  # index into camera file in segment in presentation order
  segmentId @4 :UInt32;
  # index into camera file in segment in encode order
  segmentIdEncode @5 :UInt32;
  timestampSof @6 :UInt64;
  timestampEof @7 :UInt64;

  enum Type {
    bigBoxLossless @0;   # rcamera.mkv
    fullHEVC @1;         # fcamera.hevc
    bigBoxHEVC @2;       # bcamera.hevc
    chffrAndroidH264 @3; # acamera
    fullLosslessClip @4; # prcamera.mkv
    front @5;            # dcamera.hevc
  }
}

struct AndroidLogEntry {
  id @0 :UInt8;
  ts @1 :UInt64;
  priority @2 :UInt8;
  pid @3 :Int32;
  tid @4 :Int32;
  tag @5 :Text;
  message @6 :Text;
}

struct LongitudinalPlan @0xe00b5b3eba12876c {
  modelMonoTime @0 :UInt64;
  hasLead @1 :Bool;
  fcw @2 :Bool;
  longitudinalPlanSource @3 :LongitudinalPlanSource;
  processingDelay @4 :Float32;

  # desired speed/accel/jerk over next 2.5s
  accels @5 :List(Float32);
  speeds @6 :List(Float32);
  jerks @7 :List(Float32);

  enum LongitudinalPlanSource {
    cruise @0;
    lead0 @1;
    lead1 @2;
    lead2 @3;
    e2e @4;
  }

  struct GpsTrajectory {
    x @0 :List(Float32);
    y @1 :List(Float32);
  }
}

struct LateralPlan @0xe1e9318e2ae8b51e {
  laneWidth @0 :Float32;
  lProb @1 :Float32;
  rProb @2 :Float32;
  dPathPoints @3 :List(Float32);
  dProb @4 :Float32;

  mpcSolutionValid @5 :Bool;
  desire @6 :Desire;
  laneChangeState @7 :LaneChangeState;
  laneChangeDirection @8 :LaneChangeDirection;
  useLaneLines @9 :Bool;

  # desired curvatures over next 2.5s in rad/m
  psis @10 :List(Float32);
  curvatures @11 :List(Float32);
  curvatureRates @12 :List(Float32);

  enum Desire {
    none @0;
    turnLeft @1;
    turnRight @2;
    laneChangeLeft @3;
    laneChangeRight @4;
    keepLeft @5;
    keepRight @6;
  }

  enum LaneChangeState {
    off @0;
    preLaneChange @1;
    laneChangeStarting @2;
    laneChangeFinishing @3;
  }

  enum LaneChangeDirection {
    none @0;
    left @1;
    right @2;
  }
}

struct LiveLocationKalman {

  # More info on reference frames:
  # https://github.com/commaai/openpilot/tree/master/common/transformations

  positionECEF @0 : Measurement;
  positionGeodetic @1 : Measurement;
  velocityECEF @2 : Measurement;
  velocityNED @3 : Measurement;
  velocityDevice @4 : Measurement;
  accelerationDevice @5: Measurement;


  # These angles are all eulers and roll, pitch, yaw
  # orientationECEF transforms to rot matrix: ecef_from_device
  orientationECEF @6 : Measurement;
  calibratedOrientationECEF @20 : Measurement;
  orientationNED @7 : Measurement;
  angularVelocityDevice @8 : Measurement;

  # orientationNEDCalibrated transforms to rot matrix: NED_from_calibrated
  calibratedOrientationNED @9 : Measurement;

  # Calibrated frame is simply device frame
  # aligned with the vehicle
  velocityCalibrated @10 : Measurement;
  accelerationCalibrated @11 : Measurement;
  angularVelocityCalibrated @12 : Measurement;

  gpsWeek @13 :Int32;
  gpsTimeOfWeek @14 :Float64;
  status @15 :Status;
  unixTimestampMillis @16 :Int64;
  inputsOK @17 :Bool = true;
  posenetOK @18 :Bool = true;
  gpsOK @19 :Bool = true;
  sensorsOK @21 :Bool = true;
  deviceStable @22 :Bool = true;
  timeSinceReset @23 :Float64;
  excessiveResets @24 :Bool;

  enum Status {
    uninitialized @0;
    uncalibrated @1;
    valid @2;
  }

  struct Measurement {
    value @0 : List(Float64);
    std @1 : List(Float64);
    valid @2 : Bool;
  }
}

struct ProcLog {
  cpuTimes @0 :List(CPUTimes);
  mem @1 :Mem;
  procs @2 :List(Process);

  struct Process {
    pid @0 :Int32;
    name @1 :Text;
    state @2 :UInt8;
    ppid @3 :Int32;

    cpuUser @4 :Float32;
    cpuSystem @5 :Float32;
    cpuChildrenUser @6 :Float32;
    cpuChildrenSystem @7 :Float32;
    priority @8 :Int64;
    nice @9 :Int32;
    numThreads @10 :Int32;
    startTime @11 :Float64;

    memVms @12 :UInt64;
    memRss @13 :UInt64;

    processor @14 :Int32;

    cmdline @15 :List(Text);
    exe @16 :Text;
  }

  struct CPUTimes {
    cpuNum @0 :Int64;
    user @1 :Float32;
    nice @2 :Float32;
    system @3 :Float32;
    idle @4 :Float32;
    iowait @5 :Float32;
    irq @6 :Float32;
    softirq @7 :Float32;
  }

  struct Mem {
    total @0 :UInt64;
    free @1 :UInt64;
    available @2 :UInt64;
    buffers @3 :UInt64;
    cached @4 :UInt64;
    active @5 :UInt64;
    inactive @6 :UInt64;
    shared @7 :UInt64;
  }
}

struct UbloxGnss {
  union {
    measurementReport @0 :MeasurementReport;
    ephemeris @1 :Ephemeris;
    ionoData @2 :IonoData;
    hwStatus @3 :HwStatus;
    hwStatus2 @4 :HwStatus2;
  }

  struct MeasurementReport {
    #received time of week in gps time in seconds and gps week
    rcvTow @0 :Float64;
    gpsWeek @1 :UInt16;
    # leap seconds in seconds
    leapSeconds @2 :UInt16;
    # receiver status
    receiverStatus @3 :ReceiverStatus;
    # num of measurements to follow
    numMeas @4 :UInt8;
    measurements @5 :List(Measurement);

    struct ReceiverStatus {
      # leap seconds have been determined
      leapSecValid @0 :Bool;
      # Clock reset applied
      clkReset @1 :Bool;
    }

    struct Measurement {
      svId @0 :UInt8;
      trackingStatus @1 :TrackingStatus;
      # pseudorange in meters
      pseudorange @2 :Float64;
      # carrier phase measurement in cycles
      carrierCycles @3 :Float64;
      # doppler measurement in Hz
      doppler @4 :Float32;
      # GNSS id, 0 is gps
      gnssId @5 :UInt8;
      glonassFrequencyIndex @6 :UInt8;
      # carrier phase locktime counter in ms
      locktime @7 :UInt16;
      # Carrier-to-noise density ratio (signal strength) in dBHz
      cno @8 :UInt8;
      # pseudorange standard deviation in meters
      pseudorangeStdev @9 :Float32;
      # carrier phase standard deviation in cycles
      carrierPhaseStdev @10 :Float32;
      # doppler standard deviation in Hz
      dopplerStdev @11 :Float32;
      sigId @12 :UInt8;

      struct TrackingStatus {
        # pseudorange valid
        pseudorangeValid @0 :Bool;
        # carrier phase valid
        carrierPhaseValid @1 :Bool;
        # half cycle valid
        halfCycleValid @2 :Bool;
        # half sycle subtracted from phase
        halfCycleSubtracted @3 :Bool;
      }
    }
  }

  struct Ephemeris {
    # This is according to the rinex (2?) format
    svId @0 :UInt16;
    year @1 :UInt16;
    month @2 :UInt16;
    day @3 :UInt16;
    hour @4 :UInt16;
    minute @5 :UInt16;
    second @6 :Float32;
    af0 @7 :Float64;
    af1 @8 :Float64;
    af2 @9 :Float64;

    iode @10 :Float64;
    crs @11 :Float64;
    deltaN @12 :Float64;
    m0 @13 :Float64;

    cuc @14 :Float64;
    ecc @15 :Float64;
    cus @16 :Float64;
    a @17 :Float64; # note that this is not the root!!

    toe @18 :Float64;
    cic @19 :Float64;
    omega0 @20 :Float64;
    cis @21 :Float64;

    i0 @22 :Float64;
    crc @23 :Float64;
    omega @24 :Float64;
    omegaDot @25 :Float64;

    iDot @26 :Float64;
    codesL2 @27 :Float64;
    gpsWeek @28 :Float64;
    l2 @29 :Float64;

    svAcc @30 :Float64;
    svHealth @31 :Float64;
    tgd @32 :Float64;
    iodc @33 :Float64;

    transmissionTime @34 :Float64;
    fitInterval @35 :Float64;

    toc @36 :Float64;

    ionoCoeffsValid @37 :Bool;
    ionoAlpha @38 :List(Float64);
    ionoBeta @39 :List(Float64);

  }

  struct IonoData {
    svHealth @0 :UInt32;
    tow  @1 :Float64;
    gpsWeek @2 :Float64;

    ionoAlpha @3 :List(Float64);
    ionoBeta @4 :List(Float64);

    healthValid @5 :Bool;
    ionoCoeffsValid @6 :Bool;
  }

  struct HwStatus {
    noisePerMS @0 :UInt16;
    agcCnt @1 :UInt16;
    aStatus @2 :AntennaSupervisorState;
    aPower @3 :AntennaPowerStatus;
    jamInd @4 :UInt8;
    flags @5 :UInt8;

    enum AntennaSupervisorState {
      init @0;
      dontknow @1;
      ok @2;
      short @3;
      open @4;
    }

    enum AntennaPowerStatus {
      off @0;
      on @1;
      dontknow @2;
    }
  }

  struct HwStatus2 {
    ofsI @0 :Int8;
    magI @1 :UInt8;
    ofsQ @2 :Int8;
    magQ @3 :UInt8;
    cfgSource @4 :ConfigSource;
    lowLevCfg @5 :UInt32;
    postStatus @6 :UInt32;

    enum ConfigSource {
      undefined @0;
      rom @1;
      otp @2;
      configpins @3;
      flash @4;
    }
  }
}

struct Clocks {
  bootTimeNanos @0 :UInt64;
  monotonicNanos @1 :UInt64;
  monotonicRawNanos @2 :UInt64;
  wallTimeNanos @3 :UInt64;
  modemUptimeMillis @4 :UInt64;
}

struct LiveMpcData {
  x @0 :List(Float32);
  y @1 :List(Float32);
  psi @2 :List(Float32);
  curvature @3 :List(Float32);
  qpIterations @4 :UInt32;
  calculationTime @5 :UInt64;
  cost @6 :Float64;
}

struct LiveLongitudinalMpcData {
  xEgo @0 :List(Float32);
  vEgo @1 :List(Float32);
  aEgo @2 :List(Float32);
  xLead @3 :List(Float32);
  vLead @4 :List(Float32);
  aLead @5 :List(Float32);
  aLeadTau @6 :Float32;    # lead accel time constant
  qpIterations @7 :UInt32;
  mpcId @8 :UInt32;
  calculationTime @9 :UInt64;
  cost @10 :Float64;
}

struct Joystick {
  # convenient for debug and live tuning
  axes @0: List(Float32);
  buttons @1: List(Bool);
}

struct DriverState {
  frameId @0 :UInt32;
  modelExecutionTime @1 :Float32;
  dspExecutionTime @2 :Float32;
  rawPredictions @3 :Data;

  faceOrientation @4 :List(Float32);
  facePosition @5 :List(Float32);
  faceProb @6 :Float32;
  leftEyeProb @7 :Float32;
  rightEyeProb @8 :Float32;
  leftBlinkProb @9 :Float32;
  rightBlinkProb @10 :Float32;
  faceOrientationStd @11 :List(Float32);
  facePositionStd @12 :List(Float32);
  sunglassesProb @13 :Float32;
  poorVision @14 :Float32;
  partialFace @15 :Float32;
  distractedPose @16 :Float32;
  distractedEyes @17 :Float32;
  eyesOnRoad @18 :Float32;
  phoneUse @19 :Float32;
  occludedProb @20 :Float32;
}

struct DriverMonitoringState @0xb83cda094a1da284 {
  events @0 :List(Car.CarEvent);
  faceDetected @1 :Bool;
  isDistracted @2 :Bool;
  awarenessStatus @3 :Float32;
  posePitchOffset @4 :Float32;
  posePitchValidCount @5 :UInt32;
  poseYawOffset @6 :Float32;
  poseYawValidCount @7 :UInt32;
  stepChange @8 :Float32;
  awarenessActive @9 :Float32;
  awarenessPassive @10 :Float32;
  isLowStd @11 :Bool;
  hiStdCount @12 :UInt32;
  isActiveMode @13 :Bool;
}

struct Boot {
  wallTimeNanos @0 :UInt64;
  pstore @1 :Map(Text, Data);
  commands @2 :Map(Text, Data);
  launchLog @3 :Text;
}

struct LiveParametersData {
  valid @0 :Bool;
  gyroBias @1 :Float32;
  angleOffsetDeg @2 :Float32;
  angleOffsetAverageDeg @3 :Float32;
  stiffnessFactor @4 :Float32;
  steerRatio @5 :Float32;
  sensorValid @6 :Bool;
  yawRate @7 :Float32;
  posenetSpeed @8 :Float32;
  posenetValid @9 :Bool;
  angleOffsetFastStd @10 :Float32;
  angleOffsetAverageStd @11 :Float32;
  stiffnessFactorStd @12 :Float32;
  steerRatioStd @13 :Float32;
}

struct CameraOdometry {
  frameId @4 :UInt32;
  timestampEof @5 :UInt64;
  trans @0 :List(Float32); # m/s in device frame
  rot @1 :List(Float32); # rad/s in device frame
  transStd @2 :List(Float32); # std m/s in device frame
  rotStd @3 :List(Float32); # std rad/s in device frame
}

struct Sentinel {
  enum SentinelType {
    endOfSegment @0;
    endOfRoute @1;
    startOfSegment @2;
    startOfRoute @3;
  }
  type @0 :SentinelType;
  signal @1 :Int32;
}

struct ManagerState {
  processes @0 :List(ProcessState);

  struct ProcessState {
    name @0 :Text;
    pid @1 :Int32;
    running @2 :Bool;
    shouldBeRunning @4 :Bool;
    exitCode @3 :Int32;
  }
}

struct UploaderState {
  immediateQueueSize @0 :UInt32;
  immediateQueueCount @1 :UInt32;
  rawQueueSize @2 :UInt32;
  rawQueueCount @3 :UInt32;

  # stats for last successfully uploaded file
  lastTime @4 :Float32;  # s
  lastSpeed @5 :Float32; # MB/s
  lastFilename @6 :Text;
}

struct NavInstruction {
  maneuverPrimaryText @0 :Text;
  maneuverSecondaryText @1 :Text;
  maneuverDistance @2 :Float32;  # m
  maneuverType @3 :Text; # TODO: Make Enum
  maneuverModifier @4 :Text; # TODO: Make Enum

  distanceRemaining @5 :Float32; # m
  timeRemaining @6 :Float32; # s
  timeRemainingTypical @7 :Float32; # s

  lanes @8 :List(Lane);
  showFull @9 :Bool;

  struct Lane {
    directions @0 :List(Direction);
    active @1 :Bool;
    activeDirection @2 :Direction;
  }

  enum Direction {
    none @0;
    left @1;
    right @2;
    straight @3;
  }

}

struct NavRoute {
  coordinates @0 :List(Coordinate);

  struct Coordinate {
    latitude @0 :Float32;
    longitude @1 :Float32;
  }
}

struct Event {
  logMonoTime @0 :UInt64;  # nanoseconds
  valid @1 :Bool = true;

  union {
    # *********** log metadata ***********
    initData @2 :InitData;
    sentinel @3 :Sentinel;

    # *********** bootlog ***********
    boot @4 :Boot;

    # ********** openpilot daemon msgs **********
    gpsNMEA @5 :GPSNMEAData;
    can @6 :List(CanData);
    controlsState @7 :ControlsState;
    sensorEvents @8 :List(SensorEventData);
    pandaStates @9 :List(PandaState);
    peripheralState @10 :PeripheralState;
    radarState @11 :RadarState;
    liveTracks @12 :List(LiveTracks);
    sendcan @13 :List(CanData);
    liveCalibration @14 :LiveCalibrationData;
    carState @15 :Car.CarState;
    carControl @16 :Car.CarControl;
    longitudinalPlan @17 :LongitudinalPlan;
    lateralPlan @18 :LateralPlan;
    ubloxGnss @19 :UbloxGnss;
    ubloxRaw @20 :Data;
    gpsLocationExternal @21 :GpsLocationData;
    driverState @22 :DriverState;
    liveParameters @23 :LiveParametersData;
    cameraOdometry @24 :CameraOdometry;
    thumbnail @25: Thumbnail;
    carEvents @26: List(Car.CarEvent);
    carParams @27: Car.CarParams;
    driverMonitoringState @28: DriverMonitoringState;
    liveLocationKalman @29 :LiveLocationKalman;
    modelV2 @30 :ModelDataV2;

    # camera stuff, each camera state has a matching encode idx
    roadCameraState @31 :FrameData;
    driverCameraState @32: FrameData;
    wideRoadCameraState @33: FrameData;
    roadEncodeIdx @34 :EncodeIndex;
    driverEncodeIdx @35 :EncodeIndex;
    wideRoadEncodeIdx @36 :EncodeIndex;

    # systems stuff
    androidLog @37 :AndroidLogEntry;
    managerState @38 :ManagerState;
    uploaderState @39 :UploaderState;
    procLog @40 :ProcLog;
    clocks @41 :Clocks;
    deviceState @42 :DeviceState;
    logMessage @43 :Text;

    # navigation
    navInstruction @44 :NavInstruction;
    navRoute @45 :NavRoute;
    navThumbnail @46: Thumbnail;

    # *********** debug ***********
    testJoystick @47 :Joystick;
  }
}
