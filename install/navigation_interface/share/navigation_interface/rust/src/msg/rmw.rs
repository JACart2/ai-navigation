#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__LatLongPoint() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__LatLongPoint__init(msg: *mut LatLongPoint) -> bool;
    fn navigation_interface__msg__LatLongPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LatLongPoint>, size: usize) -> bool;
    fn navigation_interface__msg__LatLongPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LatLongPoint>);
    fn navigation_interface__msg__LatLongPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LatLongPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<LatLongPoint>) -> bool;
}

// Corresponds to navigation_interface__msg__LatLongPoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LatLongPoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub latitude: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub longitude: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub elevation: f64,

}



impl Default for LatLongPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__LatLongPoint__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__LatLongPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LatLongPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LatLongPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LatLongPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LatLongPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LatLongPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LatLongPoint where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/LatLongPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__LatLongPoint() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__LatLongArray() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__LatLongArray__init(msg: *mut LatLongArray) -> bool;
    fn navigation_interface__msg__LatLongArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LatLongArray>, size: usize) -> bool;
    fn navigation_interface__msg__LatLongArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LatLongArray>);
    fn navigation_interface__msg__LatLongArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LatLongArray>, out_seq: *mut rosidl_runtime_rs::Sequence<LatLongArray>) -> bool;
}

// Corresponds to navigation_interface__msg__LatLongArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LatLongArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gpspoints: rosidl_runtime_rs::Sequence<super::super::msg::rmw::LatLongPoint>,

}



impl Default for LatLongArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__LatLongArray__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__LatLongArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LatLongArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LatLongArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LatLongArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LatLongArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LatLongArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LatLongArray where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/LatLongArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__LatLongArray() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__LocalPointsArray() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__LocalPointsArray__init(msg: *mut LocalPointsArray) -> bool;
    fn navigation_interface__msg__LocalPointsArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LocalPointsArray>, size: usize) -> bool;
    fn navigation_interface__msg__LocalPointsArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LocalPointsArray>);
    fn navigation_interface__msg__LocalPointsArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LocalPointsArray>, out_seq: *mut rosidl_runtime_rs::Sequence<LocalPointsArray>) -> bool;
}

// Corresponds to navigation_interface__msg__LocalPointsArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LocalPointsArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub localpoints: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub total_distance: std_msgs::msg::rmw::Float64,

}



impl Default for LocalPointsArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__LocalPointsArray__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__LocalPointsArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LocalPointsArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LocalPointsArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LocalPointsArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__LocalPointsArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LocalPointsArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LocalPointsArray where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/LocalPointsArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__LocalPointsArray() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__VehicleState() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__VehicleState__init(msg: *mut VehicleState) -> bool;
    fn navigation_interface__msg__VehicleState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleState>, size: usize) -> bool;
    fn navigation_interface__msg__VehicleState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleState>);
    fn navigation_interface__msg__VehicleState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleState>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleState>) -> bool;
}

// Corresponds to navigation_interface__msg__VehicleState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_navigating: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reached_destination: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stopped: bool,

}



impl Default for VehicleState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__VehicleState__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__VehicleState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__VehicleState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__VehicleState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__VehicleState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleState where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/VehicleState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__VehicleState() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__Stop() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__Stop__init(msg: *mut Stop) -> bool;
    fn navigation_interface__msg__Stop__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Stop>, size: usize) -> bool;
    fn navigation_interface__msg__Stop__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Stop>);
    fn navigation_interface__msg__Stop__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Stop>, out_seq: *mut rosidl_runtime_rs::Sequence<Stop>) -> bool;
}

// Corresponds to navigation_interface__msg__Stop
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Stop {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stop: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sender_id: std_msgs::msg::rmw::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f64,

}



impl Default for Stop {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__Stop__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__Stop__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Stop {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__Stop__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__Stop__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__Stop__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Stop {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Stop where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/Stop";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__Stop() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__WaypointsArray() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__WaypointsArray__init(msg: *mut WaypointsArray) -> bool;
    fn navigation_interface__msg__WaypointsArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<WaypointsArray>, size: usize) -> bool;
    fn navigation_interface__msg__WaypointsArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<WaypointsArray>);
    fn navigation_interface__msg__WaypointsArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<WaypointsArray>, out_seq: *mut rosidl_runtime_rs::Sequence<WaypointsArray>) -> bool;
}

// Corresponds to navigation_interface__msg__WaypointsArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WaypointsArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub waypoints: rosidl_runtime_rs::Sequence<sensor_msgs::msg::rmw::NavSatFix>,

}



impl Default for WaypointsArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__WaypointsArray__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__WaypointsArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for WaypointsArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__WaypointsArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__WaypointsArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__WaypointsArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for WaypointsArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for WaypointsArray where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/WaypointsArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__WaypointsArray() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__Obstacle() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__Obstacle__init(msg: *mut Obstacle) -> bool;
    fn navigation_interface__msg__Obstacle__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Obstacle>, size: usize) -> bool;
    fn navigation_interface__msg__Obstacle__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Obstacle>);
    fn navigation_interface__msg__Obstacle__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Obstacle>, out_seq: *mut rosidl_runtime_rs::Sequence<Obstacle>) -> bool;
}

// Corresponds to navigation_interface__msg__Obstacle
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Obstacle {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos: geometry_msgs::msg::rmw::PointStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub radius: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub followable: bool,

}



impl Default for Obstacle {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__Obstacle__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__Obstacle__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Obstacle {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__Obstacle__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__Obstacle__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__Obstacle__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Obstacle {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Obstacle where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/Obstacle";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__Obstacle() }
  }
}


#[link(name = "navigation_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__ObstacleArray() -> *const std::ffi::c_void;
}

#[link(name = "navigation_interface__rosidl_generator_c")]
extern "C" {
    fn navigation_interface__msg__ObstacleArray__init(msg: *mut ObstacleArray) -> bool;
    fn navigation_interface__msg__ObstacleArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ObstacleArray>, size: usize) -> bool;
    fn navigation_interface__msg__ObstacleArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ObstacleArray>);
    fn navigation_interface__msg__ObstacleArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ObstacleArray>, out_seq: *mut rosidl_runtime_rs::Sequence<ObstacleArray>) -> bool;
}

// Corresponds to navigation_interface__msg__ObstacleArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub obstacles: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Obstacle>,

}



impl Default for ObstacleArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_interface__msg__ObstacleArray__init(&mut msg as *mut _) {
        panic!("Call to navigation_interface__msg__ObstacleArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ObstacleArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__ObstacleArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__ObstacleArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_interface__msg__ObstacleArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ObstacleArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ObstacleArray where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_interface/msg/ObstacleArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_interface__msg__ObstacleArray() }
  }
}


