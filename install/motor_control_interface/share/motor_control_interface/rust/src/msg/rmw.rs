#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "motor_control_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__motor_control_interface__msg__VelAngle() -> *const std::ffi::c_void;
}

#[link(name = "motor_control_interface__rosidl_generator_c")]
extern "C" {
    fn motor_control_interface__msg__VelAngle__init(msg: *mut VelAngle) -> bool;
    fn motor_control_interface__msg__VelAngle__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelAngle>, size: usize) -> bool;
    fn motor_control_interface__msg__VelAngle__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelAngle>);
    fn motor_control_interface__msg__VelAngle__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelAngle>, out_seq: *mut rosidl_runtime_rs::Sequence<VelAngle>) -> bool;
}

// Corresponds to motor_control_interface__msg__VelAngle
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelAngle {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vel: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angle: f64,

}



impl Default for VelAngle {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !motor_control_interface__msg__VelAngle__init(&mut msg as *mut _) {
        panic!("Call to motor_control_interface__msg__VelAngle__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelAngle {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_control_interface__msg__VelAngle__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_control_interface__msg__VelAngle__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_control_interface__msg__VelAngle__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelAngle {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelAngle where Self: Sized {
  const TYPE_NAME: &'static str = "motor_control_interface/msg/VelAngle";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__motor_control_interface__msg__VelAngle() }
  }
}


