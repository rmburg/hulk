use std::ops::{Deref, DerefMut};

use crate::{
    impl_scope,
    joints::{Joints, Scope},
    joints_types,
};

impl_scope!(FullBody, 22);
impl_scope!(Head, 2);
impl_scope!(Arm, 4);
impl_scope!(Leg, 6);
impl_scope!(UpperBody, 8);
impl_scope!(LowerBody, 12);

joints_types!(Head, HeadJoints, BorrowedHeadJoints, BorrowedHeadJointsMut);
joints_types!(Arm, ArmJoints, BorrowedArmJoints, BorrowedArmJointsMut);
joints_types!(Leg, LegJoints, BorrowedLegJoints, BorrowedLegJointsMut);
joints_types!(
    UpperBody,
    UpperBodyJoints,
    BorrowedUpperBodyJoints,
    BorrowedUpperBodyJointsMut
);
joints_types!(
    LowerBody,
    LowerBodyJoints,
    BorrowedLowerBodyJoints,
    BorrowedLowerBodyJointsMut
);

pub struct JointsParts<T> {
    pub head: Joints<T, Head>,
    pub left_arm: Joints<T, Arm>,
    pub right_arm: Joints<T, Arm>,
    pub left_leg: Joints<T, Leg>,
    pub right_leg: Joints<T, Leg>,
}

impl<T, const N: usize> Deref for Joints<T, FullBody, [T; N]> {
    type Target = JointsParts<T>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        unsafe { &*(self.storage.as_ptr() as *const Self::Target) }
    }
}

impl<T, const N: usize> DerefMut for Joints<T, FullBody, [T; N]> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *(self.storage.as_mut_ptr() as *mut Self::Target) }
    }
}

macro_rules! joint_parts {
    ($name:ident, $scope:ident, [$($field:ident),+]) => {
        #[repr(C)]
        pub struct $name<T> {
            $(
                pub $field: T,
            )+
        }

        impl<T, const N: usize> Deref for Joints<T, $scope, [T; N]> {
            type Target = $name<T>;

            #[inline]
            fn deref(&self) -> &Self::Target {
                unsafe { &*(self.storage.as_ptr() as *const Self::Target) }
            }
        }

        impl<T, const N: usize> DerefMut for Joints<T, $scope, [T; N]> {
            #[inline]
            fn deref_mut(&mut self) -> &mut Self::Target {
                unsafe { &mut *(self.storage.as_mut_ptr() as *mut Self::Target) }
            }
        }

        impl<T, const N: usize> Deref for Joints<T, $scope, &'_ [T; N]> {
            type Target = $name<T>;

            #[inline]
            fn deref(&self) -> &Self::Target {
                unsafe { &*(self.storage.as_ptr() as *const Self::Target) }
            }
        }

        impl<T, const N: usize> Deref for Joints<T, $scope, &'_ mut [T; N]> {
            type Target = $name<T>;

            #[inline]
            fn deref(&self) -> &Self::Target {
                unsafe { &*(self.storage.as_ptr() as *const Self::Target) }
            }
        }

        impl<T, const N: usize> DerefMut for Joints<T, $scope, &'_ mut [T; N]> {
            #[inline]
            fn deref_mut(&mut self) -> &mut Self::Target {
                unsafe { &mut *(self.storage.as_mut_ptr() as *mut Self::Target) }
            }
        }
    };
}

joint_parts!(HeadParts, Head, [yaw, pitch]);
joint_parts!(
    ArmParts,
    Arm,
    [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow]
);
joint_parts!(
    LegParts,
    Leg,
    [hip_pitch, hip_roll, hip_yaw, knee, ankle_up, ankle_down]
);

// convenience methods

impl<T> HeadJoints<T> {
    pub fn from_yaw_and_pitch(yaw: T, pitch: T) -> Self {
        Self::from_array([yaw, pitch])
    }
}
