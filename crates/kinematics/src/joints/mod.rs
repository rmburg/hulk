pub mod k1;

use std::{
    clone::Clone,
    marker::PhantomData,
    ops::{Add, Index, Mul, Sub},
};

use serde::{Deserialize, Serialize};

pub trait JointsStorage<T>: Index<usize, Output = T> + IntoIterator<Item = T> + AsRef<[T]> {
    fn from_fn(f: impl FnMut(usize) -> T) -> Self;
}

impl<T, const N: usize> JointsStorage<T> for [T; N] {
    fn from_fn(f: impl FnMut(usize) -> T) -> Self {
        std::array::from_fn(f)
    }
}

#[derive(Debug, Default, PartialEq, Eq, Serialize, Deserialize, ros_z::Message)]
pub struct Joints<T = f32, S: Scope = k1::FullBody, Storage = <S as Scope>::Array<T>> {
    storage: Storage,
    _phantom: PhantomData<(T, S)>,
}

pub trait Scope {
    type Array<T>: JointsStorage<T>;

    fn map<T, U>(array: Self::Array<T>, f: impl Fn(T) -> U) -> Self::Array<U>;

    fn map_ref<'a, T: 'a, U>(values: &'a Self::Array<T>, f: impl Fn(&'a T) -> U) -> Self::Array<U>;

    fn map_mut<'a, T: 'a, U>(
        values: &'a mut Self::Array<T>,
        f: impl Fn(&'a mut T) -> U,
    ) -> Self::Array<U>;
}

#[macro_export]
macro_rules! impl_scope {
    ($name:ident, $len:expr) => {
        #[derive(
            Debug,
            Clone,
            Copy,
            Default,
            PartialEq,
            Eq,
            ::serde::Deserialize,
            ::serde::Serialize,
            ::ros_z::Message,
        )]
        pub struct $name;

        impl Scope for $name {
            type Array<T> = [T; $len];

            fn map<T, U>(values: Self::Array<T>, f: impl FnMut(T) -> U) -> Self::Array<U> {
                values.map(f)
            }

            fn map_ref<'a, T: 'a, U>(
                values: &'a Self::Array<T>,
                f: impl Fn(&'a T) -> U,
            ) -> Self::Array<U> {
                values.each_ref().map(f)
            }

            fn map_mut<'a, T: 'a, U>(
                values: &'a mut Self::Array<T>,
                f: impl Fn(&'a mut T) -> U,
            ) -> Self::Array<U> {
                values.each_mut().map(f)
            }
        }
    };
}

#[macro_export]
macro_rules! joints_types {
    ($scope:ident, $owned:ident, $borrowed:ident, $mut:ident) => {
        pub type $owned<T = f32> = Joints<T, $scope, <$scope as Scope>::Array<T>>;
        pub type $borrowed<'a, T = f32> = Joints<T, $scope, &'a <$scope as Scope>::Array<T>>;
        pub type $mut<'a, T = f32> = Joints<T, $scope, &'a mut <$scope as Scope>::Array<T>>;
    };
}

pub type BorrowedJoints<'a, T, S> = Joints<T, S, &'a <S as Scope>::Array<T>>;
pub type BorrowedJointsMut<'a, T, S> = Joints<T, S, &'a mut <S as Scope>::Array<T>>;

impl<T, S: Scope> Joints<T, S> {
    #[inline]
    pub fn as_ref(&self) -> BorrowedJoints<'_, T, S> {
        BorrowedJoints {
            storage: &self.storage,
            _phantom: PhantomData,
        }
    }

    #[inline]
    pub fn as_mut(&mut self) -> BorrowedJointsMut<'_, T, S> {
        BorrowedJointsMut {
            storage: &mut self.storage,
            _phantom: PhantomData,
        }
    }

    #[inline]
    pub fn into_array(self) -> <S as Scope>::Array<T> {
        self.storage
    }

    #[inline]
    pub fn map<U>(self, f: impl Fn(T) -> U) -> Joints<U, S> {
        Joints {
            storage: S::map(self.storage, f),
            _phantom: PhantomData,
        }
    }

    #[inline]
    pub fn fill_with(f: impl Fn() -> T) -> Self {
        Self {
            storage: <S as Scope>::Array::<T>::from_fn(|_| f()),
            _phantom: PhantomData,
        }
    }

    #[inline]
    pub fn zip<U>(self, other: Joints<U, S>) -> Joints<(T, U), S> {
        let mut pairs = self.storage.into_iter().zip(other.storage);

        Joints {
            storage: <S as Scope>::Array::<(T, U)>::from_fn(|_| pairs.next().unwrap()),
            _phantom: PhantomData,
        }
    }

    #[inline]
    pub fn all(&self, f: impl FnMut(&T) -> bool) -> bool {
        self.storage.as_ref().iter().all(f)
    }
}

// Owned joints
impl<T, S: Scope<Array<T> = [T; N]>, const N: usize> Joints<T, S, [T; N]> {
    #[inline]
    pub fn from_array(storage: <S as Scope>::Array<T>) -> Self {
        Self {
            storage,
            _phantom: PhantomData,
        }
    }
}

// Borrowed joints
impl<T, S: Scope<Array<T> = [T; N]>, const N: usize> Joints<T, S, &'_ [T; N]> {
    #[inline]
    pub fn from_ref(storage: &<S as Scope>::Array<T>) -> BorrowedJoints<'_, T, S> {
        BorrowedJoints {
            storage,
            _phantom: PhantomData,
        }
    }
}

impl<T: Clone, S: Scope<Array<T> = [T; N]>, const N: usize> Joints<T, S, &'_ [T; N]> {
    pub fn owned(&self) -> Joints<T, S, [T; N]> {
        Joints::from_array(self.storage.clone())
    }
}

impl<T: Clone, S: Scope, const N: usize> Joints<T, S, [T; N]> {
    pub fn fill(value: T) -> Self {
        Self {
            storage: std::array::from_fn(|_| value.clone()),
            _phantom: PhantomData,
        }
    }
}

impl<T, S: Scope> BorrowedJoints<'_, T, S> {
    pub fn map_ref<U>(self, f: impl Fn(&T) -> U) -> Joints<U, S> {
        Joints {
            storage: S::map_ref(self.storage, f),
            _phantom: PhantomData,
        }
    }
}

// Clone and Copy

impl<T: Clone, S: Scope<Array<T> = [T; N]>, const N: usize> Clone for Joints<T, S, [T; N]> {
    fn clone(&self) -> Self {
        Joints::from_array(self.storage.clone())
    }
}

impl<T: Copy, S: Scope<Array<T> = [T; N]>, const N: usize> Copy for Joints<T, S, [T; N]> {}

impl<T, S: Scope<Array<T> = [T; N]>, const N: usize> Clone for Joints<T, S, &'_ [T; N]> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T, S: Scope<Array<T> = [T; N]>, const N: usize> Copy for Joints<T, S, &'_ [T; N]> {}

// Arithmetic ops

impl<T, U, S: Scope> Add<Joints<U, S>> for Joints<T, S>
where
    T: Add<U>,
{
    type Output = Joints<<T as Add<U>>::Output, S>;

    fn add(self, rhs: Joints<U, S>) -> Self::Output {
        self.zip(rhs).map(|(a, b)| a + b)
    }
}

impl<T, U, S: Scope> Sub<Joints<U, S>> for Joints<T, S>
where
    T: Sub<U>,
{
    type Output = Joints<<T as Sub<U>>::Output, S>;

    fn sub(self, rhs: Joints<U, S>) -> Self::Output {
        self.zip(rhs).map(|(a, b)| a - b)
    }
}

impl<T, U, S: Scope> Mul<U> for Joints<T, S>
where
    T: Mul<U>,
    U: Copy,
{
    type Output = Joints<<T as Mul<U>>::Output, S>;

    fn mul(self, rhs: U) -> Self::Output {
        self.map(|x| x * rhs)
    }
}
