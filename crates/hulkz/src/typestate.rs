pub struct Something<T> {
    pub inner: T,
}
pub struct Nothing;

pub trait Settable<T> {}
impl<T> Settable<T> for Something<T> {}
impl<T> Settable<T> for Nothing {}
