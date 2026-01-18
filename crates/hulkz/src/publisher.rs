use std::marker::PhantomData;

use cdr::{CdrLe, Infinite};
use serde::Serialize;
use zenoh::pubsub::Publisher as ZenohPublisher;

use crate::{
    typestate::{Nothing, Settable, Something},
    Session,
};

#[derive(Debug, thiserror::Error)]
pub enum PublisherError {
    #[error("Failed to serialize data: {0}")]
    Serialization(#[from] cdr::Error),
    #[error("Zenoh transport error: {0}")]
    Zenoh(#[from] zenoh::Error),
}

pub type Result<T, E = PublisherError> = std::result::Result<T, E>;

pub struct Publisher<'a, T>
where
    T: Serialize,
{
    publisher: ZenohPublisher<'a>,
    _phantom: PhantomData<T>,
}

impl<'a, T> Publisher<'a, T>
where
    T: Serialize,
{
    pub fn new(publisher: ZenohPublisher<'a>) -> Self {
        Self {
            publisher,
            _phantom: PhantomData,
        }
    }

    pub fn builder() -> PublisherBuilder<T> {
        PublisherBuilder {
            key: Nothing,
            _phantom: PhantomData,
        }
    }

    pub async fn is_subscribed(&self) -> Result<bool> {
        let status = self
            .publisher
            .matching_status()
            .await
            .map_err(PublisherError::Zenoh)?;
        Ok(status.matching())
    }

    #[tracing::instrument(skip(self, value), level = "debug", err)]
    pub async fn put(&self, value: &T) -> Result<()> {
        let payload = cdr::serialize::<_, _, CdrLe>(value, Infinite)
            .map_err(PublisherError::Serialization)?;

        self.publisher.put(payload).await?;
        Ok(())
    }

    pub async fn put_with_subscription(&self, mut value: impl FnMut() -> T) -> Result<()> {
        if self.is_subscribed().await? {
            let value = value();
            self.put(&value).await?;
        }
        Ok(())
    }
}

pub struct PublisherBuilder<T, KeyState: Settable<String> = Nothing> {
    key: KeyState,
    _phantom: PhantomData<T>,
}

impl<T> PublisherBuilder<T, Nothing>
where
    T: Serialize,
{
    pub fn new() -> Self {
        Self {
            key: Nothing,
            _phantom: PhantomData,
        }
    }

    pub fn key(self, key: impl Into<String>) -> PublisherBuilder<T, Something<String>> {
        PublisherBuilder {
            key: Something { inner: key.into() },
            _phantom: PhantomData,
        }
    }
}

impl<T> Default for PublisherBuilder<T, Nothing>
where
    T: Serialize,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Serialize> PublisherBuilder<T, Something<String>> {
    pub async fn build(self, session: &Session) -> Result<Publisher<'_, T>> {
        let publisher = session.session.declare_publisher(self.key.inner).await?;

        Ok(Publisher {
            publisher,
            _phantom: PhantomData,
        })
    }
}
