/// Shared error bound for all CAN backend errors.
///
/// Backends define their own concrete error types; this trait ensures they are
/// compatible with the standard `Error` trait and are thread-safe.
pub trait CanError: core::error::Error + Send + Sync + 'static {}

/// Blanket implementation: any type satisfying the bounds automatically implements `CanError`.
impl<T: core::error::Error + Send + Sync + 'static> CanError for T {}
