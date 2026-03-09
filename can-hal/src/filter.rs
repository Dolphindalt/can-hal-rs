use crate::error::CanError;
use crate::id::CanId;

/// A hardware acceptance filter defined by an ID and a mask.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Filter {
    pub id: CanId,
    pub mask: u32,
}

/// Hardware acceptance filtering.
///
/// **Important**: The exact semantics of multiple filters depend on the backend.
/// Some hardware (e.g. SocketCAN) supports multiple independent filters
/// (union — a frame passes if it matches *any* filter). Other hardware
/// (e.g. PCAN, Kvaser) only supports a single filter pair per frame type
/// (standard / extended), so multiple filters must be merged into one, which
/// may accept a broader range of IDs than intended.
///
/// For portable code that needs precise multi-ID filtering, consider using a
/// single permissive hardware filter and applying software-level filtering on
/// received frames.
pub trait Filterable {
    type Error: CanError;

    /// Apply the given set of acceptance filters.
    ///
    /// Replaces any previously configured filters. An empty slice is equivalent
    /// to calling [`clear_filters`](Self::clear_filters).
    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), Self::Error>;

    /// Remove all acceptance filters (accept everything).
    fn clear_filters(&mut self) -> Result<(), Self::Error>;
}
