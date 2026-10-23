use std::fmt;

/// Checked construction, integration and user-supplied model errors.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// A configuration, state or command violates the public input contract.
    InvalidInput {
        /// Invalid field or group of fields.
        field: &'static str,
        /// Description of the required invariant.
        reason: &'static str,
    },
    /// The requested duration exceeds the configured internal work budget.
    StepLimitExceeded {
        /// Maximum permitted RK4 substeps per call.
        limit: usize,
    },
    /// Numerical overflow, non-finite model output or inability to advance time.
    NumericalFailure(&'static str),
    /// An error reported by an environmental model or guidance implementation.
    Model(String),
}

impl Error {
    pub(crate) fn invalid(field: &'static str, reason: &'static str) -> Self {
        Self::InvalidInput { field, reason }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput { field, reason } => write!(f, "{field}: {reason}"),
            Self::StepLimitExceeded { limit } => write!(f, "integration exceeded {limit} substeps"),
            Self::NumericalFailure(reason) => write!(f, "integration failed: {reason}"),
            Self::Model(message) => write!(f, "model failed: {message}"),
        }
    }
}

impl std::error::Error for Error {}

/// Result type shared by the solver and extension hooks.
pub type Result<T> = std::result::Result<T, Error>;
