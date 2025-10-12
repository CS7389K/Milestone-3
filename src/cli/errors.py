class TeleopError(Exception):
    """Base class for teleop errors."""

class ValidationError(TeleopError):
    """Raised when a command or value is invalid."""

class BackendError(TeleopError):
    """Raised when the backend (ROS/SDK) call fails."""