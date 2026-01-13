import time
from typing import Dict
from collections import defaultdict
from fastapi import HTTPException, status
from src.utils.config import get_settings
from src.utils.logging import get_logger

logger = get_logger(__name__)


class RateLimiter:
    """
    Simple in-memory rate limiter to prevent API abuse.
    """

    def __init__(self):
        self.requests = defaultdict(list)  # Maps client IP to list of request timestamps
        self.settings = get_settings()
        self.rate_limit_requests = self.settings.rate_limit_requests
        self.rate_limit_window = self.settings.rate_limit_window  # in seconds
        self.logger = get_logger(__name__)

    def is_allowed(self, client_ip: str) -> bool:
        """
        Check if a request from the given client IP is allowed.

        Args:
            client_ip: IP address of the client

        Returns:
            True if request is allowed, False otherwise
        """
        current_time = time.time()

        # Remove requests older than the time window
        self.requests[client_ip] = [
            req_time for req_time in self.requests[client_ip]
            if current_time - req_time < self.rate_limit_window
        ]

        # Check if the number of requests is within the limit
        if len(self.requests[client_ip]) < self.rate_limit_requests:
            # Add current request
            self.requests[client_ip].append(current_time)
            return True

        # Rate limit exceeded
        self.logger.warning(f"Rate limit exceeded for IP: {client_ip}")
        return False

    def get_reset_time(self, client_ip: str) -> float:
        """
        Get the time when the rate limit will reset for the given IP.

        Args:
            client_ip: IP address of the client

        Returns:
            Unix timestamp when the rate limit will reset
        """
        if not self.requests[client_ip]:
            return 0

        oldest_request = min(self.requests[client_ip])
        reset_time = oldest_request + self.rate_limit_window
        return reset_time


# Global rate limiter instance
_rate_limiter = None


def get_rate_limiter() -> RateLimiter:
    """
    Get the global rate limiter instance.

    Returns:
        RateLimiter instance
    """
    global _rate_limiter
    if _rate_limiter is None:
        _rate_limiter = RateLimiter()
    return _rate_limiter


def check_rate_limit(client_ip: str):
    """
    Check if the request is within the rate limit. Raises HTTPException if exceeded.

    Args:
        client_ip: IP address of the client

    Raises:
        HTTPException: If rate limit is exceeded
    """
    rate_limiter = get_rate_limiter()

    if not rate_limiter.is_allowed(client_ip):
        reset_time = rate_limiter.get_reset_time(client_ip)
        retry_after = int(reset_time - time.time())

        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail={
                "error": "Rate limit exceeded",
                "retry_after_seconds": max(0, retry_after)
            }
        )


if __name__ == "__main__":
    # Example usage
    limiter = get_rate_limiter()

    # Simulate requests from an IP
    client_ip = "192.168.1.1"

    for i in range(5):
        if limiter.is_allowed(client_ip):
            print(f"Request {i+1} from {client_ip} is allowed")
        else:
            print(f"Request {i+1} from {client_ip} is rate limited")